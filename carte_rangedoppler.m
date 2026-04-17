function carte_rangedoppler()
    % Traitement Range-Doppler pour un instant précis (target time)
    % Approche Bistatique Réelle : On synchronise sur le trajet direct théorique.
    
    clc; close all;
    disp('=== DÉMARRAGE SIMULATION CARTE RANGE-DOPPLER ===');

    
    %% 1. Config
    [waveform, transmitter, receiver, radiator, collector] = config_radar_system_isotrope();
    fc = radiator.OperatingFrequency;
    fs = waveform.SampleRate;
    prf = waveform.PRF; 
    c = 3e8;
    
    % On récupère le filtre adapté (pour la corrélation manuelle)
    matching_coeff = getMatchedFilter(waveform);

    
    %% 2. Scène & Canaux
    
    % Scène : appel de la fonction de génération du scénario
    [scene, Tx, Rx, Tgt] = init_scenario();
    
    % Canaux
    channel_sat_to_tgt = phased.FreeSpace('SampleRate',fs,'TwoWayPropagation',false,'OperatingFrequency',fc);
    channel_tgt_to_rx  = phased.FreeSpace('SampleRate',fs,'TwoWayPropagation',false,'OperatingFrequency',fc);

    
    %% 3. Simulation
    
    % Paramètres de simulation
    target_time = 20;
    rcs_avion = 10e6; 


    disp(['Avance rapide jusqu''à t = ' num2str(target_time) 's...']);

    while advance(scene)
        current_time = scene.SimulationTime;
        
        if current_time >= target_time
            
            disp(['--- TIR RAFALE À T = ' num2str(current_time) 's ---']);
            
            % Positions et Vitesses
            pos_tx = pose(Tx).Position.';  vel_tx = pose(Tx).Velocity.';  
            pos_rx = pose(Rx).Position.';  vel_rx = pose(Rx).Velocity.';  
            pos_tgt = pose(Tgt).Position.'; vel_tgt = pose(Tgt).Velocity.'; 
            
            

            % --- CALCUL DE LA RÉFÉRENCE BISTATIQUE ---
            [Rbist, Vbist] = ref_bist(pos_tx, pos_tgt, pos_rx, vel_tgt);
            time_direct = norm(pos_tx - pos_rx) / c;
            
            fprintf('>>> DISTANCE BISTATIQUE REELLE : %.3f km\n', Rbist/1000);
            fprintf('>>> VITESSE BISTATIQUE REELLE : %.3f km/h\n', Vbist * 3.6);

            % 5. Calcul des résolutions
            resol_dist_bist = c/fs;
            resol_vitesse_bist = (c*prf)/(fc*waveform.NumPulses);

            fprintf('>>> Résolution Distance Bistatique : %.3f km\n', resol_dist_bist/1000);
            fprintf('>>> Résolution Vitesse Bistatique : %.3f km/h\n', resol_vitesse_bist * 3.6);



            % --- TIR & PROPAGATION ---
            sig_burst = waveform(); 
            tx_sig = transmitter(sig_burst);
            
            % Propagation (Sat -> Cible -> Sol)
            % Note : On ne simule PAS le canal direct ici (God Mode), on sait juste quand il arriverait.
            [~, ang_tx_vers_tgt] = rangeangle(pos_tgt, pos_tx);
            sig_radie = radiator(tx_sig, ang_tx_vers_tgt);
            sig_at_tgt = channel_sat_to_tgt(sig_radie, pos_tx, pos_tgt, vel_tx, vel_tgt);
            sig_reflechi = sig_at_tgt * sqrt(rcs_avion);
            sig_at_rx = channel_tgt_to_rx(sig_reflechi, pos_tgt, pos_rx, vel_tgt, vel_rx);
            [~, ang_rx_vers_tgt] = rangeangle(pos_tgt, pos_rx); 
            sig_collecte = collector(sig_at_rx, ang_rx_vers_tgt);
            
            % Réception Brute (Le récepteur enregistre à partir de t=0 d'émission)
            rx_raw = receiver(sig_collecte);

            % ATTENTION : waveform ne prend pas en compte le Doppler du satellite, présent dans le signal indirect => non retiré dans rx_raw
            v_sat_radiale = dot(vel_tx, -((pos_tx - pos_tgt) /norm(pos_tx - pos_tgt)));
            fd_sat = v_sat_radiale * (fc / c);
            t_raw = (0:length(rx_raw)-1).' / fs;   % échelle de temps : origine = émission rx_raw, en Stop & Hop tvol = constant -> aucun impact FFT / Réalité -> Migrations
            rx_raw = rx_raw .* exp(-1i * 2 * pi * fd_sat * t_raw);   % signal brut corrigé (sans doppler satellite)
            %rx_raw est un vecteur comportant toutes les impulsions 



            % --- TRAITEMENT SIGNAL ---
            
            % 1. Filtre Adapté sur le signal brut
            rx_filtered_raw = filter(matching_coeff, 1, rx_raw);
            % matching_coeff = waveform inversé (temporellement) & conjugué
            % 1 => FIR : filtre à réponse impulsionnelle (convolution signal avec réponse impulsionnelle)

            % 2. Suppression du retard du filtre / Prb algorithmique du filtre (déclaration pic d'énergie lors de la fin du train d'impulsion)
            delay_samples = round(waveform.PulseWidth * fs); % 30 km en nb d'échantillons
            rx_filtered_raw = rx_filtered_raw(delay_samples + 1 : end); % On rabote le début (signal obsolète)
             
            % 3. Reshape en Matrice (Fast Time x Slow Time)
            n_samples_pulse = round(fs / prf);         % Nombre d'échantillons pour 1 cycle (PRI), axe vertical Fast Time (range)
            n_pulses = waveform.NumPulses;             % axe horizontal Slow Time (Doppler)
            len_needed = n_samples_pulse * n_pulses;   % Nombre total de points (distance <-> vitesse)

            if length(rx_filtered_raw) < len_needed 
                rx_filtered_raw = [rx_filtered_raw; zeros(len_needed-length(rx_filtered_raw),1)];   % Compléter rx_filtered_raw -> taille originale
            end   

            % Création Matrice Radar : ième colonne = ième tir / Conversion vecteur 1D -> 2D
            compressed_matrix = reshape(rx_filtered_raw(1:len_needed), n_samples_pulse, n_pulses);      

            % 4. FFT Doppler (Sur les lignes)
            rd_map = fftshift(fft(compressed_matrix, [], 2), 2);   % fftshift : freq nulle au milieu & freq négative à gauche
            % rd_map = matrice taille désirée (contraintes ambiguïtés), case = nombre complexe 


            
            % Création de l'axe temps absolu
            dt = 1/fs;                                % durée d'un échantillon
            time_axis = (0:n_samples_pulse-1) * dt;   % liste de temps échntillonné de 0 à PRI
            % (dt*n_samples_pulse = pri)
            % Remarque : pri/n_samples_pulse = 1/fs = 1/2B
            % résolution Rbist = c/2B
            
            % On calcule le retard du signal direct MODULO la PRI (-> récepteur écoute cycliquement : chaque émission)
            pri = 1/prf;
            time_direct_folded = mod(time_direct, pri);   % liste de temps compris entre [0; PRI]
            
            % Liste Axe Distance Bistatique (contrainte = PRI : distance bistatique max d'où borne supp d'affichage)
            range_axis = c * (time_axis - time_direct_folded)/1000;   
             
            
            % Liste Axe Freq & Vitesse (Standard FFT)
            % (contrainte = 1/2*pri = prf/2 d'où borne supp d'affichage)
            dop_axis = (-n_pulses/2 : n_pulses/2 - 1) * (prf / n_pulses);   % Résoltion Doppler = 1/Tint = 1/(pri*n_pulses) = (prf/n_pulses)
            speed_axis = dop_axis * (c/fc) * 3.6;                                 % liste des freq * longueur d'onde 
            


            %% 4. --- AFFICHAGE ---
            
            figure('Name', 'Carte Bistatique Custom', 'Color', 'w');
            imagesc(speed_axis, range_axis, 20*log10(abs(rd_map)));
            axis xy; colormap('jet'); colorbar;
            
            xlabel('Vitesse Doppler (km/h)');
            ylabel('Distance Bistatique (km)');
            title('Carte Range-Doppler Bistatique (Synchronisée sur Direct)');
            
            % Paramètres d'affichage spécifiques :
            
            %ylim([0 30]);   % Restreindre l'affichage
            
            % Ligne Théorique
            h = yline(Rbist/1000, 'b--', 'Range Théorique');
            h.LabelHorizontalAlignment = 'left';
            xline(Vbist * 3.6, 'b--', 'Vitesse Théorique');
            
            % Distance et Vitesse Bistatique du max de puissance
            [~, idx] = max(rd_map(:));
            [i, j] = ind2sub(size(rd_map), idx);

            dist_rd = range_axis(i);
            vit_rd  = speed_axis(j);

            fprintf('Distance Bistatique RD map : %.3f km\n', dist_rd);
            fprintf('Vitesse Bistatique RD map : %.3f km/h\n', vit_rd);
            hold on
            plot(vit_rd, dist_rd, 'kx', 'LineWidth', 2, 'MarkerSize', 10)

            disp('Traitement terminé.');
            break;
        end
    end
end
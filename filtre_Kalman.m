function [Liste_erreur, Liste_X_postM, L_X_reel, time_min, N_steps, dt] = filtre_Kalman()

    clc; close all;
    disp('=== DÉMARRAGE : TRACKING ===');

    %% 1. Initialisation des paramètres de la simulation
    
    % 1. Scène : appel de la fonction de génération du scénario
    [scene, Tx, Rx, Tgt] = init_scenario();

    % 2. Paramètres de Simulation
    time_min = 10;
    time_max = 40;
    dt = 1/scene.UpdateRate();                         % Pas de temps des mesures/avancement  
    pos_initiale_tgt = [];
    N_steps = round((time_max - time_min) / dt) + 1;   % Calcul du nombre d'états attendus
    idx_mesure = 1;                                    % Index manuel remplissage des cases

    % 3. Pré-allocation de la mémoire 
    L_Rbist = zeros(1, N_steps);
    L_Vbist  = zeros(1, N_steps);
    L_azimut    = zeros(1, N_steps);
    L_elevation = zeros(1, N_steps);
    L_pos_tx    = zeros(3, N_steps);
    L_pos_rx    = zeros(3, N_steps);
    L_X_reel    = zeros(6, N_steps);

    
    
    %% 2. Lancement du scénario sur la plage de temps défini
    
    disp('Lancement calcul des paramètres bistatiques et mesures bruitées');

    while advance(scene)
        current_time = scene.SimulationTime;

        if (round(current_time, 3) >= time_min) && (round(current_time, 3) <= time_max)

            % 1. Positions et Vitesses
            pos_tx = pose(Tx).Position.';    
            pos_rx = pose(Rx).Position.';    
            pos_tgt = pose(Tgt).Position.'; 
            vel_tgt = pose(Tgt).Velocity.';

            % 2. Calcul de la référence bistatique
            [Rbist, Vbist] = ref_bist(pos_tx, pos_tgt, pos_rx, vel_tgt);
            
            % 3. Calcul de l'azimut et de l'élévation 
            [~, angles_vrai] = rangeangle(pos_tgt, pos_rx); % DOA réelle (degrés)
            azimut_vrai = angles_vrai(1);
            elevation_vrai = angles_vrai(2);
            
            % 4. Création du bruit de mesure -> Nombre aléatoire d'une loi Gaussienne d'espérance nulle
            bruit_range = randn() * 15;         % Erreur de +/- 15 mètres  
            bruit_vitesse = randn() * 2.13;     % Erreur de +/- 2.13 m/s
            bruit_azimut = randn() * 1.5;       % Erreur de +/- 1.5 degrés
            bruit_elevation = randn() * 1.5;    % Erreur de +/- 1.5 degrés
            % On utilise la résolution comme écart-type : erreur dominée par le bruit de quantification (pas d'interpolation sub-pixel = f(SNR)) 
            
            % 5. Input filtre de Kalman : réplication des mesures carte Range-Doppler
            mesure_Rbist = Rbist + bruit_range;
            mesure_Vbist = Vbist + bruit_vitesse;
            mesure_azimut = azimut_vrai + bruit_azimut;
            mesure_elevation = elevation_vrai + bruit_elevation;

            
            % 6. Affectation dans les cases allouées
            L_Rbist(idx_mesure) = mesure_Rbist;
            L_Vbist(idx_mesure)  = mesure_Vbist;
            L_azimut(idx_mesure)    = mesure_azimut;
            L_elevation(idx_mesure) = mesure_elevation;
            L_pos_tx(:, idx_mesure) = pos_tx;
            L_pos_rx(:, idx_mesure) = pos_rx;
            L_X_reel(:, idx_mesure) = [pos_tgt; vel_tgt];


            % 7. --- INITIALISATION DE LA PISTE KALMAN (TRACK INITIATION) ---
            if isempty(pos_initiale_tgt)
                 
                % A. Fonction inverse : Création du vecteur directeur d'observation
                u_x = cosd(mesure_elevation) * cosd(mesure_azimut);
                u_y = cosd(mesure_elevation) * sind(mesure_azimut);
                u_z = sind(mesure_elevation);
                u_vec = [u_x; u_y; u_z];
                
                % B. Intersection géométrique algébrique (Droite vs Ellipsoïde)
                P_tx_rel = pos_tx - pos_rx;
                L_dir = norm(P_tx_rel);
                K = mesure_Rbist + L_dir;
                
                numerateur = (L_dir^2) - (K^2);
                denominateur = 2 * (dot(P_tx_rel, u_vec) - K);
                
                % Distance radiale induite par les fausses mesures
                d_rx = numerateur / denominateur;
                
                % C. Point de départ cartésien (Décentré de la vraie cible à cause du bruit)
                pos_XYZ_0 = pos_rx + d_rx * u_vec;
                
                % D. Création de l'État Initial (Vitesses supposées nulles)
                pos_initiale_tgt = [pos_XYZ_0; 0; 0; 0];
                
                disp(['Piste initialisée à t = ' num2str(current_time)]);
            end
            
            % 8. Avancement du curseur pour le prochain tour
            idx_mesure = idx_mesure + 1;

       end
    end



    %% 3. --- INITIALISATION PARAMETRES FILTRE KALMAN ---
    
    % Matrice de covariance sur l'état initial : indépendance des erreurs entre grandeurs
    % Variances sur x, y, z, vx, vy, vz de l'état initial
    P = eye(6) * 250000; 
    P(4,4) = 90000; P(5,5) = 90000; P(6,6) = 90000;
    
    % Liste des états post mesures
    Liste_X_postM = zeros(6, N_steps);
    Liste_X_postM(:, 1) = pos_initiale_tgt; 
    
    % Utilisation d'un "Cell Array" pour stocker les matrices de covariance 6x6
    Liste_P_postM = cell(1, length(L_Rbist)); 
    Liste_P_postM{1} = P;                          
    
    % Matrice de Transition 
    F = [1 0 0 dt 0 0;                          
         0 1 0 0 dt 0;
         0 0 1 0 0 dt;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1];
    
    % Ecart-type maximale sur l'accélération
    sigm_a = 0.1;  

    % Bruit de processus (vent, changement de direction, ...)
    % DWNA (Discrete White Noise Acceleration) : accélération 'a' subit lors de dt -> Q = variance sur les positions et vitesses résultant de 'a'  
    Q = sigm_a^2 * [dt^4/4   0        0        dt^3/2   0        0;         
                    0        dt^4/4   0        0        dt^3/2   0;
                    0        0        dt^4/4   0        0        dt^3/2;
                    dt^3/2   0        0        dt^2     0        0;
                    0        dt^3/2   0        0        dt^2     0;
                    0        0        dt^3/2   0        0        dt^2];
    
    % Bruit de mesure : erreurs indépendantes
    R = [15^2 0      0     0;                   
         0    2.13^2 0     0;
         0    0      1.5^2 0;
         0    0      0     1.5^2];
    
    % Liste des erreurs entre les états post Kalman et les états réels
    Liste_erreur = zeros(6, length(L_Rbist)); 
    
    
    %% 4. --- BOUCLE DU FILTRE DE KALMAN ---
    
    disp('Itérations Filtre Kalman');
    
    for i = 2:length(L_Rbist)
        
        % 1. PRÉDICTION
        X_prev = Liste_X_postM(:, i-1); % Extraction de la colonne entière
        P_prev = Liste_P_postM{i-1};    % Extraction de la matrice avec {}
        
        X_preM = F * X_prev;
        P_preM = F * P_prev * (F.') + Q;
        
        % 2. OBSERVATION
        % Extraction des vecteurs 3D complets avec (:, i)
        [h, H] = h_et_H(X_preM, L_pos_tx(:, i), L_pos_rx(:, i));
        Z = [L_Rbist(i); L_Vbist(i); L_azimut(i); L_elevation(i)];
        
        % 3. CORRECTION (Calcul du Gain K)
        S = H * P_preM * (H.') + R;
        K = (P_preM * (H.'))/S; 
        
        X_postM = X_preM + K * (Z - h);
        P_postM = (eye(6) - K * H) * P_preM; 
        
        % 4. STOCKAGE
        Liste_X_postM(:, i) = X_postM; 
        Liste_P_postM{i} = P_postM;
        
        % 5. CALCUL DE L'ERREUR INSTANTANÉE
        Liste_erreur(:, i) = L_X_reel(:, i) - X_postM;
    end
    
    


end










    
    

    
    
    

               





function visu_scenario_2D(scene, Tx, Rx, Tgt)
    
    % PROTOCOLE USAGE :
    % Taper dans la commande :
    % [scene, Tx, Rx, Tgt] = init_scenario(); visu_scenario_2D(scene, Tx, Rx, Tgt);

    disp('Génération de la carte de validation...');
    figure('Name', 'Scénario OneWeb', 'Color', 'w');
    
    % Récupération des trajectoires depuis les plateformes
    % (C'est ce qui permet de rendre cette fonction indépendante)
    traj_tx = Tx.Trajectory;
    traj_tgt = Tgt.Trajectory;
    
    % On récupère la position du Rx (1er waypoint)
    % Note: Rx.Trajectory est aussi un geoTrajectory
    [pos_rx, ~] = lookupPose(Rx.Trajectory, 0); 
    lat_rx = pos_rx(1);
    lon_rx = pos_rx(2);

    % 1. On récupère les points de passage pour le tracé
    t_plot = linspace(0, scene.StopTime, 100);
    
    % Extraction des positions Lat/Lon/Alt
    [pos_sat_geo, ~] = lookupPose(traj_tx, t_plot);
    [pos_tgt_geo, ~] = lookupPose(traj_tgt, t_plot);
    
    lat_sat_trace = pos_sat_geo(:,1);
    lon_sat_trace = pos_sat_geo(:,2);
    
    lat_tgt_trace = pos_tgt_geo(:,1);
    lon_tgt_trace = pos_tgt_geo(:,2);

    % 2. Tracé sur la carte
    gx = geoaxes;
    hold(gx, 'on');
    
    % Trace Satellite (Bleu)
    geoplot(gx, lat_sat_trace, lon_sat_trace, 'b-', 'LineWidth', 2, 'DisplayName', 'Satellite');
    geoplot(gx, lat_sat_trace(1), lon_sat_trace(1), 'b^', 'MarkerFaceColor', 'b'); 
    
    % Trace Avion (Rouge)
    geoplot(gx, lat_tgt_trace, lon_tgt_trace, 'r-', 'LineWidth', 2, 'DisplayName', 'Cible Avion');
    geoplot(gx, lat_tgt_trace(1), lon_tgt_trace(1), 'r>', 'MarkerFaceColor', 'r'); 
    
    % Position Station Sol (Vert)
    geoplot(gx, lat_rx, lon_rx, 'gp', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'DisplayName', 'Station Sol');
    
    % 3. Esthétique Pro
    legend(gx, 'Location', 'best');
    title(gx, 'Scénario Radar Passif : Croisement OneWeb / Cible');
    
    try
        geobasemap(gx, 'satellite');
    catch
        geobasemap(gx, 'streets');
    end
    
    % Zoom auto
    geolimits(gx, [lat_rx-2 lat_rx+2], [lon_rx-2 lon_rx+2]);
    
    disp('Carte affichée.');
end
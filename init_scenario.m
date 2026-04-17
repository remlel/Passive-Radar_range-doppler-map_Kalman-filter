function [scene, Tx, Rx, Tgt] = init_scenario()
    % 1. Initialisation
    
    % Création du scénario radar (Physique)
    % On garde la configuration Terre Centrée qui fonctionne bien chez vous
    scene = radarScenario('IsEarthCentered', true, ...
                          'UpdateRate', 10, ...
                          'StopTime', 60);

    % 2. Paramètres Physiques
    Re = 6371e3;          
    h_sat = 1200e3;       
    R_orbite = Re + h_sat; 
    
    v_sat = 7500;         
    h_tgt = 10000;        
    v_tgt = 250;          

    % Position Rx (Toulouse)
    lat_rx = 43.6047; 
    lon_rx = 1.4442;
    alt_rx = 0;

    % 3. Calcul des Trajectoires
    T_mid = scene.StopTime / 2; 

    % --- Satellite (Nord-Sud) ---
    dist_mi_parcours = v_sat * T_mid; 
    delta_angle_rad = dist_mi_parcours / R_orbite;
    delta_angle_deg = rad2deg(delta_angle_rad);
    
    lat_start = lat_rx - delta_angle_deg;
    lat_end   = lat_rx + delta_angle_deg;
    
    % Trajectoire Satellite
    traj_tx = geoTrajectory('Waypoints', [lat_start, lon_rx, h_sat; ...
                                          lat_end,   lon_rx, h_sat], ...
                            'TimeOfArrival', [0; scene.StopTime]);

    % --- Avion (Ouest-Est) ---
    dist_tgt_mi = v_tgt * T_mid;
    delta_lon_rad = dist_tgt_mi / ((Re+h_tgt) * cosd(lat_rx)); 
    delta_lon_deg = rad2deg(delta_lon_rad);
    
    lon_tgt_start = lon_rx - delta_lon_deg;
    lon_tgt_end   = lon_rx + delta_lon_deg;
    
    % Trajectoire Avion
    traj_tgt = geoTrajectory('Waypoints', [lat_rx, lon_tgt_start, h_tgt; ...
                                           lat_rx, lon_tgt_end,   h_tgt], ...
                             'TimeOfArrival', [0; scene.StopTime]);

    % --- Station Sol (Statique) ---
    % Trajectoire Rx (Points identiques pour forcer le statique)
    traj_rx = geoTrajectory('Waypoints', [lat_rx, lon_rx, alt_rx; ...
                                          lat_rx, lon_rx, alt_rx], ...
                            'TimeOfArrival', [0; scene.StopTime]);

    % 4. Création des Plateformes (Syntaxe Robuste)
    % On utilise la syntaxe minimale qui marche sur toutes les versions R2021+
    Tx = platform(scene, 'Trajectory', traj_tx);
    Rx = platform(scene, 'Trajectory', traj_rx);
    Tgt = platform(scene, 'Trajectory', traj_tgt);

    
end
function Affichage_Positions_Kalman_vs_Reelles()

    % Appelle fonction filtre de Kalman
    [~, Liste_X_postM, L_X_reel, ~, ~, ~] = filtre_Kalman();

    % --- AFFICHAGE DE LA TRAJECTOIRE 3D ---
    figure('Name', 'Trajectoire Spatiale 3D');
    
    % Trajectoire Réelle (Verte)
    plot3(L_X_reel(1,:), L_X_reel(2,:), L_X_reel(3,:), 'g', 'LineWidth', 2); hold on;
    
    % Trajectoire Kalman (Rouge pointillée)
    plot3(Liste_X_postM(1,:), Liste_X_postM(2,:), Liste_X_postM(3,:), 'r--', 'LineWidth', 1.5);
    
    % Mise en évidence des points de DÉPART (Pour voir l'erreur initiale)
    scatter3(L_X_reel(1,1), L_X_reel(2,1), L_X_reel(3,1), 100, 'go', 'filled'); % Départ Vrai
    scatter3(Liste_X_postM(1,1), Liste_X_postM(2,1), Liste_X_postM(3,1), 100, 'rx', 'LineWidth', 2); % Départ Kalman
    
    title('Fusion des trajectoires (Track Initiation)');
    xlabel('Position X ECEF / 0°, 0° (m)'); ylabel('Position Y ECEF / Est (m)'); zlabel('Position Z ECEF / Nord (m)');
    legend('Vérité Terrain', 'Estimation Kalman', 'Départ Réel', 'Départ Kalman (Bruit)');
    grid on; view(3); % Active la vue 3D



end
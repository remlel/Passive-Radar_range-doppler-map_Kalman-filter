function Affichage_Erreurs_filtre_Kalman()
    
    % Appelle fonction filtre de Kalman
    [Liste_erreur, ~, ~, time_min, N_steps, dt] = filtre_Kalman();

    % --- AFFICHAGE DES ERREURS ---

    temps_axe = time_min + (0:N_steps-1) * dt;

    figure('Name', 'Erreurs de Tracking Kalman');
    
    % --- Graphe 1 : Erreur de Position ---
    subplot(2,1,1);
    plot(temps_axe, Liste_erreur(1,:), 'r', 'LineWidth', 1.5); hold on;
    plot(temps_axe, Liste_erreur(2,:), 'g', 'LineWidth', 1.5);
    plot(temps_axe, Liste_erreur(3,:), 'b', 'LineWidth', 1.5);
    title('Erreur de Position (m)');
    xlabel('Temps de simulation (s)');
    ylabel('Erreur (m)');
    legend('Erreur X', 'Erreur Y', 'Erreur Z');
    grid on;
    
    % --- Graphe 2 : Erreur de Vitesse ---
    subplot(2,1,2);
    plot(temps_axe, Liste_erreur(4,:) * 3.6, 'r', 'LineWidth', 1.5); hold on;
    plot(temps_axe, Liste_erreur(5,:) * 3.6, 'g', 'LineWidth', 1.5);
    plot(temps_axe, Liste_erreur(6,:) * 3.6, 'b', 'LineWidth', 1.5);
    title('Erreur de Vitesse (km/h)');
    xlabel('Temps de simulation (s)');
    ylabel('Erreur (km/h)');
    legend('Erreur Vx', 'Erreur Vy', 'Erreur Vz');
    %yline(10, 'b--');
    %yline(-10, 'b--');
    grid on;


end 
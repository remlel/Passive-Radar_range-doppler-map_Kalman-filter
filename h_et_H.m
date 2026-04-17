function [h, H] = h_et_H(X, pos_tx, pos_rx)
    
    %% 1. DÉBALLAGE DE L'ÉTAT
    x = X(1);  y = X(2);  z = X(3);
    vx = X(4); vy = X(5); vz = X(6);
    
    pos_tgt = [x; y; z];
    vel_tgt = [vx; vy; vz];
    
    %% 2. VECTEURS ET DISTANCES
    
    dist_tx = norm(pos_tx - pos_tgt);
    dist_rx = norm(pos_rx - pos_tgt);
    dist_direct = norm(pos_tx - pos_rx);
    
    u_tx = (pos_tx - pos_tgt) / dist_tx;
    u_rx = (pos_rx - pos_tgt) / dist_rx;
    
    %% 3. CALCUL DE L'OBSERVATION (h)
    
    Rbist = dist_tx + dist_rx - dist_direct;
    Vbist = dot(vel_tgt, u_tx + u_rx); 

    [~, angles] = rangeangle(pos_tgt, pos_rx);
    Az = angles(1);
    El = angles(2);
    
    % Vecteur de mesure théorique final
    h = [Rbist; Vbist; Az; El];
    
    %% 4. CALCUL DE LA JACOBIENNE (H) PAR DIFFÉRENCES FINIES
    % Initialisation de la matrice H (4 lignes de mesures x 6 colonnes d'état)
    H = zeros(4, 6);
    
    % Décalage minim pour calculer la pente 
    delta = 1e-5; 
    
    % Calcul des dérivées pour chacune des 6 variables d'état (colonne par colonne)
    for i = 1:6
        X_perturbe = X;
        X_perturbe(i) = X_perturbe(i) + delta; % Perturbation sur la ième variable
        
        % Implémentation de la distance et vitesse perturbées
        pos_p = X_perturbe(1:3);
        vel_p = X_perturbe(4:6);
        
        % Calcule des mesures avec la perturbation
        dtx_p = norm(pos_tx - pos_p);
        drx_p = norm(pos_rx - pos_p);
        utx_p = (pos_tx - pos_p) / dtx_p;
        urx_p = (pos_rx - pos_p) / drx_p;
        
        R_p = dtx_p + drx_p - dist_direct;
        V_p = dot(vel_p, utx_p + urx_p);
        [~, ang_p] = rangeangle(pos_p, pos_rx);
        
        h_perturbe = [R_p; V_p; ang_p(1); ang_p(2)];
        
        % Taux de variation -> i-ème colonne de la Jacobienne
        H(:, i) = (h_perturbe - h) / delta;
    end
end
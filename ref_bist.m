function [Rbist, Vbist] = ref_bist(pos_tx, pos_tgt, pos_rx, vel_tgt)
    
    % Calcul de la distance bistatique
    dist_direct = norm(pos_tx - pos_rx);
    dist_sat_tgt = norm(pos_tx - pos_tgt);
    dist_tgt_rx = norm(pos_tgt - pos_rx);
    dist_total = dist_sat_tgt + dist_tgt_rx;
    Rbist = dist_total - dist_direct;

    % Calcul de la vitesse bistatique
    u_tx = (pos_tx - pos_tgt) / dist_sat_tgt;
    u_rx = (pos_rx - pos_tgt) / dist_tgt_rx;
    Vbist = dot(vel_tgt, u_rx + u_tx);

end
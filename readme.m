%% ================================================================
%  📡 PASSIVE RADAR SIMULATION – PROJECT OVERVIEW
%  Simulation d'un radar passif utilisant un satellite LEO (type OneWeb)
%  comme illuminateur et un EKF pour la poursuite de cible.
%  ================================================================

%% 1. HYPOTHÈSES PHYSIQUES

% - Modèle de propagation : "Stop & Hop"
%   La scène est figée pendant chaque impulsion.
%   → Le mouvement intra-impulsion de la cible est négligé.

% - Modélisation du canal :
%   Séparation des trajets :
%     * Satellite → Cible
%     * Cible → Récepteur
%   → Le trajet direct (Satellite → Récepteur) et le clutter sol sont ignorés.

% - Modèle Terre :
%   Terre sphérique en repère ECEF.

%% 2. SIMPLIFICATIONS DU SYSTÈME

% - Antenne :
%   Modèle isotrope avec gain constant de 30 dB.
%   → Pas de gestion du pointage de faisceau
%   → Simplification forte mais utile pour valider la géométrie

% - Signal :
%   Chirp LFM (Linear Frequency Modulated)
%   → Plus simple que les signaux QPSK réels (OneWeb)

%% 3. PARAMÈTRES CLÉS

% - Fréquence porteuse : 11 GHz (bande Ku)
% - PRF : 5000 Hz (définit la vitesse non ambiguë)
% - UpdateRate : 10 Hz (fluidité de la simulation)
% - Modes d'acquisition : Snapshot / Burst

%% 4. TRAITEMENT DU SIGNAL : RANGE-DOPPLER

% - Filtre adapté (Matched Filter)
%   → Compression d'impulsion pour maximiser le SNR

% - Intégration cohérente
%   → FFT 2D :
%       * Fast Time → Distance
%       * Slow Time → Doppler

% - Hypothèse idéale :
%   → Signal direct et clutter parfaitement supprimés

% - Résultat :
%   Carte Range-Doppler → pic de détection
%   → Distance bistatique
%   → Vitesse bistatique

%% 5. POURSUITE : FILTRE DE KALMAN ÉTENDU (EKF)

% - Vecteur d'état (ECEF) :
%   x = [X, Y, Z, Vx, Vy, Vz]

% - Vecteur de mesure :
%   z = [R_bistatique, Doppler, Azimut, Elevation]

%% 5.1 BRUIT DE MESURE (GAUSSIEN)

% - Erreur distance : ±15 m
% - Erreur vitesse  : ±2.13 m/s
% - Erreur angulaire : ±1.5°

%% 5.2 INITIALISATION (PLOT-TO-TRACK)

% - Estimation de la position via :
%     * Direction d'arrivée (Azimut, Elevation)
%     * Contrainte de distance bistatique (ellipsoïde)

% - Méthode :
%   Intersection entre :
%     → Droite de visée (LOS)
%     → Ellipsoïde bistatique

% - Vitesse initiale :
%   → Fixée à zéro
%   → Forte incertitude (covariance élevée)

%% 5.3 MATRICE D'OBSERVATION (H)

% - Calculée numériquement (différences finies)
% - Permet la linéarisation du modèle non linéaire

%% 5.4 BRUIT DE PROCESSUS

% - Modèle DWNA (Discrete White Noise Acceleration)
%   → Modélise l'incertitude due aux accélérations non modélisées

% ================================================================


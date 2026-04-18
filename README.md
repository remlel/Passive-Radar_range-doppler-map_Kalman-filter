# 📡 Passive Radar Simulation with EKF Tracking

Simulation of a passive radar system using a LEO satellite (OneWeb-like) as an illuminator.  
The project includes bistatic range-Doppler processing and 3D target tracking using an Extended Kalman Filter (EKF) in an ECEF frame.

---

## 🎯 What this project does

- Simulates a **bistatic radar scenario** (satellite → target → ground receiver)
- Generates a **Range-Doppler map** from synthetic radar signals
- Extracts target observables:
  - Bistatic range
  - Bistatic velocity
  - Direction of arrival (azimuth, elevation)
- Tracks the target in **3D (ECEF)** using an **Extended Kalman Filter**
- Evaluates tracking performance under **noisy measurements**

---

## 🧱 Project Structure

```
project/
│
├── init_scenario.m  
│   → Scenario generation (satellite, target, receiver in ECEF)
│
├── visu_scenario_2D.m  
│   → 2D visualization of the scenario (trajectories on Earth)
│
├── config_radar_system_isotrope.m  
│   → Radar configuration (waveform, antenna, system parameters)
│
├── carte_rangedoppler.m  
│   → Main signal processing script:
│     generation of radar signals + Range-Doppler map computation
│
├── ref_bist.m  
│   → Bistatic reference calculations:
│     bistatic range and velocity from geometry
│
├── filtre_Kalman.m  
│   → Extended Kalman Filter core:
│     state estimation (position & velocity in ECEF)
│
├── h_et_H.m  
│   → Measurement model:
│     - h(x): nonlinear measurement function
│     - H(x): associated Jacobian
│
├── Affichage_Erreurs_filtre_Kalman.m  
│   → Plot of estimation errors (position & velocity)
│
├── Affichage_Positions_Kalman_vs_Reelles.m  
│   → 3D visualization:
│     comparison between true trajectory and EKF estimation
│
└── README.md  
    → Project documentation
```

## 4. Signal Processing – Range-Doppler

- Matched Filtering → Pulse compression  
- 2D FFT:
  - Fast Time → Range  
  - Slow Time → Doppler  

- Ideal assumption:
  → Direct path and clutter removed  

- Output:
  → Bistatic range & velocity  

---

## 5. Tracking – Extended Kalman Filter (EKF)

State:

x = [X, Y, Z, Vx, Vy, Vz]

Measurements:

z = [R_bistatic, V_bistatic, Azimuth, Elevation]

---

### Measurement Noise

- Bistatic Range: ±15 m  
- Bistatic Velocity: ±2.13 m/s  
- Angles: ±1.5°

**Note:**  
The noise levels are approximated using the **Range-Doppler resolution cell size**.  
In practice, estimation accuracy depends on SNR:
- High SNR → sub-bin interpolation improves accuracy  
- Low SNR → accuracy degrades beyond resolution

---

### Initialization

- Based on:
  - Direction of Arrival  
  - Bistatic range  

- Method:
  LOS ∩ bistatic ellipsoid  

- Initial velocity:
  - 0  
  - High uncertainty  

---

### Jacobian (H)

- Computed numerically (finite differences)

---

### Process Noise

- DWNA model (white noise acceleration)

---

## 🚀 Notes

The EKF currently uses **noisy ground truth measurements**, allowing independent validation of the tracking algorithm before full integration with the range-Doppler output.

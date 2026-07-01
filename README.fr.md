# VTOL BEMT Flight-Dynamics MATLAB

Langues : [English](README.md) | [中文](README.zh-CN.md) | [Français](README.fr.md) | [Italiano](README.it.md)

Outils MATLAB pour le trim VTOL/eVTOL, le BEMT rotor, la dynamique de battement, les dérivées de stabilité et de contrôle, et la simulation de réponse non linéaire.

Le dépôt est organisé autour de trois flux de travail :

1. **Flux MATLAB pale par pale** : `RUN_ME.m` / `RUN_TRIM_AND_STABILITY.m`. C'est le flux détaillé pour le trim, les dérivées de stabilité et les dérivées de contrôle. Il utilise un modèle de battement pale par pale avec BEMT, et peut fonctionner avec les données publiques `default` ou avec des données utilisateur `lookup` / `excel`.
2. **Flux Simulink pale par pale** : `RUN_RESPONSE_SIMULINK_SETUP.m` avec `cfg.rotor.flap_model = "blade"`. Ce flux utilise le même modèle rotor détaillé pale par pale pour la réponse non linéaire dans Simulink. Il peut aussi utiliser les données `default`, `lookup` ou `excel`, mais il est lent et sert surtout aux vérifications haute fidélité.
3. **Flux Simulink rapide disque battant** : `RUN_RESPONSE_SIMULINK_SETUP.m` avec le modèle par défaut `cfg.rotor.flap_model = "disk"`, puis `RUN_FAST_DISK_MEX_RESPONSE`. Ce flux utilise un modèle réduit de battement du disque rotor et une accélération MEX pour une simulation de réponse proche du temps réel. Ce flux rapide vise le modèle public `default` et ne prend pas en charge le chemin complet par tables lookup.

Références utiles :

- Modélisation rotor pale par pale : Stephen Rutherford, *Simulation techniques for the study of the manoeuvring of advanced rotorcraft configurations*, PhD thesis, University of Glasgow, 1997. <https://theses.gla.ac.uk/30844/>
- Dynamique de battement disque rotor / tip-path-plane : R. T. N. Chen, *Effects of primary rotor parameters on flapping dynamics*, NASA TP-1431, 1980. <https://ntrs.nasa.gov/citations/19800006879>

Les données aérodynamiques privées de lookup ne sont pas incluses. Le code peut fonctionner en mode `default` sans dossier `data/`. Les formats lookup et Excel sont décrits ci-dessous pour permettre aux utilisateurs d'ajouter leurs propres données.

## Index De Documentation

| Besoin | Section à lire | Fichiers principaux |
|---|---|---|
| Choisir le bon script d'entrée | [Carte Des Points D'Entrée](#carte-des-points-dentrée) | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Lancer le trim, les dérivées de stabilité et les dérivées de contrôle | [Démarrage rapide](#démarrage-rapide), [MATLAB pale par pale](#1-matlab-pale-par-pale), [Sorties de trim](#sorties-de-trim) | `RUN_ME.m`, `RUN_TRIM_AND_STABILITY.m` |
| Choisir les données `default`, `lookup` ou `excel` | [Commutateurs principaux](#commutateurs-principaux), [Données lookup](#données-lookup), [Base aérodynamique Excel](#base-aérodynamique-excel) | `RUN_ME.m`, `data_templates/` |
| Comprendre les paramètres publics par défaut | [Modèle par défaut](#modèle-par-défaut) | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Lancer la réponse Simulink lente haute fidélité | [Simulink lent pale par pale](#2-simulink-lent-pale-par-pale), [Réponse Simulink](#réponse-simulink) | `CREATE_RESPONSE_SIMULINK_MODEL.m`, `VTOL_RESPONSE_SIMULINK.slx` |
| Lancer la réponse Simulink rapide proche temps réel | [Simulink rapide disque battant](#3-simulink-rapide-disque-battant), [Réponse Simulink](#réponse-simulink) | `RUN_RESPONSE_SIMULINK_SETUP.m`, `VTOL_RESPONSE_SIMULINK_MEX.slx` |
| Vérifier l'ordre des états, commandes et sorties | [Sorties de trim](#sorties-de-trim), [Réponse Simulink](#réponse-simulink) | `trim_results`, `out.x_sim` |
| Ajouter des tables lookup txt | [Données lookup](#données-lookup) | `data/` |
| Ajouter une base aérodynamique Excel | [Base aérodynamique Excel](#base-aérodynamique-excel) | `data_templates/` |
| Confirmer ce qui ne doit pas être publié | [Politique du dépôt public](#politique-du-dépôt-public) | `.gitignore` |

## Carte Des Points D'Entrée

| Fichier | À utiliser quand | Modèle rotor | Données prises en charge | Sortie principale |
|---|---|---|---|---|
| `RUN_ME.m` | Vous voulez modifier directement les réglages de trim, stabilité et dérivées de contrôle dans un fichier interface. | pale par pale | `default`, `lookup`, et `excel` pour fuselage/commandes | `trim_results` |
| `RUN_TRIM_AND_STABILITY.m` | Vous voulez une entrée compacte pour le workflow MATLAB de trim et dérivées. | pale par pale | suit la configuration de style `RUN_ME.m` | `trim_results` |
| `RUN_RESPONSE_SIMULINK_SETUP.m` | Vous voulez trimmer d'abord, puis générer et initialiser un modèle de réponse Simulink. | pale par pale ou disque | `default`; le chemin détaillé pale par pale peut aussi utiliser les données lookup/Excel prises en charge | variables du base workspace et modèle Simulink |
| `RUN_FAST_DISK_MEX_RESPONSE.m` | Vous voulez lancer le modèle rapide Simulink/MEX déjà généré. | disque battant | chemin public `default` seulement | sortie Simulink `out` |
| `CREATE_RESPONSE_SIMULINK_MODEL.m` | Vous voulez régénérer explicitement le modèle Simulink lent pale par pale. | pale par pale | mêmes données que le workflow de réponse lent | `VTOL_RESPONSE_SIMULINK.slx` |
| `CREATE_RESPONSE_SIMULINK_MEX_MODEL.m` | Vous voulez régénérer explicitement le modèle Simulink rapide disque/MEX. | disque battant | chemin public `default` seulement | `VTOL_RESPONSE_SIMULINK_MEX.slx` |

Le chemin normal est `RUN_ME.m` pour les études de trim et dérivées, et `RUN_RESPONSE_SIMULINK_SETUP.m` pour la simulation de réponse.

## Démarrage rapide

Ouvrir MATLAB dans ce dossier.

Trim, stabilité et dérivées de contrôle :

```matlab
RUN_ME
```

Réponse non linéaire Simulink/MEX :

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

`RUN_RESPONSE_SIMULINK_SETUP` effectue d'abord le trim, exporte les conditions initiales courantes vers le base workspace MATLAB, reconstruit le MEX rapide du modèle disque battant pour la configuration courante, puis régénère le modèle Simulink.

`RUN_FAST_DISK_MEX_RESPONSE` exécute seulement Simulink. Il ne modifie aucun paramètre aérodynamique ou de dynamique du vol et ne charge pas de fichier MAT d'initialisation en cache.

## Commutateurs principaux

Chaque flux de travail possède son propre script d'entrée. Lorsque les données lookup sont prises en charge, les commutateurs de source de données gardent les mêmes noms.

### 1. MATLAB pale par pale

Utiliser `RUN_ME.m` pour le trim, les dérivées de stabilité et les dérivées de contrôle. Ce flux utilise le modèle rotor détaillé pale par pale.

```matlab
data_mode = "default";          % "default" or "lookup"
aero_database_mode = data_mode; % "default", "lookup", or "excel"

cfg.rotor.flap_model = "blade";

cfg.switch.geometry        = data_mode;
cfg.switch.rotor_positions = data_mode;
cfg.switch.chord           = data_mode;
cfg.switch.pretwist        = data_mode;
cfg.switch.airfoil         = data_mode;
cfg.switch.fuselage        = aero_database_mode;
cfg.switch.controls        = aero_database_mode;
```

`default` fonctionne sans données privées. `lookup` utilise des tables txt dans `data/`. `excel` est pris en charge pour l'aérodynamique fuselage et surfaces de contrôle.

### 2. Simulink lent pale par pale

Utiliser `RUN_RESPONSE_SIMULINK_SETUP.m` et sélectionner le modèle pale par pale avant le setup :

```matlab
flap_model_override = "blade";
RUN_RESPONSE_SIMULINK_SETUP
```

Dans `RUN_RESPONSE_SIMULINK_SETUP.m`, les mêmes commutateurs de source de données sont utilisés :

```matlab
data_mode = "default";          % "default" or "lookup"
aero_database_mode = data_mode; % "default", "lookup", or "excel"
cfg.rotor.flap_model = requested_flap_model;  % "blade"
```

Ce chemin peut utiliser `default`, `lookup` ou `excel`, mais il est lent parce qu'il évalue la réponse détaillée pale par pale dans Simulink.

### 3. Simulink rapide disque battant

Utiliser le setup du modèle disque par défaut, puis exécuter le modèle rapide généré :

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

Pour ce flux, conserver le chemin de données public par défaut :

```matlab
data_mode = "default";
aero_database_mode = "default";
cfg.rotor.flap_model = "disk";
cfg.response.compile_mex = true;
```

Le chemin MEX rapide est destiné au modèle numérique default/disk. Il ne prend pas en charge le chemin complet par tables lookup. `RUN_FAST_DISK_MEX_RESPONSE` exécute uniquement le setup Simulink/MEX déjà généré ; il ne possède pas de commutateurs de données aérodynamiques.

## Modèle par défaut

Le modèle par défaut est volontairement simple et adapté à une publication publique. Les paramètres importants sont exposés dans `RUN_ME.m` et `RUN_RESPONSE_SIMULINK_SETUP.m`.

Les valeurs ci-dessous sont des valeurs publiques d'exemple. Elles servent aux essais, à la diffusion et à la vérification du code ; elles ne représentent pas une configuration privée de véhicule.

| Domaine | Paramètres | Valeur publique par défaut | Unité | Signification en mode default | À modifier dans |
|---|---|---:|---|---|---|
| Environnement | `cfg.environment.rho_kg_m3`, `gravity_m_s2`, `use_isa`, `initial_altitude_m` | `1.225`, `9.81`, `false`, `0` | kg/m^3, m/s^2, -, m | Le mode à densité constante utilise `rho_kg_m3`. Si `use_isa = true`, le trim utilise la densité ISA à `initial_altitude_m`, et la réponse non linéaire met à jour la densité avec le déplacement `z` en axes terre. | `RUN_ME.m`, `RUN_RESPONSE.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Véhicule | `cfg.vehicle.mass_kg` | `1900` | kg | Masse du véhicule utilisée dans l'équilibre de trim et les équations de réponse. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Inertie véhicule | `cfg.vehicle.inertia_kg_m2` | `[1966.5, 5245.3, 3282.7]` | kg m^2 | Vecteur d'inertie axes corps, ordonné `[Ixx Iyy Izz]`. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Taille et vitesse rotor | `cfg.rotor.radius_m`, `blade_count`, `omega_rad_s` | `1.3`, `5`, `90` | m, -, rad/s | Échelle rotor publique, nombre de pales et vitesse angulaire nominale. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Battement rotor | `cfg.rotor.flap_inertia_kg_m2`, `flap_spring_nm_rad` | `2.25`, `16000` | kg m^2, N m/rad | Inertie de battement de pale et raideur équivalente de battement. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Discrétisation rotor | `cfg.rotor.blade_element_count`, `azimuth_steps` | `10`, `72` | -, - | Nombre d'éléments en envergure et de stations azimutales pour le calcul rotor détaillé. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Sections de profils rotor | `cfg.rotor.airfoil_section_edges` | `[0.25, 0.40, 0.50, 0.80, 0.92]` | r/R | Coupures radiales pour les six sections de profils rotor. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Sens de rotation rotor | `cfg.rotor.rotational_direction` | `[1, -1, 1, -1, 1, -1]` | - | Signe de rotation des rotors 1 à 6. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Options de modèle rotor | `cfg.rotor.flap_model`, `flap_integrator`, `inflow_model` | selon le flux | - | Sélection du battement pale par pale ou disque, de l'intégrateur d'état de battement et du comportement d'inflow. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| CG par défaut | `cfg.defaults.geometry.x_cg_mm`, `y_cg_mm`, `z_cg_mm` | `3554.3`, `0`, `-588.7` | mm | Centre de gravité public par défaut lorsque le lookup de géométrie est désactivé. | `RUN_ME.m` |
| Positions rotor par défaut | `cfg.defaults.geometry.rotor_position_coeffs_mm` | matrice 6-by-7 | mm | Table compacte de coefficients qui déplace chaque rotor avec le tilt nacelle lorsque le lookup de position rotor est désactivé. | `RUN_ME.m` |
| Corde de pale | `cfg.defaults.chord_m` | `0.20264354` | m | Corde constante publique par défaut lorsque le lookup de corde est désactivé. | `RUN_ME.m` |
| Prétorsion de pale | `cfg.defaults.pretwist_root_deg`, `pretwist_tip_deg` | `19.279996`, `-6.276289` | deg | Description simplifiée de la prétorsion du côté racine au côté saumon lorsque le lookup est désactivé. | `RUN_ME.m` |
| Profil rotor par défaut | `cfg.defaults.airfoil.*` | `cl_alpha=5.579842`, `cl_max=1.134702`, `cd0=0.010150`, `cd_alpha2=1.758467` | mixte | Paramètres simplifiés de pente de portance, limite de portance et forme de traînée lorsque le lookup profil est désactivé. | `RUN_ME.m` |
| Références fuselage | `cfg.fuselage.reference_area_m2`, `mean_aero_chord_m`, `span_m` | `14.41`, `1.31`, `12` | m^2, m, m | Dimensions de référence pour les coefficients aérodynamiques fuselage et aile fixe par défaut. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Aéro fuselage par défaut | `cfg.defaults.fuselage.*` | voir `RUN_ME.m` | mixte | Paramètres simplifiés de traînée, portance, moment de tangage, force latérale, moment de lacet et moment de roulis. | `RUN_ME.m` |
| Surfaces de contrôle | `cfg.defaults.controls.elevator`, `rudder`, `aileron` | voir `RUN_ME.m` | par rad | Incréments de coefficients par défaut lorsque les données lookup ou Excel de contrôle sont désactivées. | `RUN_ME.m` |
| Mélange des commandes | `cfg.control_blend.*` | activé, basé sur le tilt, `sincos` | - | Répartit les canaux de commande rotor et aile fixe en fonction du tilt nacelle. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Dérivées dynamiques | `cfg.aero.dynamic_derivatives.*` | optionnel | mixte | Corrections optionnelles de dérivées aérodynamiques dynamiques de la cellule. | `RUN_ME.m` |

## Sorties de trim

Le résultat principal est `trim_results`.

```matlab
trim_results.trim_table
trim_results.stability_A
trim_results.control_B
trim_results.final_trim_var
trim_results.last_eigenvalues
trim_results.rotor_locations_m
trim_results.speed_mps
```

L'ordre des états dans la matrice linéarisée `A` est :

```text
[u, w, q, theta, v, p, phi, r]
```

L'ordre des commandes dans `B` est :

```text
[collective, longitudinal, lateral, yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

Si `cfg.control_blend.enabled = true` et `cfg.control_blend.apply_to_trim = true`, les variables de trim/commande après le collectif deviennent des commandes équivalentes pilote mélangées :

```text
[collective, blend_pitch, blend_roll, blend_yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

La loi de transition par défaut actuelle est basée sur l'angle de basculement nacelle :

```matlab
cfg.control_blend.independent_variable = "tilt_angle";
cfg.control_blend.tilt_helicopter_deg = 90;
cfg.control_blend.tilt_fixedwing_deg = 0;
cfg.control_blend.schedule = "sincos";
```

Avec `schedule = "sincos"`, le code utilise directement l'angle de basculement réel :

```text
tilt_limited = clamp(tilt_angle_deg, min(tilt_helicopter_deg, tilt_fixedwing_deg),
                                     max(tilt_helicopter_deg, tilt_fixedwing_deg))
rotor_weight = clamp(sind(tilt_limited), 0, 1)
fixed_weight = clamp(cosd(tilt_limited), 0, 1)
```

Par exemple, à 90 deg de tilt, le poids rotor vaut 1 et le poids fixed-wing vaut 0. À 0 deg de tilt, le poids rotor vaut 0 et le poids fixed-wing vaut 1. À 60 deg de tilt, les poids sont `sind(60)` et `cosd(60)`, et non une paire linéaire complémentaire.

Les trois canaux équivalents pilote mélangés sont alloués comme suit :

```text
rotor_longitudinal_deg = rotor_weight * rotor_gains(1) * blend_pitch
rotor_lateral_deg      = rotor_weight * rotor_gains(2) * blend_roll
rotor_yaw_deg          = rotor_weight * rotor_gains(3) * blend_yaw

fixed_pitch_deg += fixed_weight * fixed_gains(1) * blend_pitch
fixed_roll_deg  += fixed_weight * fixed_gains(2) * blend_roll
fixed_yaw_deg   += fixed_weight * fixed_gains(3) * blend_yaw
```

`rotor_gains` et `fixed_gains` sont ordonnés comme `[pitch, roll, yaw]`.

Les lois optionnelles `linear` et `smoothstep` restent prises en charge pour les études plus anciennes. Dans ces modes, le code calcule un poids fixed-wing scalaire à partir du tilt nacelle, puis utilise `rotor_weight = 1 - fixed_weight`.

Le vecteur de commande fixed-wing est ordonné comme `[fixed_pitch, fixed_yaw, fixed_roll]` en degrés. Lorsque les tables lookup ou feuilles Excel de surfaces physiques WL/WR/VL/VR sont utilisées, ces trois canaux sont mappés vers les déflexions physiques par `cfg.controls.surface_mixing_matrix`. Si cette matrice est vide, le mapping par défaut est :

```text
gain = cfg.controls.channel_to_physical_gain   % default 0.5

DVL1 = gain * (fixed_yaw - fixed_pitch)
DVR1 = gain * (fixed_yaw + fixed_pitch)
DVL2 = gain * (fixed_yaw - fixed_pitch)
DVR2 = gain * (fixed_yaw + fixed_pitch)

DWL1 = DWR1 = fixed_roll
DWL2 = DWR2 = fixed_roll
```

Pour utiliser un allocateur personnalisé, définir :

```matlab
cfg.controls.channel_names = {'pitch','yaw','roll'};
cfg.controls.physical_surface_names = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
cfg.controls.surface_mixing_matrix = M;  % rows: physical surfaces, columns: channels
cfg.controls.surface_bias_deg = b;       % optional bias for each physical surface
```

## Atmosphère Et Altitude

Le modèle d'environnement peut utiliser une densité constante ou la densité ISA :

```matlab
cfg.environment.use_isa = false;        % densité constante rho_kg_m3
cfg.environment.use_isa = true;         % densité ISA
cfg.environment.initial_altitude_m = 0; % altitude initiale au-dessus du niveau moyen de la mer
```

Lorsque l'ISA est activée, le trim et les dérivées de stabilité utilisent la densité à `initial_altitude_m`. Dans la réponse non linéaire MATLAB et Simulink, le 12e état de réponse est le déplacement `z` en axes terre, positif vers le bas. La mise à jour de densité utilise donc :

```text
altitude_m = initial_altitude_m - z
```

La structure de réponse enregistre :

```matlab
trim_results.response.altitude_history
trim_results.response.density_history
```

## Réponse Simulink

Le flux de réponse Simulink est :

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

Le script setup est le seul endroit où modifier le pas d'échantillonnage de réponse et la condition de trim :

```matlab
cfg.trim.tilt_angle_deg
cfg.trim.speed_mps
cfg.response.dt_s
cfg.response.pilot_stick
cfg.response.pilot_stick_to_control_gain_deg
cfg.response.control_delta              % incrément direct optionnel de debug
cfg.response.fixed_wing_control_delta   % incrément fixe optionnel de debug
cfg.response.rotor_tilt_angle_deg
```

`RUN_FAST_DISK_MEX_RESPONSE` ne contient aucun paramètre de dynamique du vol. Il exécute seulement le modèle Simulink courant avec les variables du base workspace générées par `RUN_RESPONSE_SIMULINK_SETUP`.

Le modèle Simulink contient seulement une interface manche pilote en boucle ouverte :

```text
Pilot stick input = [collective, pitch, roll, yaw], normalisé
Stick gain        = échelle de commande plein manche en deg
Rotor tilt input  = six angles de nacelle en deg
```

Le modèle généré ne contient pas de loi de commande en boucle fermée. Les utilisateurs peuvent remplacer la source de manche constante par un joystick, un signal d'essai ou une autre source de commande en boucle ouverte.

`out.x_sim` utilise l'ordre de 12 états suivant :

```text
[u v w p q r phi theta psi x y z]
```

Signification des coordonnées :

```text
u, v, w        vitesses dans les axes corps
p, q, r        vitesses angulaires dans les axes corps
phi, theta, psi angles d'Euler
x, y, z        positions terrestre/inertielle
forces_sim     axes corps [Fx Fy Fz Mx My Mz]
```

## Données lookup

Les données privées de lookup ne sont pas incluses. Si le mode lookup est activé, placer les fichiers dans :

```text
data/
```

Tous les fichiers txt doivent contenir uniquement des valeurs numériques. Ne pas ajouter d'en-têtes, noms de colonnes, unités, commentaires ou texte explicatif. La première ligne doit déjà être une ligne de données.

### Tables CL/CD des profils rotor

Fichiers :

```text
CS1_cl.txt ... CS6_cl.txt
CS1_cd.txt ... CS6_cd.txt
```

Format :

```text
Mach_1   Mach_2   Mach_3   ...
alpha_1  value(alpha_1,Mach_1)  value(alpha_1,Mach_2)  value(alpha_1,Mach_3) ...
alpha_2  value(alpha_2,Mach_1)  value(alpha_2,Mach_2)  value(alpha_2,Mach_3) ...
...
```

### Corde et prétorsion

La corde est en mètres. La prétorsion est en degrés.

```text
r_over_R_1   chord_m_1
r_over_R_2   chord_m_2
...
```

```text
r_over_R_1   pretwist_deg_1
r_over_R_2   pretwist_deg_2
...
```

### CG et positions rotor

Table CG :

```text
tilt_deg   x_cg_mm   y_cg_mm   z_cg_mm
...
```

Table de positions rotor :

```text
tilt_deg   r1_x_mm r1_y_mm r1_z_mm   r2_x_mm r2_y_mm r2_z_mm ... r6_x_mm r6_y_mm r6_z_mm
...
```

Le code interpole les positions en fonction de l'angle de tilt.

### Tables aérodynamiques de base du fuselage

Fichiers :

```text
Fuselage_cd.txt
Fuselage_cl.txt
Fuselage_cm.txt
Fuselage_cc.txt
Fuselage_cn.txt
Fuselage_cll.txt
```

Format :

```text
tilt_1   tilt_2   tilt_3   ...
alpha_1  coeff(alpha_1,tilt_1)  coeff(alpha_1,tilt_2)  coeff(alpha_1,tilt_3) ...
alpha_2  coeff(alpha_2,tilt_1)  coeff(alpha_2,tilt_2)  coeff(alpha_2,tilt_3) ...
...
```

Pour les tables latérales/directionnelles comme `cc/cn/cll`, les coefficients peuvent représenter la valeur à un angle de dérapage ou une déflexion de référence. Garder la même convention dans la configuration du code.

### Tables de surfaces de contrôle

Pour les tables de type profondeur :

```text
deflection_deg   alpha_deg   dCD   dCL   dCM
...
```

Pour les tables de type direction/aileron :

```text
deflection_deg   alpha_deg   dCC   dCN   dCLL
...
```

## Base aérodynamique Excel

La base fuselage et surfaces de contrôle peut aussi être stockée dans un seul fichier Excel. MATLAB peut lire directement différentes feuilles.

Un modèle synthétique public est fourni ici :

```text
data_templates/aero_database_template.xlsx
```

Le copier dans `data/` ou pointer `cfg.data.aero.excel_file` vers son chemin, puis remplacer les lignes de démonstration par vos propres données.

Feuilles recommandées :

```text
base_aero
WL1 WL2 WR1 WR2 VL1 VL2 VR1 VR2
```

Pour la base fuselage/cellule, deux organisations Excel sont prises en charge :

1. Mise en page feuille unique : placer tous les angles de tilt nacelle/cellule dans `base_aero` et inclure `tilt_angle_deg` comme première colonne numérique.
2. Mise en page par feuilles séparées : placer chaque angle de tilt dans une feuille séparée, par exemple `base_0`, `base_30`, `base_60`, `base_90`. Dans ce cas, définir `cfg.data.aero.base_sheets` et `cfg.data.aero.base_sheet_tilt_angle_deg` dans `RUN_ME.m`.

Exemple de réglage par feuilles séparées :

```matlab
cfg.switch.fuselage = "excel";
cfg.switch.controls = "excel";
cfg.data.aero.excel_file = 'aero_database.xlsx';
cfg.data.aero.base_sheets = {'base_0','base_30','base_60','base_90'};
cfg.data.aero.base_sheet_tilt_angle_deg = [0 30 60 90];
cfg.data.aero.control_surface_sheets = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
```

Pour utiliser les anciennes tables txt, définir `cfg.switch.fuselage = "lookup"` et `cfg.switch.controls = "lookup"`.

Schéma `base_aero` :

```text
tilt_angle_deg   beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

Schéma des feuilles séparées, lorsque l'angle de tilt est fourni par `cfg.data.aero.base_sheet_tilt_angle_deg` :

```text
beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

Schéma de feuille surface de contrôle :

```text
deflection_deg   alpha_deg   dCD   dCL   dCm   dCC   dCn   dCl
...
```

Les anciens fichiers Excel qui contiennent encore une colonne `Mach` dans ces feuilles fuselage/surfaces restent pris en charge, mais Mach n'est pas requis par le modèle fuselage/surfaces actuel.

Les feuilles Excel peuvent contenir des en-têtes et des notes textuelles ; le lecteur MATLAB ne conserve que les lignes numériques complètes. Les lignes numériques doivent toutefois former une grille d'interpolation complète pour chaque variable indépendante de la feuille.

Ce dépôt peut inclure des exemples de format et des valeurs synthétiques de démonstration, mais pas de données aérodynamiques privées.

## Politique du dépôt public

Le dépôt ne doit pas inclure :

```text
data/
legacy/
examples/ that require private data
*.mat result/cache files
*.zip packages
slprj/
codegen_mex*/
*.mexw64
*.slxc
```

Les utilisateurs doivent générer les fichiers MEX localement en exécutant `RUN_RESPONSE_SIMULINK_SETUP`.

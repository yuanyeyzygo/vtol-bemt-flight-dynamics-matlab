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

Paramètres usuels du véhicule et de l'environnement :

```matlab
cfg.environment.rho_kg_m3
cfg.environment.gravity_m_s2
cfg.vehicle.mass_kg
cfg.vehicle.inertia_kg_m2   % [Ixx Iyy Izz]
```

Paramètres rotor :

```matlab
cfg.rotor.radius_m
cfg.rotor.blade_count
cfg.rotor.omega_rad_s
cfg.rotor.flap_inertia_kg_m2
cfg.rotor.flap_spring_nm_rad
cfg.rotor.rotational_direction
cfg.rotor.blade_element_count
cfg.rotor.azimuth_steps
cfg.rotor.flap_model        % "blade" or "disk"
cfg.rotor.flap_integrator
cfg.rotor.inflow_model
```

La géométrie rotor par défaut utilise une forme analytique compacte pour chaque rotor :

```matlab
x = x0 + x_cos*cos(tilt) + x_sin*sin(tilt)
y = y0
z = z0 + z_cos*cos(tilt) + z_sin*sin(tilt)
```

Les coefficients sont stockés dans :

```matlab
cfg.defaults.geometry.rotor_position_coeffs_mm
```

Profil aérodynamique rotor et géométrie de pale par défaut :

```matlab
cfg.defaults.chord_m
cfg.defaults.pretwist_root_deg
cfg.defaults.pretwist_tip_deg
cfg.defaults.airfoil.cl_alpha_per_rad
cfg.defaults.airfoil.cl_max
cfg.defaults.airfoil.cd0
cfg.defaults.airfoil.cd_alpha2
```

Paramètres fuselage et contrôles par défaut :

```matlab
cfg.fuselage.reference_area_m2
cfg.fuselage.mean_aero_chord_m
cfg.fuselage.span_m
cfg.defaults.fuselage.cd0
cfg.defaults.fuselage.cl_alpha_per_rad
cfg.defaults.fuselage.cm_alpha_per_rad
cfg.defaults.fuselage.cc_beta
cfg.defaults.fuselage.cn_beta
cfg.defaults.fuselage.cll_beta
cfg.defaults.controls.elevator
cfg.defaults.controls.rudder
cfg.defaults.controls.aileron
```

Les dérivées aérodynamiques dynamiques sont optionnelles et exposées via :

```matlab
cfg.aero.dynamic_derivatives.enabled
cfg.aero.dynamic_derivatives.CLq
cfg.aero.dynamic_derivatives.Cmq
cfg.aero.dynamic_derivatives.Clp
cfg.aero.dynamic_derivatives.Cnp
cfg.aero.dynamic_derivatives.Cyp
cfg.aero.dynamic_derivatives.Cnr
cfg.aero.dynamic_derivatives.Clr
cfg.aero.dynamic_derivatives.Cyr
cfg.aero.dynamic_derivatives.Cma_dot
cfg.aero.dynamic_derivatives.Cn_beta_dot
```

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

Les lois optionnelles `linear` et `smoothstep` restent prises en charge pour les études plus anciennes. Dans ces modes, le code calcule d'abord un poids fixed-wing scalaire à partir de la vitesse ou du tilt, puis utilise `rotor_weight = 1 - fixed_weight`.

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
cfg.response.control_delta
cfg.response.fixed_wing_control_delta
cfg.response.rotor_tilt_angle_deg
```

`RUN_FAST_DISK_MEX_RESPONSE` ne contient aucun paramètre de dynamique du vol. Il exécute seulement le modèle Simulink courant avec les variables du base workspace générées par `RUN_RESPONSE_SIMULINK_SETUP`.

Le modèle Simulink contient une interface simple de changement de commande :

```text
Rotor control delta  = [collective, longitudinal, lateral, yaw]
Fixed-wing control   = [pitch, yaw, roll]
Rotor tilt input     = six nacelle tilt angles in deg
```

Le modèle d'exemple utilise deux blocs Step additionnés dans le canal collectif, et des constantes pour les autres canaux. Les utilisateurs peuvent remplacer ces blocs par les sorties de leur propre contrôleur.

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

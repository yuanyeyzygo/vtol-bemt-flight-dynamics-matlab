# VTOL BEMT Flight-Dynamics MATLAB

Lingue: [English](README.md) | [中文](README.zh-CN.md) | [Français](README.fr.md) | [Italiano](README.it.md)

Strumenti MATLAB per trim VTOL/eVTOL, BEMT del rotore, dinamica di flappeggio, derivate di stabilità e controllo, e simulazione di risposta non lineare.

Il repository è organizzato in tre flussi di lavoro:

1. **Flusso MATLAB pala per pala**: `RUN_ME.m` / `RUN_TRIM_AND_STABILITY.m`. È il flusso dettagliato per trim, derivate di stabilità e derivate di controllo. Usa un modello di flappeggio pala per pala con BEMT, e può funzionare con dati pubblici `default` oppure con dati utente `lookup` / `excel`.
2. **Flusso Simulink pala per pala**: `RUN_RESPONSE_SIMULINK_SETUP.m` con `cfg.rotor.flap_model = "blade"`. Usa lo stesso modello rotore dettagliato pala per pala per la risposta non lineare in Simulink. Può usare anche dati `default`, `lookup` o `excel`, ma è computazionalmente lento ed è pensato soprattutto per verifiche ad alta fedeltà.
3. **Flusso Simulink rapido disco battente**: `RUN_RESPONSE_SIMULINK_SETUP.m` con il modello predefinito `cfg.rotor.flap_model = "disk"`, seguito da `RUN_FAST_DISK_MEX_RESPONSE`. Usa un modello ridotto di flappeggio del disco rotore e accelerazione MEX per simulazioni di risposta in stile tempo reale. Questo flusso rapido è pensato per il modello pubblico `default` e non supporta il percorso completo con lookup table.

Riferimenti utili:

- Modellazione rotore pala per pala: Stephen Rutherford, *Simulation techniques for the study of the manoeuvring of advanced rotorcraft configurations*, PhD thesis, University of Glasgow, 1997. <https://theses.gla.ac.uk/30844/>
- Dinamica di flappeggio disco/tip-path-plane: R. T. N. Chen, *Effects of primary rotor parameters on flapping dynamics*, NASA TP-1431, 1980. <https://ntrs.nasa.gov/citations/19800006879>

I dati aerodinamici privati di lookup non sono inclusi. Il codice può funzionare in modalità `default` senza una cartella `data/`. I formati lookup ed Excel sono documentati sotto, così gli utenti possono aggiungere i propri dati.

## Avvio rapido

Aprire MATLAB in questa cartella.

Trim, stabilità e derivate di controllo:

```matlab
RUN_ME
```

Risposta non lineare Simulink/MEX:

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

`RUN_RESPONSE_SIMULINK_SETUP` esegue prima il trim del velivolo, esporta le condizioni iniziali correnti nel base workspace MATLAB, ricompila il MEX rapido del modello disco battente per la configurazione corrente e rigenera il modello Simulink.

`RUN_FAST_DISK_MEX_RESPONSE` esegue solo Simulink. Non modifica parametri aerodinamici o di dinamica del volo e non carica file MAT di inizializzazione in cache.

## Switch principali

Ogni flusso di lavoro ha il proprio script di ingresso. Dove i dati lookup sono supportati, gli switch della sorgente dati usano gli stessi nomi.

### 1. MATLAB pala per pala

Usare `RUN_ME.m` per trim, derivate di stabilità e derivate di controllo. Questo flusso usa il modello rotore dettagliato pala per pala.

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

`default` funziona senza dati privati. `lookup` usa tabelle txt in `data/`. `excel` è supportato per aerodinamica fusoliera e superfici di controllo.

### 2. Simulink lento pala per pala

Usare `RUN_RESPONSE_SIMULINK_SETUP.m` e selezionare il modello pala per pala prima del setup:

```matlab
flap_model_override = "blade";
RUN_RESPONSE_SIMULINK_SETUP
```

Dentro `RUN_RESPONSE_SIMULINK_SETUP.m`, vengono usati gli stessi switch della sorgente dati:

```matlab
data_mode = "default";          % "default" or "lookup"
aero_database_mode = data_mode; % "default", "lookup", or "excel"
cfg.rotor.flap_model = requested_flap_model;  % "blade"
```

Questo percorso può usare `default`, `lookup` o `excel`, ma è lento perché valuta in Simulink la risposta dettagliata pala per pala.

### 3. Simulink rapido disco battente

Usare il setup del modello disco predefinito e poi eseguire il modello rapido generato:

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

Per questo flusso, mantenere il percorso dati pubblico predefinito:

```matlab
data_mode = "default";
aero_database_mode = "default";
cfg.rotor.flap_model = "disk";
cfg.response.compile_mex = true;
```

Il percorso MEX rapido è pensato per il modello numerico default/disk. Non supporta il percorso completo con lookup table. `RUN_FAST_DISK_MEX_RESPONSE` esegue solo il setup Simulink/MEX già generato; non contiene switch per dati aerodinamici.

## Modello default

Il modello default è volutamente semplice e adatto alla pubblicazione. I parametri importanti sono esposti in `RUN_ME.m` e `RUN_RESPONSE_SIMULINK_SETUP.m`.

Parametri comuni di veicolo e ambiente:

```matlab
cfg.environment.rho_kg_m3
cfg.environment.gravity_m_s2
cfg.vehicle.mass_kg
cfg.vehicle.inertia_kg_m2   % [Ixx Iyy Izz]
```

Parametri rotore:

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

La geometria rotore default usa una forma analitica compatta per ogni rotore:

```matlab
x = x0 + x_cos*cos(tilt) + x_sin*sin(tilt)
y = y0
z = z0 + z_cos*cos(tilt) + z_sin*sin(tilt)
```

I coefficienti sono memorizzati in:

```matlab
cfg.defaults.geometry.rotor_position_coeffs_mm
```

Profilo aerodinamico rotore e geometria pala default:

```matlab
cfg.defaults.chord_m
cfg.defaults.pretwist_root_deg
cfg.defaults.pretwist_tip_deg
cfg.defaults.airfoil.cl_alpha_per_rad
cfg.defaults.airfoil.cl_max
cfg.defaults.airfoil.cd0
cfg.defaults.airfoil.cd_alpha2
```

Parametri fusoliera e controlli default:

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

Le derivate aerodinamiche dinamiche sono opzionali ed esposte tramite:

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

## Output del trim

Il risultato principale è `trim_results`.

```matlab
trim_results.trim_table
trim_results.stability_A
trim_results.control_B
trim_results.final_trim_var
trim_results.last_eigenvalues
trim_results.rotor_locations_m
trim_results.speed_mps
```

L'ordine degli stati nella matrice linearizzata `A` è:

```text
[u, w, q, theta, v, p, phi, r]
```

L'ordine dei controlli in `B` è:

```text
[collective, longitudinal, lateral, yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

Se `cfg.control_blend.enabled = true` e `cfg.control_blend.apply_to_trim = true`, le variabili di trim/controllo dopo il collettivo diventano comandi equivalenti pilota miscelati:

```text
[collective, blend_pitch, blend_roll, blend_yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

La schedulazione di transizione default corrente è basata sull'angolo di tilt della nacella:

```matlab
cfg.control_blend.independent_variable = "tilt_angle";
cfg.control_blend.tilt_helicopter_deg = 90;
cfg.control_blend.tilt_fixedwing_deg = 0;
cfg.control_blend.schedule = "sincos";
```

Con `schedule = "sincos"`, il codice usa direttamente l'angolo di tilt reale:

```text
tilt_limited = clamp(tilt_angle_deg, min(tilt_helicopter_deg, tilt_fixedwing_deg),
                                     max(tilt_helicopter_deg, tilt_fixedwing_deg))
rotor_weight = clamp(sind(tilt_limited), 0, 1)
fixed_weight = clamp(cosd(tilt_limited), 0, 1)
```

Per esempio, a 90 deg di tilt il peso rotore è 1 e il peso fixed-wing è 0. A 0 deg di tilt il peso rotore è 0 e il peso fixed-wing è 1. A 60 deg di tilt i pesi sono `sind(60)` e `cosd(60)`, non una coppia lineare complementare.

I tre canali equivalenti pilota miscelati sono allocati così:

```text
rotor_longitudinal_deg = rotor_weight * rotor_gains(1) * blend_pitch
rotor_lateral_deg      = rotor_weight * rotor_gains(2) * blend_roll
rotor_yaw_deg          = rotor_weight * rotor_gains(3) * blend_yaw

fixed_pitch_deg += fixed_weight * fixed_gains(1) * blend_pitch
fixed_roll_deg  += fixed_weight * fixed_gains(2) * blend_roll
fixed_yaw_deg   += fixed_weight * fixed_gains(3) * blend_yaw
```

Sia `rotor_gains` sia `fixed_gains` sono ordinati come `[pitch, roll, yaw]`.

Le schedulazioni opzionali `linear` e `smoothstep` restano supportate per studi precedenti. In questi modi il codice calcola prima un peso fixed-wing scalare da velocità o tilt, poi usa `rotor_weight = 1 - fixed_weight`.

Il vettore di comando fixed-wing è ordinato come `[fixed_pitch, fixed_yaw, fixed_roll]` in gradi. Quando si usano tabelle lookup o fogli Excel per superfici fisiche WL/WR/VL/VR, questi tre canali sono mappati alle deflessioni fisiche tramite `cfg.controls.surface_mixing_matrix`. Se la matrice è vuota, il mapping default è:

```text
gain = cfg.controls.channel_to_physical_gain   % default 0.5

DVL1 = gain * (fixed_yaw - fixed_pitch)
DVR1 = gain * (fixed_yaw + fixed_pitch)
DVL2 = gain * (fixed_yaw - fixed_pitch)
DVR2 = gain * (fixed_yaw + fixed_pitch)

DWL1 = DWR1 = fixed_roll
DWL2 = DWR2 = fixed_roll
```

Per usare un allocatore personalizzato, impostare:

```matlab
cfg.controls.channel_names = {'pitch','yaw','roll'};
cfg.controls.physical_surface_names = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
cfg.controls.surface_mixing_matrix = M;  % rows: physical surfaces, columns: channels
cfg.controls.surface_bias_deg = b;       % optional bias for each physical surface
```

## Risposta Simulink

Il flusso di risposta Simulink è:

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

Lo script di setup è l'unico punto in cui modificare il passo temporale della risposta e la condizione di trim:

```matlab
cfg.trim.tilt_angle_deg
cfg.trim.speed_mps
cfg.response.dt_s
cfg.response.control_delta
cfg.response.fixed_wing_control_delta
cfg.response.rotor_tilt_angle_deg
```

`RUN_FAST_DISK_MEX_RESPONSE` non contiene parametri di dinamica del volo. Esegue solo il modello Simulink corrente usando le variabili del base workspace generate da `RUN_RESPONSE_SIMULINK_SETUP`.

Il modello Simulink contiene una semplice interfaccia di variazione dei comandi:

```text
Rotor control delta  = [collective, longitudinal, lateral, yaw]
Fixed-wing control   = [pitch, yaw, roll]
Rotor tilt input     = six nacelle tilt angles in deg
```

Il modello di esempio usa due blocchi Step sommati nel canale collettivo e costanti per gli altri canali. Gli utenti possono sostituire questi blocchi con le uscite del proprio controllore.

`out.x_sim` usa il seguente ordine di 12 stati:

```text
[u v w p q r phi theta psi x y z]
```

Significato delle coordinate:

```text
u, v, w        velocità assi corpo
p, q, r        velocità angolari assi corpo
phi, theta, psi angoli di Eulero
x, y, z        posizioni terra/inerziali
forces_sim     assi corpo [Fx Fy Fz Mx My Mz]
```

## Dati lookup

I dati privati di lookup non sono inclusi. Se il modo lookup è attivo, mettere i file in:

```text
data/
```

Tutti i file txt devono contenere solo valori numerici. Non aggiungere intestazioni, nomi di colonna, unità, commenti o testo esplicativo. La prima riga deve essere già una riga di dati.

### Tabelle CL/CD dei profili rotore

File:

```text
CS1_cl.txt ... CS6_cl.txt
CS1_cd.txt ... CS6_cd.txt
```

Formato:

```text
Mach_1   Mach_2   Mach_3   ...
alpha_1  value(alpha_1,Mach_1)  value(alpha_1,Mach_2)  value(alpha_1,Mach_3) ...
alpha_2  value(alpha_2,Mach_1)  value(alpha_2,Mach_2)  value(alpha_2,Mach_3) ...
...
```

### Corda e pretwist

La corda è in metri. Il pretwist è in gradi.

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

### CG e posizioni rotore

Tabella CG:

```text
tilt_deg   x_cg_mm   y_cg_mm   z_cg_mm
...
```

Tabella posizioni rotore:

```text
tilt_deg   r1_x_mm r1_y_mm r1_z_mm   r2_x_mm r2_y_mm r2_z_mm ... r6_x_mm r6_y_mm r6_z_mm
...
```

Il codice interpola le posizioni in base all'angolo di tilt.

### Tabelle aerodinamiche base della fusoliera

File:

```text
Fuselage_cd.txt
Fuselage_cl.txt
Fuselage_cm.txt
Fuselage_cc.txt
Fuselage_cn.txt
Fuselage_cll.txt
```

Formato:

```text
tilt_1   tilt_2   tilt_3   ...
alpha_1  coeff(alpha_1,tilt_1)  coeff(alpha_1,tilt_2)  coeff(alpha_1,tilt_3) ...
alpha_2  coeff(alpha_2,tilt_1)  coeff(alpha_2,tilt_2)  coeff(alpha_2,tilt_3) ...
...
```

Per tabelle laterali/direzionali come `cc/cn/cll`, i coefficienti possono rappresentare il valore a un angolo di sideslip o a una deflessione di riferimento. Mantenere la stessa convenzione nella configurazione del codice.

### Tabelle superfici di controllo

Per tabelle tipo elevatore:

```text
deflection_deg   alpha_deg   dCD   dCL   dCM
...
```

Per tabelle tipo timone/alettone:

```text
deflection_deg   alpha_deg   dCC   dCN   dCLL
...
```

## Database aerodinamico Excel

Il database fusoliera e superfici di controllo può essere memorizzato anche in un unico file Excel. MATLAB può leggere direttamente fogli diversi.

Un template sintetico pubblico è fornito in:

```text
data_templates/aero_database_template.xlsx
```

Copiarlo in `data/` oppure puntare `cfg.data.aero.excel_file` al suo percorso, poi sostituire le righe placeholder con i propri dati.

Fogli consigliati:

```text
base_aero
WL1 WL2 WR1 WR2 VL1 VL2 VR1 VR2
```

Per il database base fusoliera/cellula, sono supportati due layout Excel:

1. Layout a foglio singolo: tenere tutti gli angoli di tilt nacella/cellula in `base_aero` e includere `tilt_angle_deg` come prima colonna numerica.
2. Layout a fogli separati: mettere ogni angolo di tilt in un foglio separato, per esempio `base_0`, `base_30`, `base_60`, `base_90`. In questo caso impostare `cfg.data.aero.base_sheets` e `cfg.data.aero.base_sheet_tilt_angle_deg` in `RUN_ME.m`.

Esempio di impostazioni per fogli separati:

```matlab
cfg.switch.fuselage = "excel";
cfg.switch.controls = "excel";
cfg.data.aero.excel_file = 'aero_database.xlsx';
cfg.data.aero.base_sheets = {'base_0','base_30','base_60','base_90'};
cfg.data.aero.base_sheet_tilt_angle_deg = [0 30 60 90];
cfg.data.aero.control_surface_sheets = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
```

Per usare le vecchie tabelle txt, impostare `cfg.switch.fuselage = "lookup"` e `cfg.switch.controls = "lookup"`.

Schema `base_aero`:

```text
tilt_angle_deg   beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

Schema dei fogli base separati, quando l'angolo di tilt è fornito tramite `cfg.data.aero.base_sheet_tilt_angle_deg`:

```text
beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

Schema foglio superficie di controllo:

```text
deflection_deg   alpha_deg   dCD   dCL   dCm   dCC   dCn   dCl
...
```

I vecchi file Excel che includono ancora una colonna `Mach` in questi fogli fusoliera/superfici restano supportati, ma Mach non è richiesto dal modello fusoliera/superfici corrente.

I fogli Excel possono includere intestazioni testuali e note; il lettore MATLAB conserva solo righe numeriche complete. Le righe numeriche devono comunque formare una griglia di interpolazione completa per ogni variabile indipendente nel foglio.

Questo repository può includere esempi di formato e valori sintetici placeholder, ma non dati aerodinamici privati.

## Policy del repository pubblico

Il repository non deve includere:

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

Gli utenti devono generare i file MEX localmente eseguendo `RUN_RESPONSE_SIMULINK_SETUP`.

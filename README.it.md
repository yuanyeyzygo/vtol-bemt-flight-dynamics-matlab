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

## Indice Della Documentazione

| Esigenza | Sezione da leggere | File principali |
|---|---|---|
| Scegliere lo script di ingresso corretto | [Mappa Dei Punti Di Ingresso](#mappa-dei-punti-di-ingresso) | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Eseguire trim, derivate di stabilità e derivate di controllo | [Avvio rapido](#avvio-rapido), [MATLAB pala per pala](#1-matlab-pala-per-pala), [Output del trim](#output-del-trim) | `RUN_ME.m`, `RUN_TRIM_AND_STABILITY.m` |
| Scegliere dati `default`, `lookup` o `excel` | [Switch principali](#switch-principali), [Dati lookup](#dati-lookup), [Database aerodinamico Excel](#database-aerodinamico-excel) | `RUN_ME.m`, `data_templates/` |
| Capire i parametri pubblici di default | [Modello default](#modello-default) | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Eseguire risposta Simulink lenta ad alta fedeltà | [Simulink lento pala per pala](#2-simulink-lento-pala-per-pala), [Risposta Simulink](#risposta-simulink) | `CREATE_RESPONSE_SIMULINK_MODEL.m`, `VTOL_RESPONSE_SIMULINK.slx` |
| Eseguire risposta Simulink rapida quasi real-time | [Simulink rapido disco battente](#3-simulink-rapido-disco-battente), [Risposta Simulink](#risposta-simulink) | `RUN_RESPONSE_SIMULINK_SETUP.m`, `VTOL_RESPONSE_SIMULINK_MEX.slx` |
| Controllare l'ordine di stati, comandi e uscite | [Output del trim](#output-del-trim), [Risposta Simulink](#risposta-simulink) | `trim_results`, `out.x_sim` |
| Aggiungere tabelle lookup txt | [Dati lookup](#dati-lookup) | `data/` |
| Aggiungere un database aerodinamico Excel | [Database aerodinamico Excel](#database-aerodinamico-excel) | `data_templates/` |

## Mappa Dei Punti Di Ingresso

| File | Quando usarlo | Modello rotore | Dati supportati | Output principale |
|---|---|---|---|---|
| `RUN_ME.m` | Vuoi modificare direttamente trim, stabilità e derivate di controllo in un unico file interfaccia. | pala per pala | `default`, `lookup`, e `excel` per fusoliera/comandi | `trim_results` |
| `RUN_TRIM_AND_STABILITY.m` | Vuoi un wrapper compatto per il workflow MATLAB di trim e derivate. | pala per pala | segue la configurazione in stile `RUN_ME.m` | `trim_results` |
| `RUN_RESPONSE_SIMULINK_SETUP.m` | Vuoi eseguire prima il trim, poi generare e inizializzare un modello di risposta Simulink. | pala per pala o disco | `default`; il percorso dettagliato pala per pala può usare anche i dati lookup/Excel supportati | variabili nel base workspace e modello Simulink |
| `RUN_FAST_DISK_MEX_RESPONSE.m` | Vuoi eseguire il modello rapido Simulink/MEX già generato. | disco battente | solo percorso pubblico `default` | output Simulink `out` |
| `CREATE_RESPONSE_SIMULINK_MODEL.m` | Vuoi rigenerare esplicitamente il modello Simulink lento pala per pala. | pala per pala | stessi dati del workflow di risposta lento | `VTOL_RESPONSE_SIMULINK.slx` |
| `CREATE_RESPONSE_SIMULINK_MEX_MODEL.m` | Vuoi rigenerare esplicitamente il modello Simulink rapido disco/MEX. | disco battente | solo percorso pubblico `default` | `VTOL_RESPONSE_SIMULINK_MEX.slx` |

Il percorso normale è `RUN_ME.m` per trim e derivate, e `RUN_RESPONSE_SIMULINK_SETUP.m` per la simulazione di risposta.

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

I valori sotto sono default pubblici di esempio. Servono per test, distribuzione e verifica del codice, non per rappresentare una configurazione privata di velivolo.

| Area | Parametri | Default pubblico | Unità | Significato in modalità default | Da modificare in |
|---|---|---:|---|---|---|
| Ambiente | `cfg.environment.rho_kg_m3`, `gravity_m_s2`, `use_isa`, `initial_altitude_m` | `1.225`, `9.81`, `false`, `0` | kg/m^3, m/s^2, -, m | La modalità a densità costante usa `rho_kg_m3`. Se `use_isa = true`, il trim usa la densità ISA a `initial_altitude_m`, e la risposta non lineare aggiorna la densità dallo spostamento `z` negli assi terra. | `RUN_ME.m`, `RUN_RESPONSE.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Veicolo | `cfg.vehicle.mass_kg` | `1900` | kg | Massa del velivolo usata nel bilancio di trim e nelle equazioni di risposta. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Inerzia veicolo | `cfg.vehicle.inertia_kg_m2` | `[1966.5, 5245.3, 3282.7]` | kg m^2 | Vettore di inerzia assi corpo ordinato come `[Ixx Iyy Izz]`. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Dimensione e velocità rotore | `cfg.rotor.radius_m`, `blade_count`, `omega_rad_s` | `1.3`, `5`, `90` | m, -, rad/s | Scala rotore pubblica, numero di pale e velocità angolare nominale. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Flappeggio rotore | `cfg.rotor.flap_inertia_kg_m2`, `flap_spring_nm_rad` | `2.25`, `16000` | kg m^2, N m/rad | Inerzia di flappeggio pala e rigidezza equivalente di flappeggio. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Discretizzazione rotore | `cfg.rotor.blade_element_count`, `azimuth_steps` | `10`, `72` | -, - | Elementi lungo la pala e stazioni azimutali usati nel calcolo rotore dettagliato. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Sezioni profilo rotore | `cfg.rotor.airfoil_section_edges` | `[0.25, 0.40, 0.50, 0.80, 0.92]` | r/R | Posizioni radiali di separazione per le sei sezioni di profilo rotore. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Verso di rotazione | `cfg.rotor.rotational_direction` | `[1, -1, 1, -1, 1, -1]` | - | Segno di rotazione dei rotori da 1 a 6. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Opzioni modello rotore | `cfg.rotor.flap_model`, `flap_integrator`, `inflow_model` | dipende dal flusso | - | Seleziona flappeggio pala per pala o disco, integratore degli stati di flappeggio e comportamento dell'inflow. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| CG default | `cfg.defaults.geometry.x_cg_mm`, `y_cg_mm`, `z_cg_mm` | `3554.3`, `0`, `-588.7` | mm | Centro di gravità pubblico default quando il lookup della geometria è disattivato. | `RUN_ME.m` |
| Posizioni rotore default | `cfg.defaults.geometry.rotor_position_coeffs_mm` | matrice 6-by-7 | mm | Tabella compatta di coefficienti che muove ogni rotore con il tilt della nacella quando il lookup posizione rotore è disattivato. | `RUN_ME.m` |
| Corda pala | `cfg.defaults.chord_m` | `0.20264354` | m | Corda costante pubblica default quando il lookup della corda è disattivato. | `RUN_ME.m` |
| Pretwist pala | `cfg.defaults.pretwist_root_deg`, `pretwist_tip_deg` | `19.279996`, `-6.276289` | deg | Descrizione semplificata del pretwist dal lato radice al lato punta quando il lookup è disattivato. | `RUN_ME.m` |
| Profilo rotore default | `cfg.defaults.airfoil.*` | `cl_alpha=5.579842`, `cl_max=1.134702`, `cd0=0.010150`, `cd_alpha2=1.758467` | misto | Parametri semplificati di pendenza della portanza, limite di portanza e forma della resistenza quando il lookup profilo è disattivato. | `RUN_ME.m` |
| Riferimenti fusoliera | `cfg.fuselage.reference_area_m2`, `mean_aero_chord_m`, `span_m` | `14.41`, `1.31`, `12` | m^2, m, m | Dimensioni di riferimento per coefficienti aerodinamici default di fusoliera e ala fissa. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Aereo fusoliera default | `cfg.defaults.fuselage.*` | vedi `RUN_ME.m` | misto | Parametri semplificati di resistenza, portanza, momento di beccheggio, forza laterale, momento d'imbardata e momento di rollio. | `RUN_ME.m` |
| Controlli ala fissa | `cfg.defaults.controls.elevator`, `rudder`, `aileron` | vedi `RUN_ME.m` | per rad | Incrementi di coefficiente default quando lookup o dati Excel delle superfici sono disattivati. | `RUN_ME.m` |
| Miscelazione controlli | `cfg.control_blend.*` | attiva, basata sul tilt, `sincos` | - | Distribuisce i canali di comando tra rotore e ala fissa in funzione del tilt della nacella. | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| Derivate dinamiche | `cfg.aero.dynamic_derivatives.*` | opzionale | misto | Correzioni opzionali delle derivate aerodinamiche dinamiche della cellula. | `RUN_ME.m` |

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

Le schedulazioni opzionali `linear` e `smoothstep` restano supportate per studi precedenti. In questi modi il codice calcola un peso fixed-wing scalare dal tilt della nacella, poi usa `rotor_weight = 1 - fixed_weight`.

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

## Atmosfera E Quota

Il modello ambiente può usare densità costante oppure densità ISA:

```matlab
cfg.environment.use_isa = false;        % densità costante rho_kg_m3
cfg.environment.use_isa = true;         % densità ISA
cfg.environment.initial_altitude_m = 0; % quota iniziale sopra il livello medio del mare
```

Quando ISA è attiva, trim e derivate di stabilità usano la densità a `initial_altitude_m`. Nella risposta non lineare MATLAB e Simulink, il 12o stato di risposta è lo spostamento `z` negli assi terra, positivo verso il basso. L'aggiornamento della densità usa quindi:

```text
altitude_m = initial_altitude_m - z
```

La struttura di risposta salva:

```matlab
trim_results.response.altitude_history
trim_results.response.density_history
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
cfg.response.pilot_stick
cfg.response.pilot_stick_to_control_gain_deg
cfg.response.control_delta              % incremento diretto opzionale di debug
cfg.response.fixed_wing_control_delta   % incremento fisso opzionale di debug
cfg.response.rotor_tilt_angle_deg
```

`RUN_FAST_DISK_MEX_RESPONSE` non contiene parametri di dinamica del volo. Esegue solo il modello Simulink corrente usando le variabili del base workspace generate da `RUN_RESPONSE_SIMULINK_SETUP`.

Il modello Simulink contiene solo un'interfaccia stick pilota ad anello aperto:

```text
Pilot stick input = [collective, pitch, roll, yaw], normalizzato
Stick gain        = scala del comando a stick pieno in deg
Rotor tilt input  = sei angoli di tilt delle gondole in deg
```

Il modello generato non contiene una legge di controllo in anello chiuso. Gli utenti possono sostituire la sorgente stick costante con un joystick, un segnale di prova o un'altra sorgente di comando ad anello aperto.

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

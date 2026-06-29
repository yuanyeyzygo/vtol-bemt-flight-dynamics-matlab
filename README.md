# VTOL BEMT Flight-Dynamics MATLAB

MATLAB tools for VTOL/eVTOL trim, rotor BEMT, flapping dynamics, stability and control derivatives, and nonlinear response simulation.

本仓库提供 VTOL/eVTOL 飞行动力学计算工具，包括配平、旋翼 BEMT、挥舞动力学、稳定性/操纵导数，以及非线性响应仿真。

The repository is organized around three workflows:

本仓库主要分为三条工作流：

1. **MATLAB individual-blade workflow**: `RUN_ME.m` / `RUN_TRIM_AND_STABILITY.m`. This is the detailed trim, stability-derivative, and control-derivative workflow. It uses an individual-blade flapping model with BEMT, and can run with either public `default` data or user-provided `lookup` / `excel` data.
2. **Simulink individual-blade workflow**: `RUN_RESPONSE_SIMULINK_SETUP.m` with `cfg.rotor.flap_model = "blade"`. This uses the same detailed individual-blade rotor model for nonlinear response in Simulink. It can also use `default`, `lookup`, or `excel` data, but it is computationally slow and is mainly for high-fidelity checks.
3. **Fast Simulink disk-flap workflow**: `RUN_RESPONSE_SIMULINK_SETUP.m` with the default `cfg.rotor.flap_model = "disk"`, followed by `RUN_FAST_DISK_MEX_RESPONSE`. This uses a reduced whole-disk flapping model and MEX acceleration for real-time-style response simulation. This fast workflow is intended for the public `default` model and does not support the full lookup-table path.

1 **MATLAB 单片桨叶工作流**：`RUN_ME.m` / `RUN_TRIM_AND_STABILITY.m`。这是细节版配平、稳定性导数和操纵导数计算流程，使用单片桨叶挥舞模型和 BEMT，可选择公开的 `default` 数据，也可使用用户自己的 `lookup` / `excel` 数据。

2 **Simulink 单片桨叶工作流**：`RUN_RESPONSE_SIMULINK_SETUP.m`，并设置 `cfg.rotor.flap_model = "blade"`。这一路径在 Simulink 非线性响应中使用同样的细节版单片桨叶旋翼模型，也可使用 `default`、`lookup` 或 `excel` 数据，但计算较慢，主要用于高精度核对。

3 **快速 Simulink 整体桨盘工作流**：默认 `cfg.rotor.flap_model = "disk"` 的 `RUN_RESPONSE_SIMULINK_SETUP.m`，然后运行 `RUN_FAST_DISK_MEX_RESPONSE`。这一路径使用降阶整体桨盘挥舞模型和 MEX 加速，面向实时/准实时响应仿真；快速模型用于公开 `default` 模型，不支持完整 lookup table 路径。

Useful references:

参考文献：

- Individual-blade rotor modeling: Stephen Rutherford, *Simulation techniques for the study of the manoeuvring of advanced rotorcraft configurations*, PhD thesis, University of Glasgow, 1997. <https://theses.gla.ac.uk/30844/>
- Disk/tip-path-plane flapping dynamics: R. T. N. Chen, *Effects of primary rotor parameters on flapping dynamics*, NASA TP-1431, 1980. <https://ntrs.nasa.gov/citations/19800006879>

Private aerodynamic lookup data are not included. The code can run in `default` mode without a `data/` folder. Lookup and Excel formats are documented below so users can add their own data.

本仓库不包含私有气动查表数据。程序在 `default` 模式下不需要 `data/` 文件夹即可运行。下文给出 lookup 和 Excel 数据格式，用户可以放入自己的数据。

## Quick Start / 快速开始

Open MATLAB in this folder.

在 MATLAB 中打开本文件夹。

Trim, stability and control derivatives:

配平、稳定性和操纵导数：

```matlab
RUN_ME
```

Nonlinear Simulink/MEX response:

非线性 Simulink/MEX 响应：

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

`RUN_RESPONSE_SIMULINK_SETUP` trims the aircraft, exports the current initial conditions to the MATLAB base workspace, rebuilds the fast disk-flap MEX for the current setup, and regenerates the Simulink model.

`RUN_RESPONSE_SIMULINK_SETUP` 会先完成配平，把当前初始条件写入 MATLAB base workspace，然后根据当前设置重新编译快速整体桨盘 MEX，并重新生成 Simulink 模型。

`RUN_FAST_DISK_MEX_RESPONSE` only runs Simulink. It does not change aerodynamic or flight-dynamics parameters and does not load cached initialization MAT files.

`RUN_FAST_DISK_MEX_RESPONSE` 只负责运行 Simulink，不修改任何气动或飞行动力学参数，也不读取缓存初始化 MAT 文件。

## Main Switches / 主要开关

Each workflow has its own entry script. Where lookup data are supported, the data-source switches use the same names.

三条工作流有各自的入口脚本；在支持查表数据的工作流中，数据源开关名称保持一致。

### 1. MATLAB Individual-Blade / MATLAB 单片桨叶

Use `RUN_ME.m` for trim, stability derivatives, and control derivatives. This workflow uses the detailed individual-blade rotor model.

使用 `RUN_ME.m` 进行配平、稳定性导数和操纵导数计算。该工作流使用细节版单片桨叶模型。

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

`default` runs without private data. `lookup` uses txt lookup tables in `data/`. `excel` is supported for fuselage and control-surface aerodynamics.

`default` 不需要私有数据；`lookup` 使用 `data/` 中的 txt 查表；机身和舵面气动也支持 `excel` 工作簿。

### 2. Slow Simulink Individual-Blade / 慢速 Simulink 单片桨叶

Use `RUN_RESPONSE_SIMULINK_SETUP.m` and select the blade model before setup:

使用 `RUN_RESPONSE_SIMULINK_SETUP.m`，并在 setup 前选择单片桨叶模型：

```matlab
flap_model_override = "blade";
RUN_RESPONSE_SIMULINK_SETUP
```

Inside `RUN_RESPONSE_SIMULINK_SETUP.m`, the same data-source switches are used:

在 `RUN_RESPONSE_SIMULINK_SETUP.m` 内部，数据源开关名称与 MATLAB 单片桨叶工作流一致：

```matlab
data_mode = "default";          % "default" or "lookup"
aero_database_mode = data_mode; % "default", "lookup", or "excel"
cfg.rotor.flap_model = requested_flap_model;  % "blade"
```

This path can use `default`, `lookup`, or `excel`, but it is slow because it evaluates the detailed individual-blade response in Simulink.

这一路径可以使用 `default`、`lookup` 或 `excel`，但由于在 Simulink 中计算细节版单片桨叶响应，速度较慢。

### 3. Fast Simulink Disk-Flap / 快速 Simulink 整体桨盘

Use the default disk model setup and then run the generated fast model:

使用默认整体桨盘 setup，然后运行生成的快速模型：

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

For this workflow, keep the public default data path:

这一路径应保持公开 default 数据路径：

```matlab
data_mode = "default";
aero_database_mode = "default";
cfg.rotor.flap_model = "disk";
cfg.response.compile_mex = true;
```

The fast MEX path is intended for the numeric default/disk model. It does not support the full lookup-table path. `RUN_FAST_DISK_MEX_RESPONSE` only runs the already generated Simulink/MEX setup; it has no aerodynamic data switches.

快速 MEX 路径面向数值化的 default/整体桨盘模型，不支持完整 lookup table 路径。`RUN_FAST_DISK_MEX_RESPONSE` 只运行已经生成的 Simulink/MEX setup，本身没有气动数据开关。

## Default Model / 缺省模型

The default model is intentionally simple and public-friendly. Important parameters are exposed in `RUN_ME.m` and `RUN_RESPONSE_SIMULINK_SETUP.m`.

缺省模型是为了开源和快速测试设置的简化模型。主要参数都暴露在 `RUN_ME.m` 和 `RUN_RESPONSE_SIMULINK_SETUP.m` 中。

Common vehicle and environment parameters:

常用整机和环境参数：

```matlab
cfg.environment.rho_kg_m3
cfg.environment.gravity_m_s2
cfg.vehicle.mass_kg
cfg.vehicle.inertia_kg_m2   % [Ixx Iyy Izz]
```

Rotor parameters:

旋翼参数：

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

Default rotor geometry uses a compact analytic form for each rotor:

缺省旋翼位置使用下面的解析形式：

```matlab
x = x0 + x_cos*cos(tilt) + x_sin*sin(tilt)
y = y0
z = z0 + z_cos*cos(tilt) + z_sin*sin(tilt)
```

The coefficients are stored in:

系数存放在：

```matlab
cfg.defaults.geometry.rotor_position_coeffs_mm
```

Default rotor airfoil and blade planform:

缺省桨叶翼型和几何：

```matlab
cfg.defaults.chord_m
cfg.defaults.pretwist_root_deg
cfg.defaults.pretwist_tip_deg
cfg.defaults.airfoil.cl_alpha_per_rad
cfg.defaults.airfoil.cl_max
cfg.defaults.airfoil.cd0
cfg.defaults.airfoil.cd_alpha2
```

Default fuselage and control parameters:

缺省机身和舵面参数：

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

Dynamic aerodynamic derivatives are optional and exposed through:

动导数为可选项，接口为：

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

## Trim Outputs / 配平输出

The main result is `trim_results`.

主要结果变量是 `trim_results`。

```matlab
trim_results.trim_table
trim_results.stability_A
trim_results.control_B
trim_results.final_trim_var
trim_results.last_eigenvalues
trim_results.rotor_locations_m
trim_results.speed_mps
```

The state order in the linearized `A` matrix is:

线性化 `A` 矩阵的状态顺序为：

```text
[u, w, q, theta, v, p, phi, r]
```

The control order in `B` is:

`B` 矩阵的控制量顺序为：

```text
[collective, longitudinal, lateral, yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

If `cfg.control_blend.enabled = true` and `cfg.control_blend.apply_to_trim = true`, the trim/control variables after collective become blended pilot-equivalent commands:

如果启用 `cfg.control_blend.enabled = true` 且 `cfg.control_blend.apply_to_trim = true`，总距之后的配平/控制量会变为综合控制通道：

```text
[collective, blend_pitch, blend_roll, blend_yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

The current default transition schedule is based on nacelle tilt angle:

当前默认过渡操纵分配按短舱倾转角调度：

```matlab
cfg.control_blend.independent_variable = "tilt_angle";
cfg.control_blend.tilt_helicopter_deg = 90;
cfg.control_blend.tilt_fixedwing_deg = 0;
cfg.control_blend.schedule = "sincos";
```

With `schedule = "sincos"`, the code uses the actual tilt angle directly:

当 `schedule = "sincos"` 时，程序直接使用当前短舱倾转角：

```text
tilt_limited = clamp(tilt_angle_deg, min(tilt_helicopter_deg, tilt_fixedwing_deg),
                                     max(tilt_helicopter_deg, tilt_fixedwing_deg))
rotor_weight = clamp(sind(tilt_limited), 0, 1)
fixed_weight = clamp(cosd(tilt_limited), 0, 1)
```

For example, at 90 deg tilt the rotor weight is 1 and fixed-wing weight is 0. At 0 deg tilt the rotor weight is 0 and fixed-wing weight is 1. At 60 deg tilt the weights are `sind(60)` and `cosd(60)`, not a complementary linear pair.

例如，90 度倾转时旋翼权重为 1、固定翼权重为 0。0 度倾转时旋翼权重为 0、固定翼权重为 1。60 度倾转时权重是 `sind(60)` 和 `cosd(60)`，不是互补的线性分配。

The three blended pilot-equivalent channels are allocated as:

三个综合操纵通道按下面的程序公式分配：

```text
rotor_longitudinal_deg = rotor_weight * rotor_gains(1) * blend_pitch
rotor_lateral_deg      = rotor_weight * rotor_gains(2) * blend_roll
rotor_yaw_deg          = rotor_weight * rotor_gains(3) * blend_yaw

fixed_pitch_deg += fixed_weight * fixed_gains(1) * blend_pitch
fixed_roll_deg  += fixed_weight * fixed_gains(2) * blend_roll
fixed_yaw_deg   += fixed_weight * fixed_gains(3) * blend_yaw
```

Both `rotor_gains` and `fixed_gains` are ordered as `[pitch, roll, yaw]`.

`rotor_gains` 和 `fixed_gains` 的顺序都是 `[pitch, roll, yaw]`。

The optional `linear` and `smoothstep` schedules are still supported for older studies. In those modes the code first computes a scalar fixed-wing weight from speed or tilt angle, then uses `rotor_weight = 1 - fixed_weight`.

为了兼容旧算例，程序仍保留 `linear` 和 `smoothstep`。在这两种模式下，程序先由速度或倾转角计算一个固定翼权重，然后使用 `rotor_weight = 1 - fixed_weight`。

The fixed-wing command vector is ordered as `[fixed_pitch, fixed_yaw, fixed_roll]` in degrees. When WL/WR/VL/VR physical-surface lookup or Excel sheets are used, these three channels are mapped to physical surface deflections by `cfg.controls.surface_mixing_matrix`. If that matrix is empty, the default mapping is:

固定翼舵面通道顺序为 `[fixed_pitch, fixed_yaw, fixed_roll]`，单位为度。当使用 WL/WR/VL/VR 物理舵面查表或 Excel sheet 时，这三个通道会通过 `cfg.controls.surface_mixing_matrix` 映射到物理舵面偏角。如果该矩阵为空，默认映射为：

```text
gain = cfg.controls.channel_to_physical_gain   % default 0.5

DVL1 = gain * (fixed_yaw - fixed_pitch)
DVR1 = gain * (fixed_yaw + fixed_pitch)
DVL2 = gain * (fixed_yaw - fixed_pitch)
DVR2 = gain * (fixed_yaw + fixed_pitch)

DWL1 = DWR1 = fixed_roll
DWL2 = DWR2 = fixed_roll
```

To use a custom allocator, set:

如果需要自定义舵面分配，设置：

```matlab
cfg.controls.channel_names = {'pitch','yaw','roll'};
cfg.controls.physical_surface_names = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
cfg.controls.surface_mixing_matrix = M;  % rows: physical surfaces, columns: channels
cfg.controls.surface_bias_deg = b;       % optional bias for each physical surface
```

## Simulink Response / Simulink 响应

The Simulink response workflow is:

Simulink 响应流程为：

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

The setup script is the only place where response sample time and trim condition should be changed:

响应步长和配平条件应只在 setup 脚本中修改：

```matlab
cfg.trim.tilt_angle_deg
cfg.trim.speed_mps
cfg.response.dt_s
cfg.response.control_delta
cfg.response.fixed_wing_control_delta
cfg.response.rotor_tilt_angle_deg
```

`RUN_FAST_DISK_MEX_RESPONSE` has no flight-dynamics parameters. It only runs the current Simulink model using base-workspace variables generated by `RUN_RESPONSE_SIMULINK_SETUP`.

`RUN_FAST_DISK_MEX_RESPONSE` 内部没有飞行动力学参数。它只使用 `RUN_RESPONSE_SIMULINK_SETUP` 生成的 base workspace 变量运行当前 Simulink 模型。

The Simulink model contains a simple control-change interface:

Simulink 模型中包含一个简单的控制扰动接口：

```text
Rotor control delta  = [collective, longitudinal, lateral, yaw]
Fixed-wing control   = [pitch, yaw, roll]
Rotor tilt input     = six nacelle tilt angles in deg
```

The example model uses two Step blocks summed into the collective channel, and constants for the remaining channels. Users can replace these blocks with their own controller outputs.

示例模型中用两个 Step 信号叠加到总距通道，其余通道先用常数。用户可以把这些块替换为自己的控制器输出。

`out.x_sim` uses the following 12-state order:

`out.x_sim` 的 12 个状态量顺序为：

```text
[u v w p q r phi theta psi x y z]
```

Coordinate meaning:

坐标含义：

```text
u, v, w      body-axis velocities
p, q, r      body-axis angular rates
phi,theta,psi Euler angles
x, y, z      earth/inertial positions
forces_sim   body-axis [Fx Fy Fz Mx My Mz]
```

## Lookup Data / 查表数据

Private lookup data are not included. If lookup mode is enabled, put files in:

私有查表数据不随仓库发布。若启用 lookup 模式，请把文件放在：

```text
data/
```

All txt files must contain numeric values only. Do not add headers, column names, units, comments, or explanatory text. The first line must already be data.

所有 txt 文件只能包含数值。不要加入表头、列名、单位、注释或说明文字。第一行必须直接是数据。

### Rotor Airfoil CL/CD Tables / 旋翼翼型 CL/CD 表

Files:

文件：

```text
CS1_cl.txt ... CS6_cl.txt
CS1_cd.txt ... CS6_cd.txt
```

Format:

格式：

```text
Mach_1   Mach_2   Mach_3   ...
alpha_1  value(alpha_1,Mach_1)  value(alpha_1,Mach_2)  value(alpha_1,Mach_3) ...
alpha_2  value(alpha_2,Mach_1)  value(alpha_2,Mach_2)  value(alpha_2,Mach_3) ...
...
```

### Chord and Pretwist / 弦长和负扭

Chord is in meters. Pretwist is in degrees.

弦长单位为米，负扭单位为度。

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

### CG and Rotor Positions / 重心和旋翼位置

CG table:

重心表：

```text
tilt_deg   x_cg_mm   y_cg_mm   z_cg_mm
...
```

Rotor position table:

旋翼位置表：

```text
tilt_deg   r1_x_mm r1_y_mm r1_z_mm   r2_x_mm r2_y_mm r2_z_mm ... r6_x_mm r6_y_mm r6_z_mm
...
```

The code interpolates positions by tilt angle.

程序会按倾转角插值旋翼位置。

### Fuselage Base Tables / 机身基础气动表

Files:

文件：

```text
Fuselage_cd.txt
Fuselage_cl.txt
Fuselage_cm.txt
Fuselage_cc.txt
Fuselage_cn.txt
Fuselage_cll.txt
```

Format:

格式：

```text
tilt_1   tilt_2   tilt_3   ...
alpha_1  coeff(alpha_1,tilt_1)  coeff(alpha_1,tilt_2)  coeff(alpha_1,tilt_3) ...
alpha_2  coeff(alpha_2,tilt_1)  coeff(alpha_2,tilt_2)  coeff(alpha_2,tilt_3) ...
...
```

For lateral/directional tables such as `cc/cn/cll`, coefficients may represent the value at a reference sideslip or control deflection. Keep the same convention in the code configuration.

对于 `cc/cn/cll` 等横航向表，系数可以表示参考侧滑角或参考偏转角下的值。使用时应保持程序配置中的定义一致。

### Control Surface Tables / 舵面表

For elevator-like tables:

升降舵类表：

```text
deflection_deg   alpha_deg   dCD   dCL   dCM
...
```

For rudder/aileron-like tables:

方向舵/副翼类表：

```text
deflection_deg   alpha_deg   dCC   dCN   dCLL
...
```

## Excel Aerodynamic Database / Excel 气动数据库

The fuselage and control-surface database can also be stored in one Excel file. MATLAB can read different sheets directly.

机身和舵面气动数据库也可以放在一个 Excel 文件中。MATLAB 可以直接读取不同 sheet。

A public synthetic template is provided at:

仓库中提供了一个不含私有数据的合成模板：

```text
data_templates/aero_database_template.xlsx
```

Copy it to `data/` or point `cfg.data.aero.excel_file` to its path, then replace the placeholder rows with your own data.

可以把它复制到 `data/` 中，或把 `cfg.data.aero.excel_file` 指向该路径，然后用自己的数据替换其中的占位数值。

Recommended sheets:

推荐 sheet：

```text
base_aero
WL1 WL2 WR1 WR2 VL1 VL2 VR1 VR2
```

For the base fuselage/airframe database, two Excel layouts are supported:

1. Single-sheet layout: keep all nacelle/airframe tilt angles in `base_aero` and include `tilt_angle_deg` as the first numeric column.
2. Split-sheet layout: put each tilt angle in a separate sheet, for example `base_0`, `base_30`, `base_60`, `base_90`. In this case set `cfg.data.aero.base_sheets` and `cfg.data.aero.base_sheet_tilt_angle_deg` in `RUN_ME.m`.

基础机身/整机气动数据库支持两种 Excel 组织方式：

1. 单 sheet：所有短舱/机体倾转角都放在 `base_aero`，第一列必须是 `tilt_angle_deg`。
2. 分 sheet：每个倾转角一个 sheet，例如 `base_0`、`base_30`、`base_60`、`base_90`。这种情况下需要在 `RUN_ME.m` 中设置 `cfg.data.aero.base_sheets` 和 `cfg.data.aero.base_sheet_tilt_angle_deg`。

Example split-sheet settings:

```matlab
cfg.switch.fuselage = "excel";
cfg.switch.controls = "excel";
cfg.data.aero.excel_file = 'aero_database.xlsx';
cfg.data.aero.base_sheets = {'base_0','base_30','base_60','base_90'};
cfg.data.aero.base_sheet_tilt_angle_deg = [0 30 60 90];
cfg.data.aero.control_surface_sheets = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
```

To use the old txt tables instead, set `cfg.switch.fuselage = "lookup"` and `cfg.switch.controls = "lookup"`.

如果使用原来的 txt 表格，则设置 `cfg.switch.fuselage = "lookup"` 和 `cfg.switch.controls = "lookup"`。

`base_aero` schema:

`base_aero` 格式：

```text
tilt_angle_deg   beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

Split base-sheet schema, when the tilt angle is supplied through `cfg.data.aero.base_sheet_tilt_angle_deg`:

分 sheet 格式如下；此时倾转角由 `cfg.data.aero.base_sheet_tilt_angle_deg` 给出：

```text
beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

Control surface sheet schema:

舵面 sheet 格式：

```text
deflection_deg   alpha_deg   dCD   dCL   dCm   dCC   dCn   dCl
...
```

Older Excel files that still include a `Mach` column in these fuselage/control-surface sheets remain supported, but Mach is not required by the current fuselage/control-surface model.

旧版 Excel 如果已经包含 `Mach` 列仍然可以读取，但当前机身/舵面模型不要求这一列。

Excel sheets may include text headers and notes; the MATLAB reader keeps only complete numeric rows. The numeric rows must still form a complete interpolation grid for every independent variable in the sheet.

Excel sheet 中可以包含文本表头和说明；MATLAB 读取时只保留完整数值行。但是数值行必须覆盖该 sheet 中所有自变量的完整插值网格。

This repository may include format examples and synthetic placeholder values, but not private aerodynamic data.

本仓库可以包含格式示例和合成占位数值，但不包含私有气动数据。

## Public Repository Policy / 开源仓库数据原则

The repository should not include:

仓库不应包含：

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

Users should generate MEX files locally by running `RUN_RESPONSE_SIMULINK_SETUP`.

用户应通过运行 `RUN_RESPONSE_SIMULINK_SETUP` 在本地生成 MEX 文件。

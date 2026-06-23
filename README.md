# VTOL BEMT Flight-Dynamics MATLAB

MATLAB tools for VTOL/eVTOL trim, rotor BEMT, flapping dynamics, stability and control derivatives, and nonlinear response simulation.

本仓库提供 VTOL/eVTOL 飞行动力学计算工具，包括配平、旋翼 BEMT、挥舞动力学、稳定性/操纵导数，以及非线性响应仿真。

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
out = RUN_FAST_DISK_MEX_RESPONSE(1.5);
```

`RUN_RESPONSE_SIMULINK_SETUP` trims the aircraft, exports the current initial conditions to the MATLAB base workspace, rebuilds the fast disk-flap MEX for the current setup, and regenerates the Simulink model.

`RUN_RESPONSE_SIMULINK_SETUP` 会先完成配平，把当前初始条件写入 MATLAB base workspace，然后根据当前设置重新编译快速整体桨盘 MEX，并重新生成 Simulink 模型。

`RUN_FAST_DISK_MEX_RESPONSE` only runs Simulink. It does not change aerodynamic or flight-dynamics parameters and does not load cached initialization MAT files.

`RUN_FAST_DISK_MEX_RESPONSE` 只负责运行 Simulink，不修改任何气动或飞行动力学参数，也不读取缓存初始化 MAT 文件。

## Main Switches / 主要开关

The main interface is `RUN_ME.m`.

主要接口在 `RUN_ME.m` 中。

```matlab
data_mode = "default";          % "default" or "lookup"
aero_database_mode = data_mode; % "default", "lookup", or "excel"

cfg.switch.geometry        = data_mode;
cfg.switch.rotor_positions = data_mode;
cfg.switch.chord           = data_mode;
cfg.switch.pretwist        = data_mode;
cfg.switch.airfoil         = data_mode;
cfg.switch.fuselage        = aero_database_mode;
cfg.switch.controls        = aero_database_mode;
```

Use `default` to run without private data. Use `lookup` or `excel` when user-provided files are available in `data/`.

选择 `default` 时不需要私有数据。选择 `lookup` 或 `excel` 时，用户需要把自己的数据文件放在 `data/` 中。

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

## Simulink Response / Simulink 响应

The Simulink response workflow is:

Simulink 响应流程为：

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(1.5);
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

Recommended sheets:

推荐 sheet：

```text
base_aero
WL1 WL2 WR1 WR2 VL1 VL2 VR1 VR2
```

`base_aero` schema:

`base_aero` 格式：

```text
tilt_deg   alpha_deg   CD   CL   CM   CC   CN   CLL
...
```

Control surface sheet schema:

舵面 sheet 格式：

```text
deflection_deg   alpha_deg   dCD   dCL   dCM   dCC   dCN   dCLL
...
```

This repository may include format examples, but not private aerodynamic data.

本仓库可以包含格式示例，但不包含私有气动数据。

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

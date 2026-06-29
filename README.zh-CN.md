# VTOL BEMT 飞行动力学 MATLAB 程序

语言： [English](README.md) | [中文](README.zh-CN.md) | [Français](README.fr.md) | [Italiano](README.it.md)

本仓库提供 VTOL/eVTOL 飞行动力学计算工具，包括配平、旋翼 BEMT、挥舞动力学、稳定性/操纵导数，以及非线性响应仿真。

本仓库主要分为三条工作流：

1. **MATLAB 单片桨叶工作流**：`RUN_ME.m` / `RUN_TRIM_AND_STABILITY.m`。这是细节版配平、稳定性导数和操纵导数计算流程，使用单片桨叶挥舞模型和 BEMT，可选择公开的 `default` 数据，也可使用用户自己的 `lookup` / `excel` 数据。
2. **Simulink 单片桨叶工作流**：`RUN_RESPONSE_SIMULINK_SETUP.m`，并设置 `cfg.rotor.flap_model = "blade"`。这一路径在 Simulink 非线性响应中使用同样的细节版单片桨叶旋翼模型，也可使用 `default`、`lookup` 或 `excel` 数据，但计算较慢，主要用于高精度核对。
3. **快速 Simulink 整体桨盘工作流**：默认 `cfg.rotor.flap_model = "disk"` 的 `RUN_RESPONSE_SIMULINK_SETUP.m`，然后运行 `RUN_FAST_DISK_MEX_RESPONSE`。这一路径使用降阶整体桨盘挥舞模型和 MEX 加速，面向实时/准实时响应仿真；快速模型用于公开 `default` 模型，不支持完整 lookup table 路径。

参考文献：

- 单片桨叶旋翼建模：Stephen Rutherford, *Simulation techniques for the study of the manoeuvring of advanced rotorcraft configurations*, PhD thesis, University of Glasgow, 1997. <https://theses.gla.ac.uk/30844/>
- 整体桨盘/桨尖路径平面挥舞动力学：R. T. N. Chen, *Effects of primary rotor parameters on flapping dynamics*, NASA TP-1431, 1980. <https://ntrs.nasa.gov/citations/19800006879>

本仓库不包含私有气动查表数据。程序在 `default` 模式下不需要 `data/` 文件夹即可运行。下文给出 lookup 和 Excel 数据格式，用户可以放入自己的数据。

## 快速开始

在 MATLAB 中打开本文件夹。

配平、稳定性和操纵导数：

```matlab
RUN_ME
```

非线性 Simulink/MEX 响应：

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

`RUN_RESPONSE_SIMULINK_SETUP` 会先完成配平，把当前初始条件写入 MATLAB base workspace，然后根据当前设置重新编译快速整体桨盘 MEX，并重新生成 Simulink 模型。

`RUN_FAST_DISK_MEX_RESPONSE` 只负责运行 Simulink，不修改任何气动或飞行动力学参数，也不读取缓存初始化 MAT 文件。

## 主要开关

三条工作流有各自的入口脚本；在支持查表数据的工作流中，数据源开关名称保持一致。

### 1. MATLAB 单片桨叶

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

`default` 不需要私有数据；`lookup` 使用 `data/` 中的 txt 查表；机身和舵面气动也支持 `excel` 工作簿。

### 2. 慢速 Simulink 单片桨叶

使用 `RUN_RESPONSE_SIMULINK_SETUP.m`，并在 setup 前选择单片桨叶模型：

```matlab
flap_model_override = "blade";
RUN_RESPONSE_SIMULINK_SETUP
```

在 `RUN_RESPONSE_SIMULINK_SETUP.m` 内部，数据源开关名称与 MATLAB 单片桨叶工作流一致：

```matlab
data_mode = "default";          % "default" or "lookup"
aero_database_mode = data_mode; % "default", "lookup", or "excel"
cfg.rotor.flap_model = requested_flap_model;  % "blade"
```

这一路径可以使用 `default`、`lookup` 或 `excel`，但由于在 Simulink 中计算细节版单片桨叶响应，速度较慢。

### 3. 快速 Simulink 整体桨盘

使用默认整体桨盘 setup，然后运行生成的快速模型：

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

这一路径应保持公开 default 数据路径：

```matlab
data_mode = "default";
aero_database_mode = "default";
cfg.rotor.flap_model = "disk";
cfg.response.compile_mex = true;
```

快速 MEX 路径面向数值化的 default/整体桨盘模型，不支持完整 lookup table 路径。`RUN_FAST_DISK_MEX_RESPONSE` 只运行已经生成的 Simulink/MEX setup，本身没有气动数据开关。

## 缺省模型

缺省模型是为了开源和快速测试设置的简化模型。主要参数都暴露在 `RUN_ME.m` 和 `RUN_RESPONSE_SIMULINK_SETUP.m` 中。

下表中的数值是公开示例缺省值，用于测试、传播和代码核对，不代表任何私有构型数据。

| 类别 | 参数 | 公开缺省值 | 单位 | 缺省模式含义 | 修改位置 |
|---|---|---:|---|---|---|
| 环境 | `cfg.environment.rho_kg_m3`, `cfg.environment.gravity_m_s2` | `1.225`, `9.81` | kg/m^3, m/s^2 | 气动力计算使用的空气密度，以及配平/运动方程使用的重力加速度。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 整机 | `cfg.vehicle.mass_kg` | `1900` | kg | 配平力平衡和响应方程使用的整机质量。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 转动惯量 | `cfg.vehicle.inertia_kg_m2` | `[1966.5, 5245.3, 3282.7]` | kg m^2 | 体轴系惯量，顺序为 `[Ixx Iyy Izz]`。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 旋翼尺寸和转速 | `cfg.rotor.radius_m`, `blade_count`, `omega_rad_s` | `1.3`, `5`, `90` | m, -, rad/s | 公开缺省旋翼尺度、桨叶片数和名义角速度。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 旋翼挥舞 | `cfg.rotor.flap_inertia_kg_m2`, `flap_spring_nm_rad` | `2.25`, `16000` | kg m^2, N m/rad | 桨叶挥舞惯量和等效挥舞刚度。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 旋翼离散 | `cfg.rotor.blade_element_count`, `azimuth_steps` | `10`, `72` | -, - | 细节旋翼计算使用的展向叶素数和方位角离散数。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 翼型分段 | `cfg.rotor.airfoil_section_edges` | `[0.25, 0.40, 0.50, 0.80, 0.92]` | r/R | 六段旋翼翼型的径向截止位置。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 旋翼转向 | `cfg.rotor.rotational_direction` | `[1, -1, 1, -1, 1, -1]` | - | 1 到 6 号旋翼的旋转方向符号。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 旋翼模型选项 | `cfg.rotor.flap_model`, `flap_integrator`, `inflow_model` | 随工作流变化 | - | 选择单片桨叶或整体桨盘挥舞模型、挥舞状态积分器，以及固定或均匀诱导速度模型。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 缺省重心 | `cfg.defaults.geometry.x_cg_mm`, `y_cg_mm`, `z_cg_mm` | `3554.3`, `0`, `-588.7` | mm | 关闭几何查表时使用的公开缺省重心。 | `RUN_ME.m` |
| 缺省旋翼位置 | `cfg.defaults.geometry.rotor_position_coeffs_mm` | 6-by-7 矩阵 | mm | 关闭旋翼位置查表时，用于描述各旋翼随短舱倾转角变化的位置系数表。 | `RUN_ME.m` |
| 桨叶弦长 | `cfg.defaults.chord_m` | `0.20264354` | m | 关闭弦长查表时使用的常值公开缺省弦长。 | `RUN_ME.m` |
| 桨叶负扭 | `cfg.defaults.pretwist_root_deg`, `pretwist_tip_deg` | `19.279996`, `-6.276289` | deg | 关闭负扭查表时使用的从根部侧到尖部侧的简化负扭描述。 | `RUN_ME.m` |
| 旋翼翼型缺省模型 | `cfg.defaults.airfoil.*` | `cl_alpha=5.579842`, `cl_max=1.134702`, `cd0=0.010150`, `cd_alpha2=1.758467` | 混合 | 关闭翼型查表时使用的简化升力斜率、升力限幅和阻力形状参数。 | `RUN_ME.m` |
| 机身参考量 | `cfg.fuselage.reference_area_m2`, `mean_aero_chord_m`, `span_m` | `14.41`, `1.31`, `12` | m^2, m, m | 缺省机身/固定翼气动系数使用的参考面积、参考弦长和参考展长。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 机身缺省气动 | `cfg.defaults.fuselage.*` | 见 `RUN_ME.m` | 混合 | 简化的基础阻力、升力、俯仰力矩、侧力、航向力矩和滚转力矩系数参数。 | `RUN_ME.m` |
| 固定翼舵面 | `cfg.defaults.controls.elevator`, `rudder`, `aileron` | 见 `RUN_ME.m` | per rad | 关闭舵面 lookup 或 Excel 数据时使用的缺省舵面增量系数。 | `RUN_ME.m` |
| 操纵混合 | `cfg.control_blend.*` | 启用，按倾转角，`sincos` | - | 随短舱倾转角在旋翼操纵通道和固定翼舵面通道之间分配操纵量。 | `RUN_ME.m`, `RUN_RESPONSE_SIMULINK_SETUP.m` |
| 动导数 | `cfg.aero.dynamic_derivatives.*` | 可选 | 混合 | 可选的整机动态气动导数修正，通过外层接口开启或修改。 | `RUN_ME.m` |

## 配平输出

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

线性化 `A` 矩阵的状态顺序为：

```text
[u, w, q, theta, v, p, phi, r]
```

`B` 矩阵的控制量顺序为：

```text
[collective, longitudinal, lateral, yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

如果启用 `cfg.control_blend.enabled = true` 且 `cfg.control_blend.apply_to_trim = true`，总距之后的配平/控制量会变为综合控制通道：

```text
[collective, blend_pitch, blend_roll, blend_yaw, fixed_pitch, fixed_yaw, fixed_roll]
```

当前默认过渡操纵分配按短舱倾转角调度：

```matlab
cfg.control_blend.independent_variable = "tilt_angle";
cfg.control_blend.tilt_helicopter_deg = 90;
cfg.control_blend.tilt_fixedwing_deg = 0;
cfg.control_blend.schedule = "sincos";
```

当 `schedule = "sincos"` 时，程序直接使用当前短舱倾转角：

```text
tilt_limited = clamp(tilt_angle_deg, min(tilt_helicopter_deg, tilt_fixedwing_deg),
                                     max(tilt_helicopter_deg, tilt_fixedwing_deg))
rotor_weight = clamp(sind(tilt_limited), 0, 1)
fixed_weight = clamp(cosd(tilt_limited), 0, 1)
```

例如，90 度倾转时旋翼权重为 1、固定翼权重为 0。0 度倾转时旋翼权重为 0、固定翼权重为 1。60 度倾转时权重是 `sind(60)` 和 `cosd(60)`，不是互补的线性分配。

三个综合操纵通道按下面的程序公式分配：

```text
rotor_longitudinal_deg = rotor_weight * rotor_gains(1) * blend_pitch
rotor_lateral_deg      = rotor_weight * rotor_gains(2) * blend_roll
rotor_yaw_deg          = rotor_weight * rotor_gains(3) * blend_yaw

fixed_pitch_deg += fixed_weight * fixed_gains(1) * blend_pitch
fixed_roll_deg  += fixed_weight * fixed_gains(2) * blend_roll
fixed_yaw_deg   += fixed_weight * fixed_gains(3) * blend_yaw
```

`rotor_gains` 和 `fixed_gains` 的顺序都是 `[pitch, roll, yaw]`。

为了兼容旧算例，程序仍保留 `linear` 和 `smoothstep`。在这两种模式下，程序先由速度或倾转角计算一个固定翼权重，然后使用 `rotor_weight = 1 - fixed_weight`。

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

如果需要自定义舵面分配，设置：

```matlab
cfg.controls.channel_names = {'pitch','yaw','roll'};
cfg.controls.physical_surface_names = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
cfg.controls.surface_mixing_matrix = M;  % rows: physical surfaces, columns: channels
cfg.controls.surface_bias_deg = b;       % optional bias for each physical surface
```

## Simulink 响应

Simulink 响应流程为：

```matlab
RUN_RESPONSE_SIMULINK_SETUP
out = RUN_FAST_DISK_MEX_RESPONSE(2.0);
```

响应步长和配平条件应只在 setup 脚本中修改：

```matlab
cfg.trim.tilt_angle_deg
cfg.trim.speed_mps
cfg.response.dt_s
cfg.response.control_delta
cfg.response.fixed_wing_control_delta
cfg.response.rotor_tilt_angle_deg
```

`RUN_FAST_DISK_MEX_RESPONSE` 内部没有飞行动力学参数。它只使用 `RUN_RESPONSE_SIMULINK_SETUP` 生成的 base workspace 变量运行当前 Simulink 模型。

Simulink 模型中包含一个简单的控制扰动接口：

```text
Rotor control delta  = [collective, longitudinal, lateral, yaw]
Fixed-wing control   = [pitch, yaw, roll]
Rotor tilt input     = six nacelle tilt angles in deg
```

示例模型中用两个 Step 信号叠加到总距通道，其余通道先用常数。用户可以把这些块替换为自己的控制器输出。

`out.x_sim` 的 12 个状态量顺序为：

```text
[u v w p q r phi theta psi x y z]
```

坐标含义：

```text
u, v, w         体轴速度
p, q, r         体轴角速度
phi, theta, psi 欧拉角
x, y, z         地轴/惯性系位置
forces_sim      体轴 [Fx Fy Fz Mx My Mz]
```

## 查表数据

私有查表数据不随仓库发布。若启用 lookup 模式，请把文件放在：

```text
data/
```

所有 txt 文件只能包含数值。不要加入表头、列名、单位、注释或说明文字。第一行必须直接是数据。

### 旋翼翼型 CL/CD 表

文件：

```text
CS1_cl.txt ... CS6_cl.txt
CS1_cd.txt ... CS6_cd.txt
```

格式：

```text
Mach_1   Mach_2   Mach_3   ...
alpha_1  value(alpha_1,Mach_1)  value(alpha_1,Mach_2)  value(alpha_1,Mach_3) ...
alpha_2  value(alpha_2,Mach_1)  value(alpha_2,Mach_2)  value(alpha_2,Mach_3) ...
...
```

### 弦长和负扭

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

### 重心和旋翼位置

重心表：

```text
tilt_deg   x_cg_mm   y_cg_mm   z_cg_mm
...
```

旋翼位置表：

```text
tilt_deg   r1_x_mm r1_y_mm r1_z_mm   r2_x_mm r2_y_mm r2_z_mm ... r6_x_mm r6_y_mm r6_z_mm
...
```

程序会按倾转角插值旋翼位置。

### 机身基础气动表

文件：

```text
Fuselage_cd.txt
Fuselage_cl.txt
Fuselage_cm.txt
Fuselage_cc.txt
Fuselage_cn.txt
Fuselage_cll.txt
```

格式：

```text
tilt_1   tilt_2   tilt_3   ...
alpha_1  coeff(alpha_1,tilt_1)  coeff(alpha_1,tilt_2)  coeff(alpha_1,tilt_3) ...
alpha_2  coeff(alpha_2,tilt_1)  coeff(alpha_2,tilt_2)  coeff(alpha_2,tilt_3) ...
...
```

对于 `cc/cn/cll` 等横航向表，系数可以表示参考侧滑角或参考偏转角下的值。使用时应保持程序配置中的定义一致。

### 舵面表

升降舵类表：

```text
deflection_deg   alpha_deg   dCD   dCL   dCM
...
```

方向舵/副翼类表：

```text
deflection_deg   alpha_deg   dCC   dCN   dCLL
...
```

## Excel 气动数据库

机身和舵面气动数据库也可以放在一个 Excel 文件中。MATLAB 可以直接读取不同 sheet。

仓库中提供了一个不含私有数据的合成模板：

```text
data_templates/aero_database_template.xlsx
```

可以把它复制到 `data/` 中，或把 `cfg.data.aero.excel_file` 指向该路径，然后用自己的数据替换其中的占位数值。

推荐 sheet：

```text
base_aero
WL1 WL2 WR1 WR2 VL1 VL2 VR1 VR2
```

基础机身/整机气动数据库支持两种 Excel 组织方式：

1. 单 sheet：所有短舱/机体倾转角都放在 `base_aero`，第一列必须是 `tilt_angle_deg`。
2. 分 sheet：每个倾转角一个 sheet，例如 `base_0`、`base_30`、`base_60`、`base_90`。这种情况下需要在 `RUN_ME.m` 中设置 `cfg.data.aero.base_sheets` 和 `cfg.data.aero.base_sheet_tilt_angle_deg`。

分 sheet 设置示例：

```matlab
cfg.switch.fuselage = "excel";
cfg.switch.controls = "excel";
cfg.data.aero.excel_file = 'aero_database.xlsx';
cfg.data.aero.base_sheets = {'base_0','base_30','base_60','base_90'};
cfg.data.aero.base_sheet_tilt_angle_deg = [0 30 60 90];
cfg.data.aero.control_surface_sheets = {'WL1','WL2','WR1','WR2','VL1','VL2','VR1','VR2'};
```

如果使用原来的 txt 表格，则设置 `cfg.switch.fuselage = "lookup"` 和 `cfg.switch.controls = "lookup"`。

`base_aero` 格式：

```text
tilt_angle_deg   beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

分 sheet 格式如下；此时倾转角由 `cfg.data.aero.base_sheet_tilt_angle_deg` 给出：

```text
beta_deg   alpha_deg   CD   CL   Cm   CC   Cn   Cl
...
```

舵面 sheet 格式：

```text
deflection_deg   alpha_deg   dCD   dCL   dCm   dCC   dCn   dCl
...
```

旧版 Excel 如果已经包含 `Mach` 列仍然可以读取，但当前机身/舵面模型不要求这一列。

Excel sheet 中可以包含文本表头和说明；MATLAB 读取时只保留完整数值行。但是数值行必须覆盖该 sheet 中所有自变量的完整插值网格。

本仓库可以包含格式示例和合成占位数值，但不包含私有气动数据。

## 开源仓库数据原则

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

用户应通过运行 `RUN_RESPONSE_SIMULINK_SETUP` 在本地生成 MEX 文件。

# VTOL BEMT Flight-Dynamics MATLAB

MATLAB implementation of a hexacopter VTOL/eVTOL trim, rotor BEMT, flapping, stability,
and control-derivative calculation.

这是一个用于六旋翼 VTOL/eVTOL 飞行动力学计算的 MATLAB 程序，包括配平、旋翼
BEMT、挥舞、稳定性导数和操纵导数计算。

Set the switches in
`RUN_ME.m` to `"default"` for built-in default models, or to `"lookup"` to read
local text (developed by users, the format will be introduced later) lookup tables from `data/`.

`RUN_ME.m` 里的 switch 可以选择 `"default"`，
使用程序内置缺省模型；也可以选择 `"lookup"`，从床且 `data/` 文件夹读取 (由用户创捷，格式说明见后文)
txt 查表数据。

## Run / 运行

Open MATLAB in this folder and run:

在 MATLAB 中打开本文件夹并运行：

```matlab
RUN_ME
```

`RUN_TRIM_AND_STABILITY.m` is a convenience wrapper that calls `RUN_ME.m`.

`RUN_TRIM_AND_STABILITY.m` 是一个便捷入口，内部直接调用 `RUN_ME.m`。

The latest result is saved to:

最新计算结果会保存到：

```text
last_run_results.mat
```

The main output variable is `trim_results`, including:

主要输出变量是 `trim_results`，包括：

```matlab
trim_results.trim_table
trim_results.stability_A
trim_results.control_B
trim_results.final_trim_var
trim_results.last_eigenvalues
trim_results.rotor_locations_m
trim_results.speed_mps
trim_results.n_speed_points
```

## Main Interface / 主接口

Edit `RUN_ME.m`.

所有主要设置都在 `RUN_ME.m` 中修改。

```matlab
data_mode = "default";  % "default" or "lookup"

cfg.switch.geometry = data_mode;
cfg.switch.rotor_positions = data_mode;
cfg.switch.chord = data_mode;
cfg.switch.pretwist = data_mode;
cfg.switch.airfoil = data_mode;
cfg.switch.fuselage = data_mode;
cfg.switch.controls = data_mode;
```

Use `"default"` to run without a `data/` folder. Use `"lookup"` when the
corresponding text files are available in `data/`.

选择 `"default"` 时不需要 `data/` 文件夹也可以运行。选择 `"lookup"` 时，程序会
从 `data/` 文件夹读取对应 txt 查表文件。

Common trim and rotor settings:

常用配平和旋翼设置：

```matlab
cfg.trim.tilt_angle_deg = 90;       % common airframe/CG/fuselage lookup angle
cfg.rotor.tilt_angle_deg = cfg.trim.tilt_angle_deg;      % scalar common nacelle tilt
% cfg.rotor.tilt_angle_deg = [90 90 90 90 85 85]; % per-rotor nacelle tilts
cfg.trim.speed_mps = [0 10 20];
cfg.trim.use_previous_solution = true;
cfg.trim.max_iterations = 10000;
cfg.stability.max_iterations = 10000;
cfg.rotor.blade_count = 5;
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
```

When `cfg.trim.use_previous_solution = true`, the first speed uses the initial
guesses in `RUN_ME.m`, and each later speed starts from the previous converged
trim vector. This is usually faster for speed sweeps.

当 `cfg.trim.use_previous_solution = true` 时，第一个速度点使用 `RUN_ME.m` 中的
初值，后续速度点会以上一个速度点的配平结果作为初值，速度扫描通常会更快。

`cfg.rotor.airfoil_section_edges` contains five nondimensional radial cutoff
locations for the six rotor airfoil sections.

`cfg.rotor.airfoil_section_edges` 是六段桨叶翼型的五个无量纲径向截止位置。

Trim initial guesses are also in `RUN_ME.m`:

配平初值也在 `RUN_ME.m` 中设置：

```matlab
cfg.trim.initial.rotor_state
cfg.trim.initial.collective_deg
cfg.trim.initial.longitudinal_deg
cfg.trim.initial.lateral_deg
cfg.trim.initial.yaw_deg
cfg.trim.initial.pitch_rad
cfg.trim.initial.roll_rad
```

`rotor_state` is repeated for all six rotors. Its length must be `2*Nb+1`:
first `Nb` flap angles, next `Nb` flap rates, and the final entry is induced
velocity.

`rotor_state` 会复制给六个旋翼。长度必须是 `2*Nb+1`：前 `Nb` 个量是挥舞角，
中间 `Nb` 个量是挥舞角速度，最后一个量是诱导速度。

## Lookup Data Folder / 查表数据文件夹

Private lookup data are not included in this repository. If lookup switches
are enabled, place text files in:

本仓库不包含私有查表数据。如果启用 `"lookup"`，请把 txt 文件放在：

```text
data/
```

or edit this line in `RUN_ME.m`:

也可以在 `RUN_ME.m` 中修改数据目录：

```matlab
data_dir = fullfile(root, 'data');
```

Expected text files:

需要的 txt 文件：

```text
CS1_cl.txt ... CS6_cl.txt
CS1_cd.txt ... CS6_cd.txt
Chord.txt
Pretwist.txt
CG_positions.txt
Rotor_positions.txt
Fuselage_cd.txt
Fuselage_cl.txt
Fuselage_cm.txt
Fuselage_cc.txt
Fuselage_cn.txt
Fuselage_cll.txt
Fuselage_elevator.txt
Fuselage_rudder.txt
Fuselage_roll.txt
```

**Important: all txt files must contain numeric data only. Do not add any
text header, column name, unit row, or comment line. The first line must already
be data.**

**重要：所有 txt 文件都只能放数值数据。不要加任何文字表头、列名、单位行或注释行。
第一行必须直接就是数据。**

For example, do not write words such as `Mach`, `alpha`, `attack angle`,
`CD`, `CL`, `tilt_angle`, or `deflection` in the file. Put the numbers directly.

例如，文件中不要写 `Mach`、`alpha`、`attack angle`、`CD`、`CL`、
`tilt_angle`、`deflection` 等文字，直接罗列数值。

All text files may be space- or tab-delimited. Angles are in degrees unless
noted otherwise.

txt 文件可以用空格或 tab 分隔。除非特别说明，所有角度单位都是度。

## Rotor Airfoil CL/CD Tables / 旋翼翼型 CL/CD 表

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

**Do not put the words `Mach`, `alpha`, `attack angle`, `CL`, or `CD` in the
file. The first row is directly the numeric Mach grid.**

**文件里不要写 `Mach`、`alpha`、`attack angle`、`CL` 或 `CD` 等文字。
第一行直接写数值 Mach 网格。**

Example:

示例：

```text
0.01    0.05    0.13    0.30
-180    0       0       0       0
-160    0.647857 0.647857 0.647857 0.647857
```

The first row is the Mach grid. Each following row starts with `alpha_deg`;
the remaining columns are coefficient values at the Mach grid points.

第一行是 Mach 网格。后续每一行的第一列是迎角 `alpha_deg`，后面的列是在各个
Mach 点上的 CL 或 CD 数值。

The section mapping is controlled by:

翼型分段由下面的变量控制：

```matlab
cfg.rotor.airfoil_section_edges = [0.25 0.40 0.50 0.80 0.92];
```

With the default values:

缺省分段为：

```text
x < 0.25  -> CS1
x < 0.40  -> CS2
x < 0.50  -> CS3
x < 0.80  -> CS4
x < 0.92  -> CS5
else      -> CS6
```

## Chord and Pretwist Tables / 弦长和负扭表

Files:

文件：

```text
Chord.txt
Pretwist.txt
```

`Chord.txt` has two numeric columns:

`Chord.txt` 有两列数值：

```text
r_over_R  chord_m
```

`Pretwist.txt` has two numeric columns:

`Pretwist.txt` 有两列数值：

```text
r_over_R  twist_deg
```

**Do not add a header row such as `r/R chord` or `r_over_R twist_deg`.
The first row must be numeric data.**

**不要加 `r/R chord` 或 `r_over_R twist_deg` 这类文字表头。第一行必须直接是数值数据。**

`r_over_R = r/R`. Chord is stored directly in metres. If the source gives
nondimensional `c/R`, multiply by rotor radius before writing `Chord.txt`.
The BEMT calculation uses `chord_m * dr` directly.

`r_over_R = r/R`。弦长直接用米作为单位。如果原始数据给的是无量纲 `c/R`，
请先乘以旋翼半径，再写入 `Chord.txt`。BEMT 中直接使用 `chord_m * dr`。

Example:

示例：

```text
0.15  0.180
0.25  0.175
0.50  0.160
1.00  0.120
```

## CG Position Table / 重心位置表

File:

文件：

```text
CG_positions.txt
```

Format:

格式：

```text
tilt_angle_deg  x_cg_mm  y_cg_mm  z_cg_mm
```

**Do not write the column names above into the txt file. They are only the
format description.**

**不要把上面这些列名写进 txt 文件；它们只是格式说明。**

The values are absolute aircraft coordinates in millimetres and are
interpolated by common airframe tilt angle.

数值是飞机坐标系下的绝对坐标，单位为毫米，并按照公共机体倾转角插值。

Example:

示例：

```text
0   3000  0  -950
30  3000  0  -950
60  3000  0  -950
90  3000  0  -950
```

## Rotor Position Table / 旋翼位置表

File:

文件：

```text
Rotor_positions.txt
```

Format:

格式：

```text
rotor_id  tilt_angle_deg  x_mm  y_mm  z_mm
```

**Do not write `rotor_id tilt_angle x y z` or any other text header into the
file. The first row must be numeric.**

**不要在文件里写 `rotor_id tilt_angle x y z` 或任何其他文字表头。第一行必须是数值。**

There must be one row for each rotor at each available tilt angle. The
coordinates are absolute aircraft coordinates in millimetres. The code converts
them to CG-relative body coordinates internally before calling the rotor BEMT
routine.

每一个可用倾转角下，每个旋翼都需要有一行数据。坐标为飞机坐标系下的绝对坐标，
单位为毫米。程序内部会转换为相对重心的机体系坐标，再传入旋翼 BEMT。

`cfg.rotor.tilt_angle_deg` may be scalar or a six-element vector. A scalar is
applied to all rotors; a vector gives individual nacelle tilt angles.

`cfg.rotor.tilt_angle_deg` 可以是一个标量，也可以是六个元素的向量。标量表示
六个旋翼共用同一个短舱倾转角；向量表示每个旋翼单独给定短舱倾转角。

Example:

示例：

```text
1  90  3000   5000  -1900
2  90  1500   2500  -1800
3  90  1500  -2500  -1800
4  90  3000  -5000  -1900
5  90  5000   2500  -1000
6  90  5000  -2500  -1000
1  60  2950   5000  -1850
2  60  1450   2500  -1750
3  60  1450  -2500  -1750
4  60  2950  -5000  -1850
5  60  5050   2500  -1050
6  60  5050  -2500  -1050
```

## Fuselage/Airframe Coefficient Tables / 机身及固定翼气动系数表

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
tilt_angle_deg  alpha_deg  coefficient
```

**Do not write `tilt_angle`, `alpha`, `CD`, `CL`, `CM`, `CC`, `CN`, or `CLL`
as a header. The file must start directly with numeric data.**

**不要把 `tilt_angle`、`alpha`、`CD`、`CL`、`CM`、`CC`、`CN` 或 `CLL`
写成表头。文件第一行必须直接是数值数据。**

The file name determines the coefficient. For example, `Fuselage_cd.txt`
supplies CD and `Fuselage_cm.txt` supplies CM.

系数类型由文件名决定。例如，`Fuselage_cd.txt` 对应 CD，`Fuselage_cm.txt` 对应 CM。

Example:

示例：

```text
0   -10  0.120
0     0  0.080
0    10  0.130
30  -10  0.150
30    0  0.100
30   10  0.160
```

## Control Surface Increment Tables / 舵面增量表

Files:

文件：

```text
Fuselage_elevator.txt
Fuselage_rudder.txt
Fuselage_roll.txt
```

Format:

格式：

```text
deflection_deg  alpha_deg  increment_1  increment_2  increment_3
```

**Do not write `deflection`, `alpha`, `dCD`, `dCL`, `dCM`, `dCC`, `dCN`,
or `dCLL` as a header. The first row must be numeric data.**

**不要把 `deflection`、`alpha`、`dCD`、`dCL`、`dCM`、`dCC`、`dCN` 或
`dCLL` 写成表头。第一行必须直接是数值数据。**

For `Fuselage_elevator.txt`, the increments are:

对于 `Fuselage_elevator.txt`，后三列依次为：

```text
dCD  dCL  dCM
```

For `Fuselage_rudder.txt` and `Fuselage_roll.txt`, the increments are:

对于 `Fuselage_rudder.txt` 和 `Fuselage_roll.txt`，后三列依次为：

```text
dCC  dCN  dCLL
```

Example:

示例：

```text
-10  -10  0.002  -0.050   0.010
-10    0  0.001  -0.045   0.008
  0  -10  0.000   0.000   0.000
  0    0  0.000   0.000   0.000
 10  -10  0.002   0.050  -0.010
 10    0  0.001   0.045  -0.008
```

The lookup call returns the three increments in the order above.

程序会按照上述顺序返回三个舵面增量。

## Repository Layout / 仓库结构

```text
.
|-- RUN_ME.m
|-- RUN_TRIM_AND_STABILITY.m
|-- README.md
`-- src/
    |-- BEMTFLAP_SWITCHED.m
    |-- CS1_cd_lookup.m
    |-- build_1d_lookup_from_txt.m
    |-- build_cg_lookup_from_txt.m
    |-- build_rotor_position_lookup_from_txt.m
    |-- build_C_lookup_from_txt.m
    `-- build_fuselage_elevator_lookup.m
```

## License / 许可证

MIT License.

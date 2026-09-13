# results

三个 study 运行后会在这里产出 CSV，装了 matplotlib 时还会产出 PNG。

- `quasi_static.py`：`A_force_displacement.csv`、`B_force_displacement.csv`、
  `C_force_displacement.csv`、`boundary_sensitivity.csv`。
- `transient.py`：`transient_A.csv`、`transient_B.csv`、`transient_C.csv`、
  `transient_summary.csv`、`transient_velocity.png`。
- `parameter_study.py`：`sweep_boundary.csv`、`sweep_preload.csv`、
  `sweep_mismatch.csv`、`sweep_membrane.csv`、`sweep_damping.csv`。

CSV 一律是 UTF-8、逗号分隔、首行表头，长度单位 m，时间 s，力 N，质量 kg。
同名文件会被覆盖，重新跑 study 即刷新。这些是模型输出，不是实测数据；与实验
对照前不要直接引用绝对值。

快速重跑：

```bash
python studies/quasi_static.py
python studies/transient.py
python studies/parameter_study.py
python -m pytest -q
```

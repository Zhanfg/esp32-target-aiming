# 09_PLOTS

放从 `04_SIMULATION/results/` 的仿真 CSV 画出来的图，以及生成脚本。数据只读，不改动仿真目录里的任何文件。

实测数据的图以后从 `08_DATA/` 来，到时另加。引用某张图时用相对路径，不要把图复制到别处各自维护。

## 重新生成

```bash
pip install matplotlib
python make_plots.py
```

脚本按自身文件位置定位 `04_SIMULATION/results/`，在哪个工作目录运行都一样。中文字体优先用
Microsoft YaHei，其次 SimHei。matplotlib 缺失时脚本不报错退出，改为写出 `PLOT_SPECS.md`，
逐张列出数据来源、坐标轴、单位与要标注的点，供有 matplotlib 的机器照着重画。

## 图清单

| 文件 | 内容 | 对应结论 |
|---|---|---|
| fig01_force_displacement.png | A / B / C 力—位移曲线，标 B、C 的 snap 阈值与 C 的极限点 | STOP-01 单级对照，STOP-02 双级静态阈值与作动行程 |
| fig02_transient.png | A / B / C 膜片位移与速度时间历程，标峰值、到达时间与 snap 时刻 | STOP-01 / STOP-02 动态响应 |
| fig03_boundary_threshold.png | snap 阈值随 k_base 变化，右图给 B 与 C 的相对差 | §23.3 边界刚度 |
| fig04_sweep_mismatch.png | 峰值速度、级间间隔随刚度比变化，标出级序翻转 | §23.3 级间失配 |
| fig05_sweep_membrane.png | 峰值速度随膜片刚度变化 | §23.3 膜片柔度 |
| fig06_sweep_damping.png | 峰值速度与尾部衰减随阻尼变化 | §23.3 阻尼 |
| fig07_sweep_preload.png | 峰值速度随预载变化 | §23.3 预载 |

## 读图要点

- fig01：B 与 C 的 snap 阈值几乎相同，都在 1.713 N。C 需要的作动行程约 9.9 mm，比 B 的
  6.6 mm 多一个双稳单元的量。静态扫描在第一个极限点停止，图上 C 的极限点就是两级串联的
  共同极限，此处一级与二级压缩量相同，各自距单级 fold 只差 0.008 mm。
- fig02：膜片峰值速度 A 为 0.47 m/s，B 为 8.94 m/s，C 为 10.57 m/s。C 的两级 snap 分别在
  12.54 ms 与 19.75 ms，间隔 7.21 ms，二级先于一级。
- fig03：`boundary_sensitivity.csv` 里没有 A 的阈值，普通膜片没有 snap，所以三条线是 VMT
  单元固有 snap 力、B 阈值、C 阈值。B 与 C 的差在 3.6 ppm 以内，可以当成重合。
- fig05：峰值速度对柔度非单调。刚度放大 0.1 倍处有个低点 7.00 m/s，放大 3 倍处最高
  9.05 m/s，放大 100 倍降到 3.30 m/s。

## 当前环境的出图情况

本机已装 matplotlib 3.11.2，上面七张 PNG 都由 `python make_plots.py` 实际生成，宽度
2147 到 2312 px，满足缩小后阅读与投稿排版的尺寸要求。

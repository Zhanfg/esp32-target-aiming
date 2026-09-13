# data/

## sample_track.csv

合成的目标航迹，供 PC 端验证求解器和回归测试使用。文件由脚本生成，手改后会与生成脚本不一致，
不要直接编辑。

表头 `t_ms,px,py,confidence`，120 行数据，不含表头。

- 采样：20 ms 一帧，`t_ms = i * 20`，全长约 2.4 s
- 运动：从像素 (200, 240) 出发，以 (80, 0) px/s 匀速直线运动
- 噪声：像素高斯噪声标准差 0.3 px
- 遮挡：帧下标 [60, 75) 共 15 帧，`confidence = 0`，坐标仍沿真值给出
- 随机种子：20240913

## 重新生成

在 05_FIRMWARE/algo_reference/ 目录下执行：

```bash
python data/make_sample_track.py
```

脚本固定 seed，重跑结果逐字节一致。生成参数在 `data/make_sample_track.py` 顶部的常量里，
需要别的场景时复制脚本另存。

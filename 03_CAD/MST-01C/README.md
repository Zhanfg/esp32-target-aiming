# MST-01C 双级串联 snap

## 目的

§13 的 C 级。两个 von Mises truss 单元串联后再推膜片，用来判断 sequential snap 是否改善
输出时间历程或重复性。C 与 B 的对照是 §5.3 的核心问题。

## 模块与文件

- `build.py`：装配 FRAME、RELEASE_MODULE、STAGE_1、STAGE_GAP、STAGE_2、OUTPUT、
  DIAPHRAGM_MODULE 并自检。
- FRAME、DIAPHRAGM_MODULE、RELEASE_MODULE：与 B 级同一套。
- STAGE_1、STAGE_2：两个失稳单元串联，弹性片都是独立可换件。
- STAGE_GAP：级间垫片，垫片厚度就是 engagement gap 的粗调。
- OUTPUT：第二级顶点到膜片的传力杆。

## 级间参数扫描

§23.3 要求优先扫边界刚度、预载、级间失配、膜片刚度、机械阻尼。C 级留了两个几何旋钮：

- engagement gap：换 `STAGE_GAP` 垫片，或在 `build.py` 改 `ENGAGEMENT_GAP`，
  改变第二级开始受力的 Z 向位置。
- 级间刚度失配：把 `STAGE1_LEAF` 与 `STAGE2_LEAF` 设成不同材料或不同厚度，
  例如一级 PET、二级弹簧钢，得到两级不同的 snap 阈值。

这两个旋钮都只改一个独立件或一个变量，不影响其余模块，符合 §23.3 不盲目扫所有几何的要求。

## 关键尺寸与来源

失稳单元几何与 B 级相同：a=8 mm、L=10 mm、h0=6 mm，对齐仿真。两级底边分别放在 Z=15 mm
与 Z=27 mm，级间间隙由垫片填到需要的 engagement gap，默认 2 mm，待实测标定。

| 尺寸 | 取值 | 来源 |
|---|---|---|
| 两级底边间距 | 12 mm | 布局取值，容纳拱高 6 mm、顶点滑块 4 mm 与垫片 |
| engagement gap | 2 mm | 工程估计，§23.3 扫描变量，待实测标定 |
| 轴向长度 | 44 mm | §1.2 长度 30–50 mm 内 |

## 释放机构

与 B 级同一套自锁卡榫加电磁铁拔销。自锁升角 6°、摩擦角 8.53°、保持比 1.43，
断电不释放，细节见 `lib/release.py` 与 B 级 README。

## 装配顺序

1. 框架同 B 级装好。
2. 装 STAGE_1，底边 Z=15 mm，夹块压片，插铰销，装顶点滑块。
3. 放 STAGE_GAP 垫片。
4. 装 STAGE_2，底边 Z=27 mm，重复夹片与滑块步骤。
5. 第二级顶点接 OUTPUT 传力杆到膜片。
6. 装 RELEASE_MODULE，卡榫扣住第一级滑块后台阶。
7. 装膜片压环。手动确认两级都能各自 snap。

## 可拆可测性

两级弹性片都能单独抽出更换，级间垫片随手可换，压环可换膜。顶部敞开，两级 snap 的先后
与顶点运动都能拍到，便于 §23.1 判定 Stage 1 / Stage 2 顺序。轴向 44 mm、横向 22 mm，
落在 §1.2 envelope 内，已接近长度上限，再往上加级会超 envelope，这也是双级到此为止的原因。

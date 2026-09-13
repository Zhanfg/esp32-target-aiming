# 03_CAD

结构模型按规格书 §13 的五级原型分开，一级一个子目录：

- `MST-01A/`：普通膜片基线，无 snap-through，低速执行器直接推膜片。
- `MST-01B/`：单级 snap，一个失稳单元。
- `MST-01C/`：双级串联 snap，两个失稳单元，要能判定两级先后。
- `MST-01D/`：可切换边界研究版，locked / released support 或等效的边界改变，带多种预载状态。
- `MST-01E/`：自动供给完整版，6–8 位供给盘、索引机构、Mode B 相关结构。

§14 定了 A → B → C → D → E 的次序，不许跳级，所以目录编号也按这个顺序推进，后续级别在上一级
达标前不动。

设计原则见 §22：模块化（§22.1 要求至少拆成可独立更换的几块）、可拆可测（§22.2，传感靶这类装在运动件
上的附件要能单独拆下做对照）、关键件一体化（§22.3）。§5 的 Stage 1 / Stage 2 两级机械定义和 §6 的
膜片安装方式都会落到具体零件上，失稳单元的候选结构见 §7。

文件名带版本，改了结构就另存一版，别覆盖上一版，方便回查某次实验用的是哪套模型。

## 三级原型的关系

本轮先做 A/B/C 三级，结构沿同一条轴向串联，差别只在中间放什么：

| 级别 | 中间级 | 失稳单元 | 释放卡榫 | 轴向长度 |
|---|---|---|---|---|
| A | 低速执行器直推 | 无 | 无 | 32 mm |
| B | 单个 von Mises truss | 1 个 | 有 | 38 mm |
| C | 两个 von Mises truss 串联 | 2 个 | 有 | 44 mm |

A 没有储能元件，按 §13 是直接推动的基线，不需要释放卡榫。B 与 C 用同一套自锁卡榫加电磁铁拔销的
释放机构。三级共用 FRAME、DIAPHRAGM_MODULE，C 额外多一个 STAGE_2。

## 参数库怎么用

`lib/` 是纯 Python，算尺寸、体积、包围盒、质量都不需要 FreeCAD：

- `lib/params.py`：所有结构尺寸、材料密度、布局锚点、参数来源登记表 `PARAM_SOURCES`。
- `lib/frame.py`：FRAME 模块几何。
- `lib/stage.py`：STAGE_1 / STAGE_2 的 von Mises truss 单元，弹性片是独立件。
- `lib/membrane.py`：DIAPHRAGM_MODULE，膜片加前后两片可拆压环。
- `lib/release.py`：RELEASE_MODULE，自锁卡榫与电磁铁拔销，含自锁余量计算。
- `lib/checks.py`：件数、包围盒、体积、质量、envelope 对照。
- `lib/freecad_export.py`：把 Solid 列表转 FreeCAD 实体并导出 STEP / FCStd，best-effort。

改尺寸先改 `params.py`，各级 `build.py` 会跟着变。`params.layout(level)` 给出该级沿 Z 的模块区间。

## 重建命令

在 `03_CAD/` 目录下：

```powershell
python MST-01A/build.py
python MST-01B/build.py
python MST-01C/build.py
```

每跑一次打印一份几何自检：件数、包围盒、体积、按材料质量估算、模块件数与 envelope 判定。
追加 `--freecad` 会尝试导出 STEP 与 FCStd，当前环境没装 FreeCAD 时会打印跳过提示，不影响自检。

## 与 §22 三条原则的对应

- §22.1 模块化：C 级可拆成 FRAME、STAGE_1、STAGE_2、DIAPHRAGM_MODULE、RELEASE_MODULE 五块，
  外加 OUTPUT 传力杆；B 级去掉 STAGE_2；A 级把中间级换成 ACTUATOR。
- §22.2 可测试性：压环用 M2 螺钉夹紧，松开即可换膜片；失稳单元弹性片由夹块与铰销定位，
  松开夹块螺钉可整片抽出更换；框架顶部敞开，从上方能拍到 snap 过程；自锁卡榫与电磁铁露在外侧。
- §22.3 不要过早一体化：打印件只做框架、夹块、滑块、压环、卡榫座与电磁铁座，弹性件与销都是
  独立标准件或片材，第一版优先看得见、拆得开、测得到。

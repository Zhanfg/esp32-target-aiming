"""MST-01 A/B/C/D/E CAD 参数化库。

- params         参数集中处、几何原语、布局、来源登记
- frame          FRAME 模块
- stage          STAGE_1 / STAGE_2 失稳单元
- membrane       DIAPHRAGM_MODULE 膜片与可拆压环
- release        RELEASE_MODULE 自锁卡榫与电磁铁拔销
- boundary       BOUNDARY_MODULE 可切换边界（D 级）
- preload        PRELOAD_MODULE 多种预载状态（D 级）
- magazine       MAGAZINE_MODULE 6/8 位供给盘与索引（E 级）
- checks         几何自检
- freecad_export FreeCAD 导出（best-effort）
- stl            纯 Python STL 写出与读回
"""

from . import (boundary, checks, frame, freecad_export, magazine, membrane,
               params, preload, release, stage, stl)

__all__ = ["params", "frame", "stage", "membrane", "release", "boundary",
           "preload", "magazine", "checks", "freecad_export", "stl"]

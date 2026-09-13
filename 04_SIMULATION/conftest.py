"""pytest 路径引导：让 tests 与 studies 都能 import models。"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

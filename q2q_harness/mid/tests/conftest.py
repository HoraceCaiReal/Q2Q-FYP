"""让 pytest 能直接 import q2q_orchestration（包在 mid/q2q_orchestration/q2q_orchestration）。

这些测试**不启动 ROS**：machine.py / ready_gate.py 都是纯 Python，
事件表能离线跑，改逻辑之前先改这里。
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'q2q_orchestration'))

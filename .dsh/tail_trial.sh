RUN=$(python3 -c 'import json,pathlib;print(json.loads((pathlib.Path.home()/".q2q/runs/step3_control.json").read_text())["run_dir"])')
echo "RUN=$RUN"
echo "=== console tail ==="
CONSOLE=$(find "$RUN" -maxdepth 1 -name 'step7_console*.log' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)
tail -20 "$CONSOLE" 2>/dev/null || echo "(no console log yet)"
echo "=== orch log tail ==="
ORCH=$(find "$RUN" -maxdepth 2 -name orch.log -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)
tail -8 "$ORCH" 2>/dev/null
echo "=== clock ==="
source /home/ros/.q2q/step3_env.sh 2>/dev/null
timeout 8 ros2 topic echo /clock --once 2>/dev/null | head -4

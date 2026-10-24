#!/usr/bin/env bash
set -eo pipefail
source /home/ros/.q2q/step3_env.sh
RUN=$(python3 -c 'import json,pathlib;print(json.loads((pathlib.Path.home()/".q2q/runs/step3_control.json").read_text())["run_dir"])')
LOG="$RUN/step7_console_$(date +%H%M%S).log"
nohup python3 /home/ros/.q2q/step7_trial.py "$@" > "$LOG" 2>&1 < /dev/null &
echo "LAUNCHED pid=$! log=$LOG (launch is not an acceptance PASS)"

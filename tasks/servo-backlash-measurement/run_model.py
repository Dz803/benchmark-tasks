"""One official Dream trial using the published task instruction."""
import json
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parent
run = root / 'runs/model'
run.mkdir(parents=True, exist_ok=False)
instruction = json.loads((root/'task.json').read_text())['instruction']
command = ['inspect-robots','run','--embodiment','yam_arena','--policy','dream',
 '--policy-checkpoint','dreamlabs/dream-1-beta','-P','timeout_s=90',
 '-E',f'task={root / "task.py"}','-E','object_jitter=0','-E','control_hz=20',
 '-E','cam_width=320','-E','cam_height=176','-E','top_cam=demo',
 '--instruction',instruction,'--scorer','task_score','--max-steps','2400',
 '--store-frames','--seed','42','--no-prompt','--grader','none','--no-rerun',
 '--log-dir',str(run)]
with (run/'console.log').open('w') as log:
 result = subprocess.run(command,cwd=root,stdout=log,stderr=subprocess.STDOUT)
raise SystemExit(result.returncode)

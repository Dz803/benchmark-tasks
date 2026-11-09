"""Run the complete initial-state physical demonstration."""
from pathlib import Path
import os
import sys
from powered_demo import run
if __name__=='__main__':
    if os.environ.get('REPAIR_CHECKPOINT'):
        raise RuntimeError('The complete demo entry point does not accept diagnostic checkpoints.')
    run(Path(sys.argv[1]))

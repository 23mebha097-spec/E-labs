#!/usr/bin/env python
"""Debug runner - captures full error output to debug.log"""

import sys
import traceback

# Redirect stderr to file for debugging
debug_log = open('debug.log', 'w')
sys.stderr = debug_log

try:
    print("Starting E-Labs application...", file=debug_log, flush=True)
    from main import main
    main()
except Exception as e:
    print(f"\n\n{'='*60}", file=debug_log, flush=True)
    print(f"EXCEPTION CAUGHT IN DEBUG RUNNER:", file=debug_log, flush=True)
    print(f"{'='*60}\n", file=debug_log, flush=True)
    traceback.print_exc(file=debug_log)
    print(f"\n{'='*60}\n", file=debug_log, flush=True)
finally:
    debug_log.close()

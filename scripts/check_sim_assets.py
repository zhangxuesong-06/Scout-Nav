#!/usr/bin/env python3
"""Load each migrated world through Fortress physics for five iterations."""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile

from scout_sim.scene import catalog, prepare_scene


def main():
    root = Path(__file__).resolve().parents[1]
    share = root / 'src/scout_sim'
    logs = root / 'log/simulation/world_checks'
    logs.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, IGN_GAZEBO_RESOURCE_PATH=str(share / 'models'),
               SDF_PATH=str(share / 'models'), IGN_PARTITION='scout_world_asset_check')
    results = {}
    for key in catalog(share):
        with tempfile.TemporaryDirectory(prefix='scout_asset_check_') as temp:
            path, _, _ = prepare_scene(share, temp, world=key, camera=False)
            logfile = logs / (key.replace('/', '_') + '.log')
            with logfile.open('w') as output:
                process = subprocess.Popen(['ign', 'gazebo', '-s', '-r', '-v', '3',
                                            '--iterations', '5', str(path)],
                                           env=env, stdout=output, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                try:
                    code = process.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    code = 'timeout'
                finally:
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=5)
            results[key] = {'exit_code': code, 'log': str(logfile)}
            print(key, code, flush=True)
    (logs / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    if any(r['exit_code'] != 0 for r in results.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()

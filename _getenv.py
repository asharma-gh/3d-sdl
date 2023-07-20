import sys
import subprocess as sp
import json
res = sp.run(['micromamba', 'info', '--json'], stdout=sp.PIPE)

if res:
    js = json.loads(res.stdout)
    # Print include dir
    print(js["env location"])
    sys.exit(0)
sys.exit(1)

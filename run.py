import os
import sys

os.environ["OMP_CANCELLATION"] = 'TRUE'
os.environ["OMP_NESTED"] = 'TRUE'
cmd = "./Debug/nes_emulator " + sys.argv[1]
os.system(cmd)
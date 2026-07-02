#!/usr/bin/env python3
"""
Recompile openpilot's driving model for the AMD GPU (tinygrad AMD backend).

openpilot's build only considers CUDA/QCOM/CPU, so on this machine the model
runs as LLVM CPU code and cannot hold 20 Hz while BeamNG is running. This
script re-runs openpilot's own compile step (mirroring
selfdrive/modeld/SConscript) with DEV=AMD and updates the runtime device
json. Everything it touches is a scons BUILD ARTIFACT — openpilot source is
untouched, per project policy.

Revert any time:  cd ~/openpilot && tools/op.sh build

Run inside the distrobox:
    python3 tools/build_model_gpu.py            # DEV=AMD
    MODEL_DEV=CPU:LLVM python3 tools/build_model_gpu.py   # back to CPU without scons
"""
import json
import os
import subprocess
import sys

OP = os.path.expanduser(os.environ.get('OPENPILOT_DIR', '~/openpilot'))
sys.path.insert(0, OP)

from openpilot.common.file_chunker import chunk_file, get_chunk_targets, get_existing_chunks   # noqa: E402
from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye                # noqa: E402
from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE                         # noqa: E402
from openpilot.selfdrive.modeld.constants import ModelConstants                                # noqa: E402
from openpilot.selfdrive.modeld.helpers import TG_INPUT_DEVICES_PATH, modeld_pkl_path          # noqa: E402

DEV = os.environ.get('MODEL_DEV', 'AMD')
# JIT_BATCH_SIZE=0 + FLOAT16=1 mirror comma's own AMD (usbgpu) tinygrad flags
EXTRA = os.environ.get('MODEL_EXTRA_FLAGS', 'FLOAT16=1 JIT_BATCH_SIZE=0' if DEV == 'AMD' else '')

modeld_dir = os.path.join(OP, 'openpilot/selfdrive/modeld')
onnx = os.path.join(modeld_dir, 'models/driving_supercombo.onnx')
pkl = str(modeld_pkl_path(usbgpu=False))
model_w, model_h = MEDMODEL_INPUT_SIZE
frame_skip = ModelConstants.MODEL_RUN_FREQ // ModelConstants.MODEL_CONTEXT_FREQ
cam_res = [f'{_ar_ox_fisheye.width}x{_ar_ox_fisheye.height}',
           f'{_os_fisheye.width}x{_os_fisheye.height}']

env = os.environ.copy()
env['PYTHONPATH'] = OP
env['DEV'] = DEV
for kv in EXTRA.split():
    k, v = kv.split('=', 1)
    env[k] = v

cmd = [sys.executable, f'{modeld_dir}/compile_modeld.py',
       '--model-size', f'{model_w}x{model_h}',
       '--camera-resolutions', *cam_res,
       '--onnx', onnx,
       '--output', pkl,
       '--frame-skip', str(frame_skip)]
print(f'[build_model_gpu] DEV={DEV} {EXTRA}\n[build_model_gpu] {" ".join(cmd)}', flush=True)
subprocess.run(cmd, env=env, check=True)

# modeld loads the CHUNKED pkl (open_file_chunked) — rechunk like SConscript does
onnx_size = sum(os.path.getsize(f) for f in get_existing_chunks(onnx))
targets = get_chunk_targets(pkl, 1.2 * onnx_size + 10 * 1024 * 1024)
chunk_file(pkl, targets)
print(f'[build_model_gpu] chunked into {len(targets)} chunk(s)', flush=True)

# point modeld's runtime input devices at the same backend.
# NB: DEV can be a compile spec like "CPU:LLVM"; the runtime Device[] name is
# just the backend ("CPU") — scons writes tg_backend here, not tg_flags.
runtime_dev = DEV.split(':')[0]
with open(TG_INPUT_DEVICES_PATH) as f:
    devices = json.load(f)
devices['openpilot.selfdrive.modeld.modeld']['default'] = {'WARP_DEV': runtime_dev, 'QUEUE_DEV': runtime_dev}
with open(TG_INPUT_DEVICES_PATH, 'w') as f:
    json.dump(devices, f)
    f.write('\n')
print(f'[build_model_gpu] tg_input_devices.json → modeld on {DEV}', flush=True)
print('[build_model_gpu] DONE — revert with: cd ~/openpilot && tools/op.sh build', flush=True)

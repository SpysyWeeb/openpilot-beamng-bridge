#!/usr/bin/env python3
"""
Model-output visualizer — draw what modeld is thinking onto the camera frame.

Projects modelV2's planned path, lane lines, road edges and lead onto the
current road-camera frame using openpilot's own intrinsics, exactly like the
onroad UI does. One image answers: is the model planning, and does its
geometry line up with the road pixels (i.e. is the camera mount/calibration
coherent)?

Run inside the distrobox while the stack is up:
    python3 tools/model_viz.py [--out /tmp/model_viz.png]

Model frame is [x fwd, y left, z up] relative to the calibrated camera;
device frame is [fwd, right, down], so device = (x, -y, -z).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.expanduser(os.environ.get('OPENPILOT_DIR', '~/openpilot')))

import numpy as np
from PIL import Image, ImageDraw

import openpilot.cereal.messaging as messaging
from msgq.visionipc import VisionIpcClient, VisionStreamType
from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame

W, H = 1928, 1208


def project(K, pts_model):
    """Model-frame Nx3 -> Nx2 pixels (NaN when behind camera)."""
    pts = np.asarray(pts_model, dtype=float)
    dev = np.stack([pts[:, 0], -pts[:, 1], -pts[:, 2]], axis=1)   # [F,L,U] -> [F,R,D]
    view = dev @ view_frame_from_device_frame.T                    # -> [R,D,F]
    with np.errstate(divide='ignore', invalid='ignore'):
        uv = (view[:, :2] / view[:, 2:3])
    px = uv @ K[:2, :2].T + K[:2, 2]
    px[view[:, 2] < 0.5] = np.nan                                  # behind / too close
    return px


def draw_line(d, px, color, width=4):
    pts = [(float(u), float(v)) for u, v in px if np.isfinite(u) and np.isfinite(v)]
    if len(pts) >= 2:
        d.line(pts, fill=color, width=width)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/model_viz.png')
    args = ap.parse_args()

    sm = messaging.SubMaster(['modelV2', 'carState', 'radarState'])
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        sm.update(100)
        if sm.updated['modelV2']:
            break
    else:
        raise SystemExit('no modelV2 — is the stack up?')
    m = sm['modelV2']

    cli = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
    while not cli.connect(False):
        time.sleep(0.05)
    buf = None
    for _ in range(100):
        buf = cli.recv(100)
        if buf is not None:
            break
    y = np.frombuffer(bytes(buf.data)[:W * H], dtype=np.uint8).reshape(H, W)
    img = Image.fromarray(y, 'L').convert('RGB')
    d = ImageDraw.Draw(img)

    K = DEVICE_CAMERAS[('pc', 'unknown')].fcam.intrinsics

    def xyz(traj):
        return np.stack([np.array(traj.x), np.array(traj.y), np.array(traj.z)], axis=1)

    # road edges (red), lane lines (yellow), path (green, thick)
    for edge in m.roadEdges:
        draw_line(d, project(K, xyz(edge)), (255, 60, 60), 4)
    for i, ll in enumerate(m.laneLines):
        draw_line(d, project(K, xyz(ll)), (255, 220, 40), 3)
    draw_line(d, project(K, xyz(m.position)), (60, 255, 90), 8)

    lead = sm['radarState'].leadOne
    if lead.status:
        px = project(K, [[float(lead.dRel), float(-lead.yRel), 0.0]])[0]
        if np.isfinite(px).all():
            u, v = px
            d.rectangle([u - 25, v - 25, u + 25, v + 25], outline=(80, 160, 255), width=4)

    img.save(args.out)

    pos = xyz(m.position)
    lls = [round(float(np.array(ll.y)[0]), 2) for ll in m.laneLines]
    print(f'saved {args.out}')
    print(f'vEgo={sm["carState"].vEgo:.1f}  path reach={pos[-1,0]:.1f}m  '
          f'path z[0]={pos[0,2]:+.2f} z[-1]={pos[-1,2]:+.2f}')
    print(f'laneLines y@0m={lls}  (expect ~[+3.x,+1.x,-1.x,-3.x] if y is left-positive)')
    print(f'laneLineProbs={[round(p,2) for p in m.laneLineProbs]}  '
          f'roadEdge stds={[round(float(s),2) for s in m.roadEdgeStds]}')


if __name__ == '__main__':
    main()

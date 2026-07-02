#!/usr/bin/env python3
"""
Model-output visualizer — see what modeld is thinking, live.

Projects modelV2's planned path, lane lines, road edges and lead onto the
current road-camera frame (openpilot's own intrinsics) plus a top-down
panel. openpilot's stock UI draws the model's `position` head, which
collapses to a short stub on ALL synthetic imagery (MetaDrive included) —
so in sim this window, not the stock UI, is the truth about the model.

Single shot:   python3 tools/model_viz.py [--out /tmp/model_viz.png]
Live window:   python3 tools/model_viz.py --live      (run from a terminal
               in your desktop session so the window can appear)

Conventions (verified empirically): model frame = [x fwd, y right, z up
from road plane]; camera CAM_HEIGHT above that plane; device = (x, y, h-z).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.expanduser(os.environ.get('OPENPILOT_DIR', '~/openpilot')))

import numpy as np
from PIL import Image, ImageDraw

# Layout compat: root-layout trees (release branches) have cereal at the repo
# root and no openpilot.cereal — alias it so both layouts work.
try:
    import openpilot.cereal.messaging  # noqa: F401
except ModuleNotFoundError:
    import cereal as _cereal
    import openpilot as _openpilot
    sys.modules['openpilot.cereal'] = _cereal
    _openpilot.cereal = _cereal

import openpilot.cereal.messaging as messaging
from msgq.visionipc import VisionIpcClient, VisionStreamType
from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame

W, H = 1928, 1208
CAM_HEIGHT = 1.22   # the model's assumed camera height above the road plane
TD_W, SCALE = 420, 14.0  # top-down: px width, px per meter


def project(K, pts_model):
    pts = np.asarray(pts_model, dtype=float)
    dev = np.stack([pts[:, 0], pts[:, 1], CAM_HEIGHT - pts[:, 2]], axis=1)
    view = dev @ view_frame_from_device_frame.T
    with np.errstate(divide='ignore', invalid='ignore'):
        uv = view[:, :2] / view[:, 2:3]
    px = uv @ K[:2, :2].T + K[:2, 2]
    px[view[:, 2] < 0.5] = np.nan
    return px


def draw_line(d, px, color, width=4):
    pts = [(float(u), float(v)) for u, v in px if np.isfinite(u) and np.isfinite(v)]
    if len(pts) >= 2:
        d.line(pts, fill=color, width=width)


def xyz(traj):
    return np.stack([np.array(traj.x), np.array(traj.y), np.array(traj.z)], axis=1)


def render(sm, cli, K):
    """One combined frame (camera overlay + top-down) as a PIL Image."""
    buf = None
    for _ in range(50):
        buf = cli.recv(50)
        if buf is not None:
            break
    if buf is None:
        return None
    y = np.frombuffer(bytes(buf.data)[:W * H], dtype=np.uint8).reshape(H, W)
    img = Image.fromarray(y, 'L').convert('RGB')
    d = ImageDraw.Draw(img)

    m = sm['modelV2']
    for edge in m.roadEdges:
        draw_line(d, project(K, xyz(edge)), (255, 60, 60), 4)
    for ll in m.laneLines:
        draw_line(d, project(K, xyz(ll)), (255, 220, 40), 3)
    draw_line(d, project(K, xyz(m.position)), (60, 255, 90), 8)

    lead = sm['radarState'].leadOne
    if lead.status:
        px = project(K, [[float(lead.dRel), float(lead.yRel), 0.0]])[0]
        if np.isfinite(px).all():
            u, v = px
            d.rectangle([u - 25, v - 25, u + 25, v + 25], outline=(80, 160, 255), width=4)

    td = Image.new('RGB', (TD_W, H), (18, 18, 28))
    dtd = ImageDraw.Draw(td)

    def td_px(pts):
        pts = np.asarray(pts, dtype=float)
        u = TD_W / 2 + pts[:, 1] * SCALE
        v = H - 20 - pts[:, 0] * SCALE
        return list(zip(u.tolist(), v.tolist()))

    for gx in range(0, 81, 10):
        vv = H - 20 - gx * SCALE
        dtd.line([(0, vv), (TD_W, vv)], fill=(45, 45, 60), width=1)
        dtd.text((4, vv - 12), f'{gx}m', fill=(110, 110, 130))
    for edge in m.roadEdges:
        dtd.line(td_px(xyz(edge)), fill=(255, 60, 60), width=3)
    for ll in m.laneLines:
        dtd.line(td_px(xyz(ll)), fill=(255, 220, 40), width=2)
    dtd.line(td_px(xyz(m.position)), fill=(60, 255, 90), width=5)
    dtd.polygon([(TD_W / 2 - 8, H - 8), (TD_W / 2 + 8, H - 8), (TD_W / 2, H - 30)],
                fill=(200, 200, 255))
    vego = sm['carState'].vEgo
    curv = float(m.action.desiredCurvature)
    dtd.text((10, 8), f'vEgo {vego * 2.237:5.1f} mph', fill=(220, 220, 240))
    dtd.text((10, 24), f'desiredCurv {curv:+.4f}', fill=(60, 255, 90))
    dtd.text((10, 40), f'reach {list(m.position.x)[-1]:.0f} m (aux head)', fill=(160, 160, 180))

    combined = Image.new('RGB', (img.width + TD_W, H))
    combined.paste(img, (0, 0))
    combined.paste(td, (img.width, 0))
    return combined


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/model_viz.png')
    ap.add_argument('--live', action='store_true', help='show a live window (~8 Hz)')
    args = ap.parse_args()

    sm = messaging.SubMaster(['modelV2', 'carState', 'radarState'])
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        sm.update(100)
        if sm.updated['modelV2']:
            break
    else:
        raise SystemExit('no modelV2 — is the stack up?')

    cli = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_ROAD, True)
    while not cli.connect(False):
        time.sleep(0.05)
    K = DEVICE_CAMERAS[('pc', 'unknown')].fcam.intrinsics

    if not args.live:
        img = render(sm, cli, K)
        img.save(args.out)
        m = sm['modelV2']
        print(f'saved {args.out}')
        print(f'laneLines y@0m={[round(float(np.array(ll.y)[0]), 2) for ll in m.laneLines]}')
        return

    import pygame
    pygame.init()
    disp_w, disp_h = (W + TD_W) // 2, H // 2
    screen = pygame.display.set_mode((disp_w, disp_h))
    pygame.display.set_caption('Model Vision — path / lanes / edges / lead')
    clock = pygame.time.Clock()
    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
        sm.update(50)
        img = render(sm, cli, K)
        if img is not None:
            img = img.resize((disp_w, disp_h))
            surf = pygame.image.frombuffer(img.tobytes(), img.size, 'RGB')
            screen.blit(surf, (0, 0))
            pygame.display.flip()
        clock.tick(8)
    pygame.quit()


if __name__ == '__main__':
    main()

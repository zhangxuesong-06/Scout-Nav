#!/usr/bin/env python3
"""RGB-only Orbbec grape detection preview; no ROS or motion commands."""
import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', help='RGB /dev/videoN or /dev/v4l/by-id path')
    parser.add_argument('--model', default=str(root / 'models/grapes/wgisd-grape-cluster-detection-v1.pt'))
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--imgsz', type=int, default=800)
    parser.add_argument('--device', default='0')
    args = parser.parse_args()
    if not 0 < args.conf < 1 or args.imgsz <= 0:
        parser.error('Require 0 < conf < 1 and imgsz > 0')
    if not Path(args.model).is_file():
        parser.error(f'Model not found: {args.model}')
    if args.camera is None:
        cameras = sorted(Path('/dev/v4l/by-id').glob('*Orbbec*video-index0'))
        if len(cameras) != 1:
            parser.error(f'Found {len(cameras)} Orbbec RGB cameras; choose one with --camera: {cameras}')
        args.camera = str(cameras[0])

    model = YOLO(args.model)
    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f'Cannot open RGB camera: {args.camera}')
    out = root / 'runs/grape_camera_live'
    out.mkdir(parents=True, exist_ok=True)
    title = 'Scout grape detection - Q or Esc to quit, S to save'
    frame_count, last_report = 0, 0.0
    started = time.monotonic()
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 15)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(title, 960, 720)
        print(f'Camera: {args.camera}\nModel: {args.model}\nClasses: {model.names}\n'
              f'Confidence: {args.conf}; RGB preview only. Q/Esc exits; S saves to {out}', flush=True)
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('RGB camera stopped providing frames')
            result = model.predict(frame, device=args.device, imgsz=args.imgsz,
                                   conf=args.conf, verbose=False)[0]
            annotated = result.plot(line_width=2)
            frame_count += 1
            now = time.monotonic()
            fps = frame_count / max(now - started, 0.001)
            cv2.putText(annotated, f'Detections: {len(result.boxes)} | conf >= {args.conf:.2f} | {fps:.1f} FPS',
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.imshow(title, annotated)
            key = cv2.waitKey(1) & 0xff
            if frame_count == 1 or key in (ord('s'), ord('S')):
                name = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                cv2.imwrite(str(out / f'{name}_raw.jpg'), frame)
                cv2.imwrite(str(out / f'{name}_detection.jpg'), annotated)
                print(f'Saved RGB and detection images: {out / name}', flush=True)
            if now - last_report >= 5:
                report = dict(time=datetime.now().astimezone().isoformat(), camera=args.camera,
                              model=args.model, conf=args.conf, imgsz=args.imgsz,
                              frames=frame_count, fps=round(fps, 2), shape=list(frame.shape),
                              detections=[dict(name=model.names[int(box.cls.item())],
                                               confidence=float(box.conf.item()),
                                               box_xyxy=box.xyxy.cpu().tolist()[0])
                                          for box in result.boxes])
                status_path = out / 'status.json'
                temp_path = status_path.with_suffix('.tmp')
                temp_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
                temp_path.replace(status_path)
                last_report = now
                print(f'Frames: {frame_count}; {fps:.1f} FPS; detections: {len(result.boxes)}', flush=True)
            if key in (27, ord('q'), ord('Q')) or cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print('Camera released; preview stopped.', flush=True)


if __name__ == '__main__':
    main()

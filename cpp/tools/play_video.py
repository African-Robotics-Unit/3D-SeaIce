#!/usr/bin/env python3
"""Simple MP4 player — press space to pause/resume, q to quit, left/right to step frames."""

import sys
import cv2

def play(path: str) -> None:
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit(f"Cannot open: {path}")

    fps      = cap.get(cv2.CAP_PROP_FPS) or 25.0
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    delay_ms = max(1, int(1000 / fps))

    paused = False
    print(f"{path}  {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}  {fps:.1f} fps  "
          f"{n_frames} frames\n"
          "Controls: space=pause  q=quit  ←/→=step frame")

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                break  # end of file

        pos = int(cap.get(cv2.CAP_PROP_POS_FRAMES))
        cv2.setWindowTitle("player", f"{path}  [{pos}/{n_frames}]")
        cv2.imshow("player", frame)

        key = cv2.waitKey(1 if paused else delay_ms) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            paused = not paused
        elif key == 81 or key == 2:  # left arrow (Linux / Mac)
            cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, pos - 2))
            ok, frame = cap.read()
            paused = True
        elif key == 83 or key == 3:  # right arrow
            ok, frame = cap.read()
            paused = True

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Usage: python3 play_video.py <file.mp4>")
    play(sys.argv[1])

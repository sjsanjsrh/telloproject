from __future__ import annotations

from pathlib import Path
import random

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent / "res" / "data_cropped_yolo"
TRAIN_IMG_DIR = ROOT / "images" / "train"
VAL_IMG_DIR = ROOT / "images" / "val"
TRAIN_LABEL_DIR = ROOT / "labels" / "train"
VAL_LABEL_DIR = ROOT / "labels" / "val"
OUT_PATH = Path(__file__).resolve().parent / "res" / "dataset_preview.png"


def parse_segments(label_path: Path, width: int, height: int):
    if not label_path.exists():
        return []

    segments = []
    for line in label_path.read_text(encoding="utf-8").splitlines():
        parts = line.strip().split()
        if len(parts) < 7:
            continue
        class_id = int(float(parts[0]))
        coords = np.asarray([float(value) for value in parts[1:]], dtype=np.float32)
        points = coords.reshape(-1, 2)
        points[:, 0] *= width
        points[:, 1] *= height
        segments.append((class_id, points))
    return segments


def load_sample_paths() -> list[tuple[Path, Path, str]]:
    samples: list[tuple[Path, Path, str]] = []
    for image_dir, label_dir, split_name in (
        (TRAIN_IMG_DIR, TRAIN_LABEL_DIR, "train"),
        (VAL_IMG_DIR, VAL_LABEL_DIR, "val"),
    ):
        if not image_dir.exists():
            continue
        candidates = sorted(image_dir.glob("*.jpg"))[:4]
        for image_path in candidates:
            label_path = label_dir / f"{image_path.stem}.txt"
            samples.append((image_path, label_path, split_name))
    return samples


def draw_overlay(image: np.ndarray, label_path: Path) -> np.ndarray:
    output = image.copy()
    height, width = image.shape[:2]
    segments = parse_segments(label_path, width, height)

    for class_id, points in segments:
        contour = np.round(points).astype(np.int32).reshape(-1, 1, 2)
        if len(contour) < 3:
            continue

        color = (0, 200, 0) if class_id == 0 else (0, 120, 255)
        cv2.polylines(output, [contour], True, color, 2, cv2.LINE_AA)
        cv2.fillPoly(output, [contour], color=(color[0], color[1], color[2]))
        cv2.addWeighted(output, 0.25, image, 0.75, 0.0, output)
        cv2.polylines(output, [contour], True, color, 2, cv2.LINE_AA)

        x, y, w, h = cv2.boundingRect(contour)
        thickness = min(w, h)
        cv2.rectangle(output, (x, y), (x + w, y + h), (255, 255, 255), 1)
        cv2.putText(
            output,
            f"cls={class_id} bbox={w}x{h}px th~{thickness}px",
            (x, max(18, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    split_tag = label_path.parent.name
    cv2.putText(output, f"{split_tag}: {image.shape[1]}x{image.shape[0]}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return output


def main() -> None:
    samples = load_sample_paths()
    if not samples:
        raise RuntimeError(f"No samples found under {ROOT}")

    random.seed(7)
    random.shuffle(samples)
    samples = samples[:8]

    tiles = []
    for image_path, label_path, split_name in samples:
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        overlay = draw_overlay(image, label_path)
        tile = cv2.resize(overlay, (640, 360), interpolation=cv2.INTER_AREA)
        label = f"{split_name} | {image_path.name}"
        cv2.putText(tile, label, (10, 348), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(tile, label, (10, 348), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)

    if not tiles:
        raise RuntimeError("No readable images found for preview")

    cols = 2
    rows = (len(tiles) + cols - 1) // cols
    blank = np.zeros_like(tiles[0])
    while len(tiles) < rows * cols:
        tiles.append(blank.copy())

    row_images = []
    for row_index in range(rows):
        row_images.append(np.hstack(tiles[row_index * cols : (row_index + 1) * cols]))

    grid = np.vstack(row_images)
    cv2.imwrite(str(OUT_PATH), grid)
    print(f"saved: {OUT_PATH}")


if __name__ == "__main__":
    main()

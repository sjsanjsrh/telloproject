from __future__ import annotations

import json
import argparse
from pathlib import Path
import random
import re

import numpy as np
import cv2

PROJECT_ROOT = Path(__file__).resolve().parent
YOLO_ROOT_CANDIDATES = (
    PROJECT_ROOT / "res" / "data_cropped_yolo",
    PROJECT_ROOT / "data_cropped_yolo",
)
COCO_JSON_PATH = PROJECT_ROOT / "data" / "result_coco.json"
COCO_IMAGE_DIR = PROJECT_ROOT / "data" / "img"
OUT_PATH = PROJECT_ROOT / "res" / "dataset_preview.png"
IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize YOLO/COCO segmentation labels.")
    parser.add_argument("--filter", default="", help="only visualize images whose file name contains this text")
    parser.add_argument("--filter-regex", default="", help="only visualize images whose file name matches this regex")
    parser.add_argument("--out", default=str(OUT_PATH), help="output preview image path")
    return parser.parse_args()


def find_yolo_root() -> Path | None:
    for root in YOLO_ROOT_CANDIDATES:
        if (root / "images").exists() and (root / "labels").exists():
            return root
    return None


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


def load_yolo_samples(root: Path) -> tuple[list[tuple[Path, Path, str]], list[tuple[Path, str]]]:
    train_img_dir = root / "images" / "train"
    val_img_dir = root / "images" / "val"
    train_label_dir = root / "labels" / "train"
    val_label_dir = root / "labels" / "val"

    samples: list[tuple[Path, Path, str]] = []
    missing_labels: list[tuple[Path, str]] = []
    for image_dir, label_dir, split_name in (
        (train_img_dir, train_label_dir, "train"),
        (val_img_dir, val_label_dir, "val"),
    ):
        if not image_dir.exists():
            continue
        candidates = []
        for ext in IMAGE_EXTENSIONS:
            candidates.extend(image_dir.glob(f"*{ext}"))
        for image_path in sorted(candidates):
            label_path = label_dir / f"{image_path.stem}.txt"
            if label_path.exists():
                samples.append((image_path, label_path, split_name))
            else:
                missing_labels.append((image_path, split_name))
    return samples, missing_labels


def _resolve_image_path(image_dir: Path, file_name: str) -> Path | None:
    direct = image_dir / file_name
    if direct.exists():
        return direct

    base_name = Path(file_name).name
    direct = image_dir / base_name
    if direct.exists():
        return direct

    if "-" in base_name:
        suffix_name = base_name.split("-", 1)[1]
        direct = image_dir / suffix_name
        if direct.exists():
            return direct

    stem = Path(file_name).stem
    for ext in IMAGE_EXTENSIONS:
        candidate = image_dir / f"{stem}{ext}"
        if candidate.exists():
            return candidate
    return None


def load_coco_samples() -> tuple[list[tuple[Path, list[dict], str]], list[tuple[str, str]]]:
    data = json.loads(COCO_JSON_PATH.read_text(encoding="utf-8"))
    images = sorted(data.get("images", []), key=lambda item: item.get("id", 0))
    annotations = data.get("annotations", [])

    ann_by_image_id: dict[int, list[dict]] = {}
    for ann in annotations:
        ann_by_image_id.setdefault(int(ann.get("image_id", -1)), []).append(ann)

    samples: list[tuple[Path, list[dict], str]] = []
    missing_images: list[tuple[str, str]] = []
    for image_info in images:
        file_name = image_info.get("file_name", "")
        image_path = _resolve_image_path(COCO_IMAGE_DIR, file_name)
        if image_path is None:
            missing_images.append((file_name, "missing image file"))
            continue
        image_id = int(image_info.get("id", -1))
        samples.append((image_path, ann_by_image_id.get(image_id, []), "coco"))

    return samples, missing_images


def draw_overlay_from_yolo(image: np.ndarray, label_path: Path) -> np.ndarray:
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


def draw_overlay_from_coco(image: np.ndarray, annotations: list[dict], image_label: str) -> np.ndarray:
    output = image.copy()
    height, width = image.shape[:2]

    for ann in annotations:
        class_id = int(ann.get("category_id", 0)) - 1
        color = (0, 200, 0) if class_id == 0 else (0, 120, 255)
        for segment in ann.get("segmentation", []):
            coords = np.asarray(segment, dtype=np.float32)
            if coords.size < 6:
                continue
            points = coords.reshape(-1, 2)
            contour = np.round(points).astype(np.int32).reshape(-1, 1, 2)
            if len(contour) < 3:
                continue

            cv2.polylines(output, [contour], True, color, 2, cv2.LINE_AA)
            cv2.fillPoly(output, [contour], color=(color[0], color[1], color[2]))
            cv2.addWeighted(output, 0.25, image, 0.75, 0.0, output)
            cv2.polylines(output, [contour], True, color, 2, cv2.LINE_AA)

            x, y, w, h = cv2.boundingRect(contour)
            thickness = min(w, h)
            cv2.rectangle(output, (x, y), (x + w, y + h), (255, 255, 255), 1)
            cv2.putText(
                output,
                f"cat={ann.get('category_id')} bbox={w}x{h}px th~{thickness}px",
                (x, max(18, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

    cv2.putText(output, f"{image_label}: {image.shape[1]}x{image.shape[0]}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return output


def main() -> None:
    args = parse_args()
    yolo_root = find_yolo_root()
    if yolo_root is not None:
        mode = "yolo"
        samples, missing_items = load_yolo_samples(yolo_root)
    elif COCO_JSON_PATH.exists() and COCO_IMAGE_DIR.exists():
        mode = "coco"
        samples, missing_items = load_coco_samples()
    else:
        roots = ", ".join(str(root) for root in YOLO_ROOT_CANDIDATES)
        raise RuntimeError(f"No samples found under {roots} or {COCO_IMAGE_DIR}")

    if not samples:
        raise RuntimeError("No readable samples found")

    if args.filter:
        samples = [sample for sample in samples if args.filter in sample[0].name]
        if not samples:
            raise RuntimeError(f"No samples matched filter: {args.filter}")
    if args.filter_regex:
        pattern = re.compile(args.filter_regex)
        samples = [sample for sample in samples if pattern.search(sample[0].name)]
        if not samples:
            raise RuntimeError(f"No samples matched regex: {args.filter_regex}")

    print(f"mode: {mode}")
    if mode == "yolo":
        print(f"root: {yolo_root}")
    print(f"found {len(samples)} samples")
    if missing_items:
        print(f"missing items: {len(missing_items)}")
        for item, reason in missing_items:
            print(f"  {reason}: {item}")
    else:
        print("missing items: 0")

    random.seed(7)
    random.shuffle(samples)

    tiles = []
    for sample in samples:
        image_path = sample[0]
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        if mode == "yolo":
            _, label_path, split_name = sample
            overlay = draw_overlay_from_yolo(image, label_path)
        else:
            _, annotations, split_name = sample
            overlay = draw_overlay_from_coco(image, annotations, image_path.name)
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
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), grid)
    print(f"saved: {out_path}")


if __name__ == "__main__":
    main()

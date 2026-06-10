from __future__ import annotations

import argparse
import json
import random
import re
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent
MASK_ROOT_CANDIDATES = (
    PROJECT_ROOT / "res" / "data_masks",
    PROJECT_ROOT / "data_masks",
)
OUT_PATH = PROJECT_ROOT / "res" / "dataset_preview.png"
IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
PALETTE = (
    (40, 220, 80),
    (40, 160, 255),
    (230, 80, 255),
    (255, 190, 40),
    (80, 255, 230),
    (255, 90, 90),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize mask PNG dataset samples.")
    parser.add_argument("--root", default="", help="mask dataset root. Defaults to yolo_seg/res/data_masks if present")
    parser.add_argument("--filter", default="", help="only visualize images whose file name contains this text")
    parser.add_argument("--filter-regex", default="", help="only visualize images whose file name matches this regex")
    parser.add_argument("--out", default=str(OUT_PATH), help="output preview image path")
    parser.add_argument("--max-samples", type=int, default=24, help="maximum number of samples to render")
    return parser.parse_args()


def find_mask_root(root_arg: str) -> Path | None:
    if root_arg:
        root = Path(root_arg)
        return root if root.exists() else None

    for root in MASK_ROOT_CANDIDATES:
        if (root / "images").exists() and (root / "masks").exists():
            return root
    return None


def load_split_annotations(root: Path, split_name: str) -> dict[str, list[dict]]:
    annotation_path = root / "annotations" / f"{split_name}.json"
    if not annotation_path.exists():
        return {}

    annotations = json.loads(annotation_path.read_text(encoding="utf-8"))
    return {item.get("image", ""): item.get("objects", []) for item in annotations}


def collect_samples(root: Path) -> tuple[list[tuple[Path, Path, str, list[dict]]], list[tuple[Path, str]]]:
    samples: list[tuple[Path, Path, str, list[dict]]] = []
    missing_masks: list[tuple[Path, str]] = []

    for split_name in ("train", "val"):
        image_dir = root / "images" / split_name
        mask_dir = root / "masks" / split_name
        if not image_dir.exists():
            continue

        images: list[Path] = []
        for ext in IMAGE_EXTENSIONS:
            images.extend(image_dir.glob(f"*{ext}"))

        objects_by_image = load_split_annotations(root, split_name)
        for image_path in sorted(images):
            mask_path = mask_dir / f"{image_path.stem}.png"
            if mask_path.exists():
                samples.append((image_path, mask_path, split_name, objects_by_image.get(image_path.name, [])))
            else:
                missing_masks.append((image_path, "missing mask"))

    return samples, missing_masks


def draw_box_label(output: np.ndarray, bbox: list[int], label: str, color: tuple[int, int, int]) -> None:
    x, y, width, height = bbox
    x2 = x + width
    y2 = y + height
    cv2.rectangle(output, (x, y), (x2, y2), color, 2, cv2.LINE_AA)

    text = label or "object"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.55
    thickness = 2
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    label_y = y - 8 if y - text_height - 10 >= 0 else y + text_height + 10
    top_left = (x, label_y - text_height - baseline - 4)
    bottom_right = (x + text_width + 8, label_y + baseline)
    cv2.rectangle(output, top_left, bottom_right, color, -1)
    cv2.putText(output, text, (x + 4, label_y - 3), font, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)


def blend_object_mask(output: np.ndarray, mask: np.ndarray, color: tuple[int, int, int]) -> None:
    if mask.shape[:2] != output.shape[:2]:
        mask = cv2.resize(mask, (output.shape[1], output.shape[0]), interpolation=cv2.INTER_NEAREST)

    mask_bool = mask > 0
    color_layer = np.zeros_like(output)
    color_layer[mask_bool] = color
    blended = cv2.addWeighted(output, 0.72, color_layer, 0.28, 0)
    output[mask_bool] = blended[mask_bool]

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is not None:
        hierarchy = hierarchy[0]
        for index, contour in enumerate(contours):
            contour_color = color if hierarchy[index][3] == -1 else (0, 0, 255)
            cv2.drawContours(output, [contour], -1, contour_color, 2, cv2.LINE_AA)


def color_for_object(obj: dict) -> tuple[int, int, int]:
    class_id = int(obj.get("class_id", 0))
    return PALETTE[class_id % len(PALETTE)]


def draw_overlay(image: np.ndarray, mask: np.ndarray, split_name: str, objects: list[dict], root: Path) -> np.ndarray:
    output = image.copy()

    if objects:
        for obj in objects:
            color = color_for_object(obj)
            object_mask_name = obj.get("mask", "")
            object_mask_path = root / "object_masks" / split_name / object_mask_name
            object_mask = cv2.imread(str(object_mask_path), cv2.IMREAD_GRAYSCALE) if object_mask_name else None
            if object_mask is None:
                object_mask = mask
            blend_object_mask(output, object_mask, color)
    else:
        blend_object_mask(output, mask, PALETTE[0])

    cv2.putText(output, f"{split_name}: {image.shape[1]}x{image.shape[0]}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    for obj in objects:
        bbox = obj.get("bbox_xywh")
        if bbox and len(bbox) == 4:
            color = color_for_object(obj)
            draw_box_label(output, [int(value) for value in bbox], obj.get("label", "object"), color)
    return output


def main() -> None:
    args = parse_args()
    root = find_mask_root(args.root)
    if root is None:
        candidates = ", ".join(str(candidate) for candidate in MASK_ROOT_CANDIDATES)
        raise RuntimeError(f"No mask dataset found. Checked: {candidates}")

    samples, missing_items = collect_samples(root)
    if args.filter:
        samples = [sample for sample in samples if args.filter in sample[0].name]
    if args.filter_regex:
        pattern = re.compile(args.filter_regex)
        samples = [sample for sample in samples if pattern.search(sample[0].name)]
    if not samples:
        raise RuntimeError("No readable mask samples found")

    print(f"mode: mask")
    print(f"root: {root}")
    print(f"found {len(samples)} samples")
    print(f"missing items: {len(missing_items)}")
    for item, reason in missing_items[:20]:
        print(f"  {reason}: {item}")

    random.seed(7)
    random.shuffle(samples)
    samples = samples[: args.max_samples]

    tiles = []
    for image_path, mask_path, split_name, objects in samples:
        image = cv2.imread(str(image_path))
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if image is None or mask is None:
            continue

        overlay = draw_overlay(image, mask, split_name, objects, root)
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

    grid = np.vstack([np.hstack(tiles[row * cols : (row + 1) * cols]) for row in range(rows)])
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), grid)
    print(f"saved: {out_path}")


if __name__ == "__main__":
    main()

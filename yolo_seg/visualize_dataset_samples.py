from __future__ import annotations

import argparse
import json
import random
import re
import sys
from pathlib import Path

import cv2
import numpy as np

CURRENT_DIR = Path(__file__).resolve().parent
REPO_ROOT = CURRENT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from yolo_seg.prepare_coco import split_label_studio_masks

PROJECT_ROOT = CURRENT_DIR
MASK_ROOT_CANDIDATES = (
    PROJECT_ROOT / "res" / "data_masks",
    PROJECT_ROOT / "data_masks",
)
IMAGE_ROOT_CANDIDATES = (
    PROJECT_ROOT / "data",
)
LABEL_STUDIO_JSON_CANDIDATES = (
    PROJECT_ROOT / "data",
    PROJECT_ROOT,
)
OUT_PATH = PROJECT_ROOT / "res" / "dataset_preview.png"
PREPROCESSED_MASK_ROOT = PROJECT_ROOT / "res" / "data_masks"
IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
DEFAULT_FILL_LABELS = {"full_gate"}
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
    parser.add_argument(
        "--root",
        default="",
        help="dataset root. Defaults to yolo_seg/res/data_masks for mask datasets, otherwise yolo_seg/data for image-only datasets",
    )
    parser.add_argument("--filter", default="", help="only visualize images whose file name contains this text")
    parser.add_argument("--filter-regex", default="", help="only visualize images whose file name matches this regex")
    parser.add_argument("--out", default=str(OUT_PATH), help="output preview image path")
    parser.add_argument("--max-samples", type=int, default=24, help="maximum number of samples to render")
    return parser.parse_args()


def find_mask_root(root_arg: str) -> Path | None:
    if root_arg:
        root = Path(root_arg)
        if root.exists() and (root / "images").exists() and (root / "masks").exists():
            return root
        return None

    for root in MASK_ROOT_CANDIDATES:
        if (root / "images").exists() and (root / "masks").exists():
            return root
    return None


def find_image_root(root_arg: str) -> Path | None:
    if root_arg:
        root = Path(root_arg)
        return root if root.exists() else None

    for root in IMAGE_ROOT_CANDIDATES:
        if root.exists():
            return root
    return None


def find_label_studio_json(root_arg: str) -> Path | None:
    if root_arg:
        root = Path(root_arg)
        if root.is_file() and root.suffix.lower() == ".json" and root.exists():
            return root
        if root.is_dir():
            json_files = sorted(root.glob("*.json"))
            if json_files:
                return json_files[0]
        return None

    for root in LABEL_STUDIO_JSON_CANDIDATES:
        if not root.exists():
            continue
        json_files = sorted(root.glob("*.json"))
        if json_files:
            return json_files[0]
    return None


def load_split_annotations(root: Path, split_name: str) -> dict[str, list[dict]]:
    annotation_path = root / "annotations" / f"{split_name}.json"
    if not annotation_path.exists():
        return {}

    annotations = json.loads(annotation_path.read_text(encoding="utf-8"))
    return {item.get("image", ""): item.get("objects", []) for item in annotations}


def collect_mask_samples(root: Path) -> tuple[list[tuple[Path, Path, str, list[dict]]], list[tuple[Path, str]]]:
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


def collect_image_samples(root: Path) -> tuple[list[tuple[Path, None, str, list[dict]]], list[tuple[Path, str]]]:
    samples: list[tuple[Path, None, str, list[dict]]] = []
    skipped: list[tuple[Path, str]] = []

    def has_images(directory: Path) -> bool:
        return any(directory.glob(f"*{ext}") for ext in IMAGE_EXTENSIONS)

    image_dirs = [path for path in root.rglob("*") if path.is_dir() and has_images(path)]
    if not image_dirs and any(root.glob(f"*{ext}")):
        image_dirs = [root]

    for image_dir in sorted(set(image_dirs)):
        split_name = image_dir.relative_to(root).parts[0] if image_dir != root and image_dir.relative_to(root).parts else root.name
        images: list[Path] = []
        for ext in IMAGE_EXTENSIONS:
            images.extend(image_dir.glob(f"*{ext}"))

        for image_path in sorted(images):
            samples.append((image_path, None, split_name, []))

    if not samples:
        skipped.append((root, "no images found"))

    return samples, skipped


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


def draw_overlay(
    image: np.ndarray,
    mask: np.ndarray | None,
    split_name: str,
    objects: list[dict],
    root: Path,
) -> np.ndarray:
    output = image.copy()

    if objects:
        for obj in objects:
            color = color_for_object(obj)
            object_mask_name = obj.get("mask", "")
            object_mask_path = root / "object_masks" / split_name / object_mask_name
            object_mask = cv2.imread(str(object_mask_path), cv2.IMREAD_GRAYSCALE) if object_mask_name else None
            if object_mask is not None:
                blend_object_mask(output, object_mask, color)
    elif mask is not None:
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
    mode = "mask"
    root = find_mask_root(args.root)
    if root is not None:
        samples, missing_items = collect_mask_samples(root)
    else:
        root = find_image_root(args.root)
        if root is None:
            mask_candidates = ", ".join(str(candidate) for candidate in MASK_ROOT_CANDIDATES)
            image_candidates = ", ".join(str(candidate) for candidate in IMAGE_ROOT_CANDIDATES)
            raise RuntimeError(f"No dataset found. Checked mask roots: {mask_candidates}; image roots: {image_candidates}")

        json_path = find_label_studio_json(args.root) or find_label_studio_json(str(root))
        if json_path is not None:
            print(f"preparing mask dataset from: {json_path}")
            split_label_studio_masks(
                json_path,
                root,
                PREPROCESSED_MASK_ROOT,
                split=0.8,
                seed=42,
                fill_labels=DEFAULT_FILL_LABELS,
                min_component_area_ratio=0.0005,
            )
            root = PREPROCESSED_MASK_ROOT
            mode = "preprocessed-mask"
            samples, missing_items = collect_mask_samples(root)
        else:
            mode = "image"
            samples, missing_items = collect_image_samples(root)

    if args.filter:
        samples = [sample for sample in samples if args.filter in sample[0].name]
    if args.filter_regex:
        pattern = re.compile(args.filter_regex)
        samples = [sample for sample in samples if pattern.search(sample[0].name)]
    if not samples:
        raise RuntimeError("No readable mask samples found")

    print(f"mode: {mode}")
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
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE) if mask_path is not None else None
        if image is None or (mask_path is not None and mask is None):
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

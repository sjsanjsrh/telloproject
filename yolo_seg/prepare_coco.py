from __future__ import annotations

import argparse
import json
import random
import shutil
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

import cv2
import numpy as np

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")
DEFAULT_MIN_COMPONENT_AREA_RATIO = 0.0005


class InputStream:
    def __init__(self, data: str):
        self.data = data
        self.i = 0

    def read(self, size: int) -> int:
        out = self.data[self.i : self.i + size]
        self.i += size
        return int(out, 2)


def access_bit(data: list[int], num: int) -> int:
    base = int(num // 8)
    shift = 7 - int(num % 8)
    return (data[base] & (1 << shift)) >> shift


def bytes_to_bits(data: list[int]) -> str:
    return "".join(str(access_bit(data, i)) for i in range(len(data) * 8))


def decode_label_studio_rle(rle: list[int]) -> np.ndarray:
    stream = InputStream(bytes_to_bits(rle))
    num = stream.read(32)
    word_size = stream.read(5) + 1
    rle_sizes = [stream.read(4) + 1 for _ in range(4)]

    i = 0
    out = np.zeros(num, dtype=np.uint8)
    while i < num:
        is_run = stream.read(1)
        end = i + 1 + stream.read(rle_sizes[stream.read(2)])
        if is_run:
            out[i:end] = stream.read(word_size)
            i = end
        else:
            while i < end:
                out[i] = stream.read(word_size)
                i += 1
    return out


def resolve_cli_path(path_text: str, must_exist: bool = False) -> Path:
    candidate = Path(path_text)
    if candidate.is_absolute():
        return candidate

    current_dir = Path(__file__).resolve().parent
    repo_root = current_dir.parent
    for base_dir in (repo_root, current_dir):
        resolved = base_dir / candidate
        if not must_exist or resolved.exists():
            return resolved

    return repo_root / candidate


def resolve_label_studio_image_path(image_value: str, image_root: Path) -> Path:
    parsed = urlparse(image_value)
    if parsed.path == "/data/local-files/":
        relative_text = parse_qs(parsed.query).get("d", [""])[0]
    else:
        relative_text = image_value

    relative = Path(unquote(relative_text).replace("\\", "/"))
    if relative.is_absolute():
        return relative

    parts = relative.parts
    if parts and parts[0] == "tello_slap_imgs":
        relative = Path(*parts[1:])

    return image_root / relative


def make_unique_name(image_path: Path, task_id: int | str, used_names: set[str]) -> str:
    name = image_path.name
    if name not in used_names:
        used_names.add(name)
        return name

    counter = 0
    while True:
        suffix = f"{task_id}-{counter}-" if counter else f"{task_id}-"
        unique_name = f"{suffix}{name}"
        if unique_name not in used_names:
            used_names.add(unique_name)
            return unique_name
        counter += 1


def extract_brush_results(task: dict) -> list[dict]:
    annotations = task.get("annotations") or []
    if not annotations:
        return []

    results = annotations[0].get("result") or []
    return [
        result
        for result in results
        if result.get("type") == "brushlabels"
        and result.get("value", {}).get("format") == "rle"
        and result.get("value", {}).get("rle")
    ]


def collect_label_names(tasks: list[dict]) -> list[str]:
    labels: list[str] = []
    seen: set[str] = set()
    for task in tasks:
        for result in extract_brush_results(task):
            for label in result.get("value", {}).get("brushlabels", ["object"]):
                if label not in seen:
                    seen.add(label)
                    labels.append(label)
    return labels or ["object"]


def merge_brush_masks(results: list[dict]) -> np.ndarray | None:
    merged: np.ndarray | None = None
    for result in results:
        mask = brush_result_to_mask(result)
        if mask is None:
            continue
        merged = mask if merged is None else cv2.bitwise_or(merged, mask)
    return merged


def brush_result_to_mask(result: dict) -> np.ndarray | None:
    value = result["value"]
    width = int(result.get("original_width") or 0)
    height = int(result.get("original_height") or 0)
    if width <= 0 or height <= 0:
        return None

    decoded = decode_label_studio_rle(value["rle"])
    alpha = decoded.reshape((height, width, 4))[:, :, 3]
    return np.asarray(alpha > 0, dtype=np.uint8) * 255


def fill_mask_holes(mask: np.ndarray) -> np.ndarray:
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(mask)
    if contours:
        cv2.drawContours(filled, contours, -1, 255, thickness=cv2.FILLED)
    return filled


def remove_small_holes(mask: np.ndarray, min_area_ratio: float) -> np.ndarray:
    if min_area_ratio <= 0:
        return mask

    binary = np.asarray(mask > 0, dtype=np.uint8) * 255
    height, width = binary.shape[:2]
    min_area = max(1, int(height * width * min_area_ratio))

    flood = binary.copy()
    flood_mask = np.zeros((height + 2, width + 2), dtype=np.uint8)
    cv2.floodFill(flood, flood_mask, (0, 0), 255)

    background = flood
    holes = cv2.bitwise_not(background)
    holes = cv2.bitwise_and(holes, cv2.bitwise_not(binary))

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(holes, connectivity=8)
    cleaned = binary.copy()
    for label_id in range(1, num_labels):
        area = int(stats[label_id, cv2.CC_STAT_AREA])
        if area < min_area:
            cleaned[labels == label_id] = 255
    return cleaned


def remove_small_components(mask: np.ndarray, min_area_ratio: float) -> np.ndarray:
    if min_area_ratio <= 0:
        return mask

    min_area = max(1, int(mask.shape[0] * mask.shape[1] * min_area_ratio))
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    cleaned = np.zeros_like(mask)
    for label_id in range(1, num_labels):
        area = int(stats[label_id, cv2.CC_STAT_AREA])
        if area >= min_area:
            cleaned[labels == label_id] = 255
    return cleaned


def should_fill_result_holes(result: dict, fill_labels: set[str]) -> bool:
    labels = result.get("value", {}).get("brushlabels") or []
    return any(label in fill_labels for label in labels)


def mask_to_bbox(mask: np.ndarray) -> list[int] | None:
    points = cv2.findNonZero(mask)
    if points is None:
        return None
    x, y, width, height = cv2.boundingRect(points)
    return [int(x), int(y), int(width), int(height)]


def mask_to_yolo_segments(mask: np.ndarray) -> list[list[float]]:
    height, width = mask.shape[:2]
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    segments: list[list[float]] = []
    for contour in contours:
        if len(contour) < 3:
            continue
        epsilon = max(1.0, 0.001 * cv2.arcLength(contour, True))
        approx = cv2.approxPolyDP(contour, epsilon, True)
        if len(approx) < 3:
            continue
        points = approx.reshape(-1, 2).astype(np.float32)
        points[:, 0] = np.clip(points[:, 0] / width, 0.0, 1.0)
        points[:, 1] = np.clip(points[:, 1] / height, 0.0, 1.0)
        segments.append(points.reshape(-1).tolist())
    return segments


def write_yolo_label(label_path: Path, objects: list[dict], object_mask_dir: Path) -> int:
    lines: list[str] = []
    for obj in objects:
        object_mask = cv2.imread(str(object_mask_dir / obj["mask"]), cv2.IMREAD_GRAYSCALE)
        if object_mask is None:
            continue
        for segment in mask_to_yolo_segments(object_mask):
            coords = " ".join(f"{coord:.6f}" for coord in segment)
            lines.append(f"{int(obj['class_id'])} {coords}")
    label_path.write_text("\n".join(lines), encoding="utf-8")
    return len(lines)


def write_yolo_data_yaml(out_dir: Path, names: list[str]) -> Path:
    data_yaml = out_dir / "data.yaml"
    lines = [
        f"path: {out_dir.as_posix()}",
        "train: images/train",
        "val: images/val",
        f"nc: {len(names)}",
        "names:",
    ]
    for index, name in enumerate(names):
        lines.append(f"  {index}: {name}")
    data_yaml.write_text("\n".join(lines), encoding="utf-8")
    return data_yaml


def write_dataset_info(out_dir: Path, names: list[str], report: dict) -> Path:
    info_path = out_dir / "dataset.json"
    info = {
        "format": "mask_png_numpy",
        "images": "images/{split}/*.png",
        "masks": "masks/{split}/*.png",
        "object_masks": "object_masks/{split}/*.png",
        "annotations": "annotations/{split}.json",
        "yolo_labels": "labels/{split}/*.txt",
        "data_yaml": "data.yaml",
        "labels": names,
        "report": report,
    }
    info_path.write_text(json.dumps(info, indent=2, ensure_ascii=False), encoding="utf-8")
    return info_path


def split_label_studio_masks(
    src_json: str | Path,
    image_root: str | Path,
    out_dir: str | Path,
    split: float = 0.8,
    seed: int = 42,
    fill_labels: set[str] | None = None,
    min_component_area_ratio: float = DEFAULT_MIN_COMPONENT_AREA_RATIO,
) -> dict:
    src_json = Path(src_json)
    image_root = Path(image_root)
    out_dir = Path(out_dir)

    tasks = json.loads(src_json.read_text(encoding="utf-8"))
    if not isinstance(tasks, list):
        raise ValueError("Label Studio JSON export must be a list of tasks")

    rng = random.Random(seed)
    shuffled = list(tasks)
    rng.shuffle(shuffled)
    cut = int(len(shuffled) * split)
    split_tasks = {"train": shuffled[:cut], "val": shuffled[cut:]}

    if out_dir.exists():
        shutil.rmtree(out_dir)
    for split_name in split_tasks:
        (out_dir / "images" / split_name).mkdir(parents=True, exist_ok=True)
        (out_dir / "masks" / split_name).mkdir(parents=True, exist_ok=True)
        (out_dir / "object_masks" / split_name).mkdir(parents=True, exist_ok=True)
        (out_dir / "labels" / split_name).mkdir(parents=True, exist_ok=True)
        (out_dir / "annotations").mkdir(parents=True, exist_ok=True)

    names = collect_label_names(tasks)
    label_to_id = {name: index for index, name in enumerate(names)}
    fill_labels = fill_labels or set()
    used_names: set[str] = set()
    annotations_by_split: dict[str, list[dict]] = {"train": [], "val": []}
    report = {
        "train": {"images": 0, "masks": 0},
        "val": {"images": 0, "masks": 0},
        "missing_images": [],
        "empty_masks": [],
    }

    for split_name, items in split_tasks.items():
        for task in items:
            image_value = task.get("data", {}).get("image", "")
            image_path = resolve_label_studio_image_path(image_value, image_root)
            if not image_path.exists():
                report["missing_images"].append({"task_id": task.get("id"), "image": str(image_path)})
                continue

            output_name = make_unique_name(image_path, task.get("id", "task"), used_names)
            output_stem = Path(output_name).stem
            shutil.copy2(image_path, out_dir / "images" / split_name / output_name)
            report[split_name]["images"] += 1

            results = extract_brush_results(task)
            objects: list[dict] = []
            for object_index, result in enumerate(results):
                object_mask = brush_result_to_mask(result)
                if object_mask is None:
                    continue
                if should_fill_result_holes(result, fill_labels):
                    object_mask = fill_mask_holes(object_mask)
                object_mask = remove_small_components(object_mask, min_component_area_ratio)
                object_mask = remove_small_holes(object_mask, min_component_area_ratio)
                bbox = mask_to_bbox(object_mask)
                if bbox is None:
                    continue
                label = (result.get("value", {}).get("brushlabels") or ["object"])[0]
                object_mask_name = f"{output_stem}_{object_index:02d}.png"
                cv2.imwrite(str(out_dir / "object_masks" / split_name / object_mask_name), object_mask)
                objects.append(
                    {
                        "label": label,
                        "class_id": label_to_id.get(label, -1),
                        "bbox_xywh": bbox,
                        "mask": object_mask_name,
                    }
                )

            mask = None
            for result in results:
                object_mask = brush_result_to_mask(result)
                if object_mask is None:
                    continue
                if should_fill_result_holes(result, fill_labels):
                    object_mask = fill_mask_holes(object_mask)
                object_mask = remove_small_components(object_mask, min_component_area_ratio)
                object_mask = remove_small_holes(object_mask, min_component_area_ratio)
                mask = object_mask if mask is None else cv2.bitwise_or(mask, object_mask)

            if mask is None or not np.any(mask):
                image = cv2.imread(str(image_path))
                if image is None:
                    report["empty_masks"].append({"task_id": task.get("id"), "image": output_name})
                    continue
                mask = np.zeros(image.shape[:2], dtype=np.uint8)
                report["empty_masks"].append({"task_id": task.get("id"), "image": output_name})
            else:
                mask = remove_small_holes(mask, min_component_area_ratio)

            cv2.imwrite(str(out_dir / "masks" / split_name / f"{output_stem}.png"), mask)
            report[split_name]["masks"] += 1
            yolo_label_count = write_yolo_label(
                out_dir / "labels" / split_name / f"{output_stem}.txt",
                objects,
                out_dir / "object_masks" / split_name,
            )
            annotations_by_split[split_name].append(
                {
                    "image": output_name,
                    "mask": f"{output_stem}.png",
                    "label": f"{output_stem}.txt",
                    "yolo_segments": yolo_label_count,
                    "objects": objects,
                }
            )

    for split_name, annotations in annotations_by_split.items():
        annotation_path = out_dir / "annotations" / f"{split_name}.json"
        annotation_path.write_text(json.dumps(annotations, indent=2, ensure_ascii=False), encoding="utf-8")

    data_yaml = write_yolo_data_yaml(out_dir, names)
    dataset_info = write_dataset_info(out_dir, names, report)
    report["dataset_info"] = str(dataset_info)
    report["data_yaml"] = str(data_yaml)
    report["labels"] = names
    report["filled_hole_labels"] = sorted(fill_labels)
    report["min_component_area_ratio"] = min_component_area_ratio
    return report


def verify_mask_dataset(dataset_root: Path) -> dict:
    report: dict[str, dict] = {}
    for split_name in ("train", "val"):
        image_dir = dataset_root / "images" / split_name
        mask_dir = dataset_root / "masks" / split_name
        label_dir = dataset_root / "labels" / split_name
        image_files = sorted(path for path in image_dir.iterdir() if path.suffix.lower() in IMAGE_EXTENSIONS) if image_dir.exists() else []
        mask_files = sorted(mask_dir.glob("*.png")) if mask_dir.exists() else []
        label_files = sorted(label_dir.glob("*.txt")) if label_dir.exists() else []
        image_stems = {path.stem for path in image_files}
        mask_stems = {path.stem for path in mask_files}
        label_stems = {path.stem for path in label_files}

        report[split_name] = {
            "images": len(image_files),
            "masks": len(mask_files),
            "labels": len(label_files),
            "missing_masks": [path.name for path in image_files if path.stem not in mask_stems],
            "missing_labels": [path.name for path in image_files if path.stem not in label_stems],
            "missing_images": [path.name for path in mask_files if path.stem not in image_stems],
        }
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Prepare mask PNG/NumPy dataset from Label Studio JSON.")
    parser.add_argument("--json", default="yolo_seg/data/project-2-at-2026-06-09-03-33-803af9b1.json", help="source Label Studio JSON path")
    parser.add_argument("--images", default="yolo_seg/data", help="root folder for Label Studio local-files images")
    parser.add_argument("--out", default="yolo_seg/res/data_masks", help="output mask dataset folder")
    parser.add_argument("--split", type=float, default=0.8, help="train split ratio")
    parser.add_argument("--seed", type=int, default=42, help="random seed used for splitting")
    parser.add_argument("--fill-label", action="append", default=["full_gate"], help="label whose mask holes should be filled. Can be repeated")
    parser.add_argument("--no-fill-full-gate", action="store_true", help="do not fill holes for full_gate")
    parser.add_argument(
        "--min-component-area-ratio",
        type=float,
        default=DEFAULT_MIN_COMPONENT_AREA_RATIO,
        help="remove mask components smaller than this fraction of the image area",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    src_json = resolve_cli_path(args.json, must_exist=True)
    image_root = resolve_cli_path(args.images, must_exist=True)
    out_path = resolve_cli_path(args.out)

    print(f"Label Studio JSON : {src_json}")
    print(f"Image root        : {image_root}")
    print(f"Output dataset    : {out_path}")

    fill_labels = set(args.fill_label)
    if args.no_fill_full_gate:
        fill_labels.discard("full_gate")
    report = split_label_studio_masks(
        src_json,
        image_root,
        out_path,
        split=args.split,
        seed=args.seed,
        fill_labels=fill_labels,
        min_component_area_ratio=args.min_component_area_ratio,
    )
    verify = verify_mask_dataset(out_path)

    print("--- prepare summary ---")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    print("--- mask verify ---")
    print(json.dumps(verify, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()

import json
import argparse
from pathlib import Path
import random
import shutil


def split_coco(src_json, src_images, out_dir, split=0.8, seed=42):
    """Split a COCO JSON dataset into train/val JSONs and copy images.

    src_json: path to original COCO json
    src_images: folder containing image files referenced in JSON
    out_dir: output folder where annotations/train.json, annotations/val.json and images/train,val will be created
    Returns path to generated data.yaml info (train/val json paths, nc, names)
    """
    src_json = Path(src_json)
    src_images = Path(src_images)
    out_dir = Path(out_dir)
    if not src_json.exists():
        raise FileNotFoundError(f"COCO JSON not found: {src_json}")

    data = json.loads(src_json.read_text(encoding='utf-8'))
    images = data.get('images', [])
    annotations = data.get('annotations', [])
    categories = data.get('categories', [])

    rng = random.Random(seed)
    imgs = list(images)
    rng.shuffle(imgs)
    cut = int(len(imgs) * split)
    train_imgs = imgs[:cut]
    val_imgs = imgs[cut:]
    train_ids = {im['id'] for im in train_imgs}
    val_ids = {im['id'] for im in val_imgs}

    train_ann = [a for a in annotations if a.get('image_id') in train_ids]
    val_ann = [a for a in annotations if a.get('image_id') in val_ids]

    annot_dir = out_dir / 'annotations'
    images_root = out_dir / 'images'

    # Rebuild the generated dataset folders from scratch so stale files from
    # earlier preprocessing runs do not leak into the next split.
    if annot_dir.exists():
        shutil.rmtree(annot_dir)
    if images_root.exists():
        shutil.rmtree(images_root)

    annot_dir.mkdir(parents=True, exist_ok=True)
    images_train_dir = images_root / 'train'
    images_val_dir = images_root / 'val'
    images_train_dir.mkdir(parents=True, exist_ok=True)
    images_val_dir.mkdir(parents=True, exist_ok=True)

    # write train.json and val.json
    train_json = {
        'images': train_imgs,
        'annotations': train_ann,
        'categories': categories,
    }
    val_json = {
        'images': val_imgs,
        'annotations': val_ann,
        'categories': categories,
    }
    train_json_path = annot_dir / 'train.json'
    val_json_path = annot_dir / 'val.json'
    train_json_path.write_text(json.dumps(train_json), encoding='utf-8')
    val_json_path.write_text(json.dumps(val_json), encoding='utf-8')

    source_images = sorted([image_path for image_path in src_images.iterdir() if image_path.is_file()])

    def match_source_images(file_name: str) -> list[Path]:
        exact = src_images / file_name
        if exact.exists():
            return [exact]

        base_name = Path(file_name).name
        exact = src_images / base_name
        if exact.exists():
            return [exact]

        matches = []
        for image_path in source_images:
            if image_path.name in file_name or file_name in image_path.name:
                matches.append(image_path)
        return matches

    match_report = {
        'missing': [],
        'ambiguous': [],
        'duplicate_source_uses': [],
    }

    source_uses: dict[str, list[str]] = {}
    for im in images:
        fname = im.get('file_name')
        matches = match_source_images(fname)
        if not matches:
            match_report['missing'].append(fname)
            continue
        if len(matches) > 1:
            match_report['ambiguous'].append({
                'coco_file': fname,
                'matches': [path.name for path in matches],
            })
            continue
        source_uses.setdefault(matches[0].name, []).append(fname)

    for source_name, coco_names in source_uses.items():
        if len(coco_names) > 1:
            match_report['duplicate_source_uses'].append({
                'source_file': source_name,
                'coco_files': coco_names,
            })

    if match_report['missing'] or match_report['ambiguous']:
        raise RuntimeError(f"Image matching failed: {json.dumps(match_report, indent=2, ensure_ascii=False)}")

    def find_source_image(file_name: str) -> Path | None:
        matches = match_source_images(file_name)
        if len(matches) == 1:
            return matches[0]
        return None

    # copy image files referenced in train/val
    def copy_images(img_list, dst_dir):
        for im in img_list:
            fname = im.get('file_name')
            src = find_source_image(fname)

            if src is not None and src.exists():
                shutil.copy2(src, dst_dir / Path(fname).name)

    copy_images(train_imgs, images_train_dir)
    copy_images(val_imgs, images_val_dir)

    # build data.yaml-like dict
    names = [c.get('name') for c in categories]
    nc = len(categories)

    return {
        'train_json': str(train_json_path),
        'val_json': str(val_json_path),
        'nc': nc,
        'names': names,
        'out_dir': str(out_dir),
        'match_report': match_report,
    }


def verify_yolo_dataset(dataset_root: Path) -> dict:
    report: dict[str, dict] = {}
    for split_name in ('train', 'val'):
        image_dir = dataset_root / 'images' / split_name
        label_dir = dataset_root / 'labels' / split_name

        image_files = sorted([path for path in image_dir.iterdir() if path.is_file()]) if image_dir.exists() else []
        label_files = sorted(label_dir.glob('*.txt')) if label_dir.exists() else []
        label_stems = {path.stem for path in label_files}

        missing_labels = [path.name for path in image_files if path.stem not in label_stems]
        missing_images = [path.name for path in label_files if not any((image_dir / f'{path.stem}{ext}').exists() for ext in ('.jpg', '.jpeg', '.png', '.bmp', '.webp'))]

        report[split_name] = {
            'images': len(image_files),
            'labels': len(label_files),
            'missing_labels': missing_labels,
            'missing_images': missing_images,
        }

    return report


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


def write_out_yaml(out_dir: Path, base_info: dict):
    out_dir.mkdir(parents=True, exist_ok=True)
    out_yaml = out_dir / 'data.yaml'
    nc = base_info.get('nc', 1)
    names = base_info.get('names', [])
    content_lines = []
    content_lines.append(f'path: {out_dir.as_posix()}')
    content_lines.append('train: images/train')
    content_lines.append('val: images/val')
    content_lines.append(f'nc: {nc}')
    if names:
        content_lines.append('names:')
        for index, name in enumerate(names):
            content_lines.append(f'  {index}: {name}')
    out_yaml.write_text('\n'.join(content_lines), encoding='utf-8')
    return out_yaml


def parse_args():
    parser = argparse.ArgumentParser(description='Split COCO JSON into train/val and convert to YOLO segmentation format.')
    parser.add_argument('--json', default='yolo_seg/data/result_coco.json', help='source COCO JSON path')
    parser.add_argument('--images', default='yolo_seg/data/img', help='source image directory referenced by the COCO JSON')
    parser.add_argument('--out', default='yolo_seg/res/data_cropped', help='output folder for split data')
    parser.add_argument('--split', type=float, default=0.8, help='train split ratio')
    parser.add_argument('--seed', type=int, default=42, help='random seed used for splitting')
    return parser.parse_args()


def main():
    from ultralytics.data.converter import convert_coco

    args = parse_args()
    src_json = resolve_cli_path(args.json, must_exist=True)
    src_images = resolve_cli_path(args.images, must_exist=True)
    out_path = resolve_cli_path(args.out)

    print(f'COCO JSON   : {src_json}')
    print(f'COCO images : {src_images}')
    print(f'Output base : {out_path}')

    coco_info = split_coco(str(src_json), str(src_images), str(out_path), split=args.split, seed=args.seed)
    converted_root = out_path.parent / f'{out_path.name}_yolo'
    if converted_root.exists():
        shutil.rmtree(converted_root)
    convert_coco(labels_dir=str(out_path / 'annotations'), save_dir=str(converted_root), use_segments=True)
    shutil.copytree(out_path / 'images', converted_root / 'images', dirs_exist_ok=True)
    out_yaml = write_out_yaml(converted_root, coco_info)

    report = verify_yolo_dataset(converted_root)

    print('--- split summary ---')
    print(f"train json : {coco_info['train_json']}")
    print(f"val json   : {coco_info['val_json']}")
    print(f"data.yaml  : {out_yaml}")
    print('--- image match verify ---')
    print(json.dumps(coco_info['match_report'], indent=2, ensure_ascii=False))
    print('--- yolo verify ---')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()

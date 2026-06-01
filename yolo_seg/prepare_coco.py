import json
from pathlib import Path
import random
import shutil
import re


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
    annot_dir.mkdir(parents=True, exist_ok=True)
    images_train_dir = out_dir / 'images' / 'train'
    images_val_dir = out_dir / 'images' / 'val'
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

    def normalize_name(name: str) -> str:
        return re.sub(r'[^a-zA-Z0-9]+', '', name).lower()

    source_index = {}
    for image_path in src_images.iterdir():
        if image_path.is_file():
            source_index.setdefault(normalize_name(image_path.name), image_path)
            source_index.setdefault(normalize_name(image_path.stem), image_path)

    # copy image files referenced in train/val
    def copy_images(img_list, dst_dir):
        for im in img_list:
            fname = im.get('file_name')
            candidate_names = [fname]
            if '-' in fname:
                candidate_names.append(fname.split('-', 1)[1])
            candidate_names.append(Path(fname).name)

            src = None
            for candidate_name in candidate_names:
                direct = src_images / candidate_name
                if direct.exists():
                    src = direct
                    break
                src = source_index.get(normalize_name(candidate_name))
                if src is not None:
                    break

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
    }

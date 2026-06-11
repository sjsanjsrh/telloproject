from ultralytics import YOLO
import argparse
from pathlib import Path
import sys

# local preprocessing module
CURRENT_DIR = Path(__file__).resolve().parent
REPO_ROOT = CURRENT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from yolo_seg.prepare_coco import split_label_studio_masks

ARGS_YAML = CURRENT_DIR / 'args.yaml'


def resolve_cli_path(path_text: str, must_exist: bool = False) -> Path:
    candidate = Path(path_text)
    if candidate.is_absolute():
        return candidate

    for base_dir in (REPO_ROOT, CURRENT_DIR):
        resolved = base_dir / candidate
        if not must_exist or resolved.exists():
            return resolved

    return REPO_ROOT / candidate


def resolve_output_path(path_text: str) -> Path:
    candidate = Path(path_text)
    if candidate.is_absolute():
        return candidate

    for base_dir in (CURRENT_DIR, REPO_ROOT):
        resolved = base_dir / candidate
        if resolved.parent.exists() or resolved.parent == base_dir:
            return resolved

    return CURRENT_DIR / candidate


def find_label_studio_json(image_root: Path) -> Path | None:
    candidates = []
    for base_dir in (image_root, CURRENT_DIR / 'data', CURRENT_DIR):
        if base_dir.exists():
            candidates.extend(sorted(base_dir.glob('*.json'), key=lambda path: path.stat().st_mtime, reverse=True))

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--args-yaml', default=str(ARGS_YAML), help='YOLO hyperparameter YAML path')
    p.add_argument('--model', default='yolo26n-seg.pt', help='initial YOLO model/weights path')
    p.add_argument('--data', default='', help='Label Studio JSON or ready Ultralytics data.yaml path. Leave empty to auto-detect a JSON export from the images root.')
    p.add_argument('--images', default='yolo_seg/data', help='source image root for Label Studio local-files paths')
    p.add_argument('--out', default='yolo_seg/res/data_masks', help='output folder for mask PNG/NumPy dataset')
    p.add_argument('--split', type=float, default=0.8, help='train split ratio when preparing masks')
    p.add_argument('--seed', type=int, default=42, help='random seed when preparing masks')
    p.add_argument('--fill-label', action='append', default=['full_gate'], help='label whose mask holes should be filled. Can be repeated')
    p.add_argument('--no-fill-full-gate', action='store_true', help='do not fill holes for full_gate')
    p.add_argument('--min-component-area-ratio', type=float, default=0.0005, help='remove mask components smaller than this fraction of image area')
    p.add_argument('--epochs', type=int, default=None)
    p.add_argument('--imgsz', type=int, default=None)
    p.add_argument('--batch', type=int, default=None)
    p.add_argument('--device', default=None, help='training device, e.g. cpu or cuda:0')
    p.add_argument('--workers', type=int, default=None, help='number of dataloader worker processes; omit to use Ultralytics default')
    return p.parse_args()


def main():
    args = parse_args()
    if args.workers is not None and args.workers < 0:
        raise ValueError('--workers must be 0 or greater')

    args_yaml_path = resolve_cli_path(args.args_yaml, must_exist=False)
    images_path = resolve_cli_path(args.images, must_exist=True)
    data_source = resolve_cli_path(args.data, must_exist=True) if args.data else None
    if data_source is None:
        data_source = find_label_studio_json(images_path)
        if data_source is None:
            raise FileNotFoundError(f'No JSON export found. Looked in: {images_path}, {CURRENT_DIR / "data"}, {CURRENT_DIR}')

    if data_source.suffix.lower() == '.json':
        out_path = resolve_output_path(args.out)
        print('Preparing object-mask YOLO dataset: json=', data_source, ' images=', images_path, ' out=', out_path)
        fill_labels = set(args.fill_label)
        if args.no_fill_full_gate:
            fill_labels.discard('full_gate')
        info = split_label_studio_masks(
            data_source,
            images_path,
            out_path,
            split=args.split,
            seed=args.seed,
            fill_labels=fill_labels,
            min_component_area_ratio=args.min_component_area_ratio,
        )
        data_to_use = Path(info['data_yaml'])
    else:
        data_to_use = data_source

    # 모델 및 데이터 세팅
    model = YOLO(args.model)

    # 학습 시작
    train_kwargs = dict(
        cfg=str(args_yaml_path),
        data=str(data_to_use),
        project=str(CURRENT_DIR / 'res' / 'runs'),
    )
    if args.epochs is not None:
        train_kwargs['epochs'] = args.epochs
    if args.imgsz is not None:
        train_kwargs['imgsz'] = args.imgsz
    if args.batch is not None:
        train_kwargs['batch'] = args.batch
    if args.device is not None:
        train_kwargs['device'] = args.device
    if args.workers is not None:
        train_kwargs['workers'] = args.workers

    print(f'Using args yaml: {args_yaml_path}')
    print(f'Using model: {args.model}')
    print(f'Using data: {data_to_use}')
    model.train(**train_kwargs)


if __name__ == '__main__':
    main()

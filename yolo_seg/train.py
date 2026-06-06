from ultralytics import YOLO
from ultralytics.data.converter import convert_coco
import argparse
from pathlib import Path
import sys
import shutil

# local preprocessing module
CURRENT_DIR = Path(__file__).resolve().parent
REPO_ROOT = CURRENT_DIR.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from yolo_seg.prepare_coco import split_coco

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


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--args-yaml', default=str(ARGS_YAML), help='YOLO hyperparameter YAML path')
    p.add_argument('--model', default='yolo26n-seg.pt', help='initial YOLO model/weights path')
    p.add_argument('--data', default='yolo_seg/data/result_coco.json', help='data path (COCO JSON or Ultralytics data.yaml)')
    p.add_argument('--preprocess', action='store_true', help='Run random 4:3 crop preprocessing before training')
    p.add_argument('--images', default='data/img', help='source images folder for preprocessing')
    p.add_argument('--labels', default='data/labels', help='source labels folder for preprocessing (YOLO txt). If missing, images only')
    p.add_argument('--out', default='yolo_seg/res/data_cropped', help='output folder for preprocessed data')
    p.add_argument('--min-scale', type=float, default=0.6, help='min crop scale for preprocessing')
    p.add_argument('--trials', type=int, default=1, help='number of crops per image')
    p.add_argument('--epochs', type=int, default=None)
    p.add_argument('--imgsz', type=int, default=None)
    p.add_argument('--batch', type=int, default=None)
    p.add_argument('--device', default=None, help='training device, e.g. cpu or cuda:0')
    p.add_argument('--workers', type=int, default=None, help='number of dataloader worker processes; omit to use Ultralytics default')
    return p.parse_args()


def read_basic_yaml(yaml_path: Path):
    # very light-weight reader: extract nc and names if present
    res = {}
    if not yaml_path.exists():
        return res
    for line in yaml_path.read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if line.startswith('nc:'):
            try:
                res['nc'] = int(line.split(':', 1)[1].strip())
            except Exception:
                pass
        if line.startswith('names:'):
            # assume rest of line is list (e.g. names: ['a','b']) or a yaml block follows
            rest = line.split(':', 1)[1].strip()
            if rest.startswith('['):
                try:
                    res['names'] = eval(rest)
                except Exception:
                    res['names'] = []
    return res


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


def main():
    args = parse_args()
    if args.workers is not None and args.workers < 0:
        raise ValueError('--workers must be 0 or greater')

    cwd = REPO_ROOT
    args_yaml_path = resolve_cli_path(args.args_yaml, must_exist=False)
    data_source = resolve_cli_path(args.data, must_exist=True)

    # preprocess when explicitly requested or when the input is a COCO JSON file
    if args.preprocess or data_source.suffix.lower() == '.json':
        src_json = data_source
        images_path = resolve_cli_path(args.images, must_exist=True)
        out_path = resolve_output_path(args.out)
        print('Running COCO split preprocessing: json=', src_json, ' images=', images_path, ' out=', out_path)
        coco_info = split_coco(str(src_json), str(images_path), str(out_path), split=0.8)
        converted_root = out_path.parent / f'{out_path.name}_yolo'
        if converted_root.exists():
            shutil.rmtree(converted_root)
        convert_coco(labels_dir=str(out_path / 'annotations'), save_dir=str(converted_root), use_segments=True)
        shutil.copytree(out_path / 'images', converted_root / 'images', dirs_exist_ok=True)
        out_yaml = write_out_yaml(converted_root, coco_info)
        data_to_use = out_yaml
    else:
        # assume args.data is already a ready-to-use Ultralytics YAML
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

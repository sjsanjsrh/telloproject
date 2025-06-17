import cv2
import numpy as np
import glob
import yaml
import os

# 캘리브레이션용 체커보드 패턴 크기 (가로, 세로 내부 코너 수)
CHECKERBOARD = (9, 6)

# 객체 포인트 준비 (예: (0,0,0), (1,0,0), ... (8,5,0))
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

obj_points = []  # 3D 점
img_points = []  # 2D 점

# 이미지 경로
images = glob.glob("calib_images/*.png")  # 이미지 폴더가 현재 작업 폴더에 있어야 함

print(f"📸 이미지 개수: {len(images)}")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        obj_points.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        img_points.append(corners2)
    else:
        print(f"❌ 체커보드 코너를 찾을 수 없습니다: {fname}")

if len(obj_points) == 0:
    raise Exception("❌ 유효한 코너가 감지되지 않았습니다. 이미지들을 다시 확인하세요.")

# 카메라 캘리브레이션 수행
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

if ret:
    print("\n✅ 캘리브레이션 성공!")
    print("\n카메라 행렬 (camera matrix):")
    print(mtx)
    print("\n왜곡 계수 (distortion coefficients):")
    print(dist)

    calib_result = {
        "image_width": gray.shape[1],
        "image_height": gray.shape[0],
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": mtx.flatten().tolist()
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": dist.flatten().tolist()
        }
    }

    # 정확한 경로로 저장 (pytorch_mpiigaze_demo/ptgaze/data/calib)
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..","ptgaze", "data", "calib"))
    os.makedirs(base_dir, exist_ok=True)
    save_path = os.path.join(base_dir, "camera_params.yaml")

    with open(save_path, "w") as f:
        yaml.dump(calib_result, f)

    print(f"\n📁 캘리브레이션 결과 저장 완료: {save_path}")
else:
    print("❌ 캘리브레이션 실패")
import cv2
import os

# 저장할 디렉토리
save_dir = "./calib_images"
os.makedirs(save_dir, exist_ok=True)

# 카메라 열기
cap = cv2.VideoCapture(0)  # 0번 인덱스 카메라 열기

if not cap.isOpened():
    raise Exception("❌ 카메라를 열 수 없습니다.")
else:
    print("✅ 카메라가 성공적으로 열렸습니다!")

print("▶ 's' 키: 이미지 저장 | 'q' 키: 종료")

i = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 프레임을 읽을 수 없습니다.")
        break

    # 프레임을 화면에 표시
    cv2.imshow("Capture", frame)

    # 키 입력 대기
    key = cv2.waitKey(1)
    if key == ord('s'):
        filename = os.path.join(save_dir, f"image_{i:02d}.png")
        cv2.imwrite(filename, frame)
        print(f"💾 저장됨: {filename}")
        i += 1
    elif key == ord('q'):
        break

# 카메라 및 윈도우 자원 해제
cap.release()
cv2.destroyAllWindows()
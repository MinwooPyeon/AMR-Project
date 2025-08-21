import cv2
import numpy as np

def rotate_image(image, angle):
    # 이미지 중심 좌표
    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)

    # 회전 행렬 계산
    M = cv2.getRotationMatrix2D(center, angle, 1.0)

    # 회전 적용 (경계 포함)
    rotated = cv2.warpAffine(image, M, (w, h), flags=cv2.INTER_NEAREST, borderValue=205)
    return rotated

def load_map_as_grid(pgm_path):
    # 맵 이미지 로드 (흑백)
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)

    if img is None:
        raise FileNotFoundError(f"맵 파일을 찾을 수 없습니다: {pgm_path}")
    rotated = rotate_image(img, -30)
    cv2.imshow('PGM Image', rotated)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # OpenCV는 0 (검정) ~ 255 (흰색)
    # SLAM에서는 일반적으로:
    #   - 0: 점유 (검정, obstacle)
    #   - 205~255: 자유 공간 (흰색)
    #   - 127~: 미확인 (회색)

    grid = np.zeros_like(img, dtype=np.uint8)

    grid[img <= 50] = 2        # 점유 (검정)
    grid[img >= 200] = 1       # 자유 (흰색)
    grid[(img > 50) & (img < 200)] = 0  # 미확인 (회색)

    return grid

grids = load_map_as_grid("./test.pgm")
print(grids.size)


'''
1. slam pgm file 불러오기
2. map 알맞은 형태로 각도 조절
3. map에서 움직일 수 있는 부분만 범위 체크
4. 맵 크기, AMR 크기에 맞춰서 map을 grid화해서 배열로 저장
5. 실제 map의 좌표와 배열의 grid를 1대1 mapping
6. 장애물이 있는지 없는지 grid들을 구분
7. 구분된 map에서 최소한의 회전으로 현재 위치에서 목적지까지 갈 수 있는 경로 A* 알고리즘으로 찾기
왼쪽 위 : (-5, -5) 오른쪽 위 : (-5, 5)
왼쪽 밑 : (5,  -5) 오른쪽 밑 : ( 5, 5)
'''
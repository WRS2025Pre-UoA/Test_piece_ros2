import cv2
import numpy as np

def dist(p1, p2):
    """
    2点間のユークリッド距離を計算する関数。
    """
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def perspective_transform(img, points):
    """
    4つの点を基に画像の射影変換を行う関数。
    """
    lengths = [dist(points[i], points[(i+1) % 4]) for i in range(4)]
    max_length = int(max(lengths))
    
    square = np.array([[0, 0], [max_length, 0], 
                       [max_length, max_length], [0, max_length]], dtype="float32")
    
    M = cv2.getPerspectiveTransform(points, square)
    output_size = (max_length, max_length)
    warped = cv2.warpPerspective(img, M, output_size)
    
    return warped

def detect_and_measure_lines(img, canny_thresh1, canny_thresh2, max_line_gap, scale=20):
    """
    画像内の黒い直線を検出し、cm単位で長さを計算する関数。
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, canny_thresh1, canny_thresh2, apertureSize=3)

    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=100, minLineLength=30, maxLineGap=max_line_gap)
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)  # 検出した直線を赤色で表示
            length = dist((x1, y1), (x2, y2))
            length_cm = (length / img.shape[1]) * scale
            cv2.putText(img, f"Length: {length_cm:.2f} cm", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
            print(f"Line length in cm: {length_cm:.2f}")

    return img

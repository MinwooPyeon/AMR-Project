import cv2
import time
import torch
import numpy as np
from ultralytics import YOLO
from torch_geometric.data import Data
from torch_geometric.nn import GCNConv, global_mean_pool
import torch.nn.functional as F
import torch.nn as nn

# -----------------------------
# 설정
# -----------------------------
NUM_CLASSES = 11  # YOLO 클래스 개수
CLASS_NAMES = [
    'FLAMMABLE_MATERIAL', 'FOREIGN_OBJECT_WET', 'HANDLING_EQUIPMENT',
    'IMPROPER_GEAR', 'INDIVIDUAL_MATERIAL', 'MATERIAL_COLLAPSE',
    'NOT_WEAR_WORKER', 'SMOKING_ACTIVITY', 'SMOKING_VIOLATION',
    'STACKED_MATERIAL', 'WEAR_WORKER'
]

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"사용 디바이스: {device}")

# 상황 레이블 (GCN 학습 시 사용한 기준과 동일)
SITUATION_LABELS = ["Normal", "Accident-1", "Accident-2", "Accident-3"]

# -----------------------------
# GCN 모델 정의
# -----------------------------
class SituationGCN(nn.Module):
    def __init__(self, in_channels, hidden_channels, num_classes):
        super(SituationGCN, self).__init__()
        self.conv1 = GCNConv(in_channels, hidden_channels)
        self.conv2 = GCNConv(hidden_channels, hidden_channels)
        self.fc = nn.Linear(hidden_channels, num_classes)

    def forward(self, x, edge_index, batch):
        x = self.conv1(x, edge_index)
        x = F.relu(x)
        x = self.conv2(x, edge_index)
        x = F.relu(x)
        x = global_mean_pool(x, batch)
        return self.fc(x)

# -----------------------------
# GCN 모델 로드
# -----------------------------
in_channels = NUM_CLASSES + 4  # One-Hot + bbox
gcn_model = SituationGCN(in_channels=in_channels, hidden_channels=64, num_classes=4).to(device)
gcn_model.load_state_dict(torch.load("best_model.pth", map_location=device))
gcn_model.eval()

# -----------------------------
# YOLO 모델 로드
# -----------------------------
yolo_model = YOLO("best.pt").to(device)

# -----------------------------
# YOLO 결과 → GCN 입력 생성 함수
# -----------------------------
def create_graph_from_yolo_results(results):
    classes = []
    coords = []
    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls_id = int(box.cls[0])
            classes.append(cls_id)
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = xyxy
            cx = ((x1 + x2) / 2) / r.orig_shape[1]
            cy = ((y1 + y2) / 2) / r.orig_shape[0]
            w = (x2 - x1) / r.orig_shape[1]
            h = (y2 - y1) / r.orig_shape[0]
            coords.append([cx, cy, w, h])

    # 객체 없음
    if len(classes) == 0:
        return None

    # 객체가 1개뿐 → edge 없음 → 상황 Normal 처리
    if len(classes) == 1:
        return "SINGLE_NODE"

    classes = np.array(classes)
    coords = np.array(coords)

    # 노드 특징 = 클래스 One-Hot + bbox
    enc = np.eye(NUM_CLASSES)[classes]
    node_features = np.hstack([enc, coords])
    x = torch.tensor(node_features, dtype=torch.float)

    # Edge 생성 (fully connected + 거리)
    edge_index = []
    edge_attr = []
    num_nodes = len(classes)
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                edge_index.append([i, j])
                dist = np.sqrt((coords[i][0] - coords[j][0]) ** 2 + (coords[i][1] - coords[j][1]) ** 2)
                edge_attr.append([dist])

    if len(edge_index) == 0:
        return "NO_EDGE"

    edge_index = torch.tensor(edge_index, dtype=torch.long).t().contiguous()
    edge_attr = torch.tensor(edge_attr, dtype=torch.float)

    return Data(x=x, edge_index=edge_index, edge_attr=edge_attr)

# -----------------------------
# 메인 함수
# -----------------------------
def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("웹캠을 열 수 없습니다.")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # YOLO 탐지
        results = yolo_model(frame, stream=False)

        # GCN 입력 생성
        graph_data = create_graph_from_yolo_results(results)
        situation_text = "Normal"  # 기본값

        if graph_data is None or graph_data == "SINGLE_NODE" or graph_data == "NO_EDGE":
            situation_text = "Normal"
        else:
            graph_data = graph_data.to(device)
            batch = torch.zeros(graph_data.num_nodes, dtype=torch.long).to(device)
            with torch.no_grad():
                out = gcn_model(graph_data.x, graph_data.edge_index, batch)
                pred = out.argmax(dim=1).item()
                situation_text = SITUATION_LABELS[pred]

        # YOLO 탐지 박스 & 레이블 표시
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = yolo_model.names[class_id]
                color = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                label = f"{class_name} {confidence:.2f}"
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # 상황 인식 결과 화면 표시
        cv2.putText(frame, f"Situation: {situation_text}", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.imshow("Webcam - YOLO + GCN", frame)

        # 'q' 눌러 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

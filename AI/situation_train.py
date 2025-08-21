import os
import torch
import numpy as np
from torch_geometric.data import Data, InMemoryDataset, DataLoader
from torch_geometric.nn import GCNConv, global_mean_pool
import torch.nn.functional as F
import torch.nn as nn
from sklearn.model_selection import train_test_split
from tqdm import tqdm

# -----------------------------
# 설정
# -----------------------------
NUM_CLASSES = 11
IMG_W, IMG_H = 1920, 1080
CLASS_NAMES = [
    'FLAMMABLE_MATERIAL', 'FOREIGN_OBJECT_WET', 'HANDLING_EQUIPMENT',
    'IMPROPER_GEAR', 'INDIVIDUAL_MATERIAL', 'MATERIAL_COLLAPSE',
    'NOT_WEAR_WORKER', 'SMOKING_ACTIVITY', 'SMOKING_VIOLATION',
    'STACKED_MATERIAL', 'WEAR_WORKER'
]

# situation_id → 제외할 클래스 매핑
SITUATION_TO_REMOVE_CLASSES = {
    "UA-06": [5],       # MATERIAL_COLLAPSE
    "UA-07": [3],    # SMOKING_ACTIVITY, SMOKING_VIOLATION
    "UA-08": [7,8]        # NOT_WEAR_WORKER
}

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"사용 중인 디바이스: {device}")

# 상황 라벨 매핑
def map_situation_id(situation_id):
    if situation_id == 'UA-06':
        return 1
    elif situation_id == 'UA-07':
        return 2
    elif situation_id == 'UA-08':
        return 3
    else:
        return 0  # 정상

# -----------------------------
# YOLO txt → Graph 변환
# -----------------------------
def parse_yolo_txt(file_path, situation_label, situation_id):
    with open(file_path, 'r') as f:
        lines = f.readlines()

    classes = []
    coords = []

    for line in lines:
        vals = line.strip().split()
        cls_id = int(vals[0])
        cx, cy, w, h = map(float, vals[1:])
        classes.append(cls_id)
        coords.append([cx, cy, w, h])

    classes = np.array(classes)
    coords = np.array(coords)

    # 제외할 클래스
    remove_classes = SITUATION_TO_REMOVE_CLASSES.get(situation_id, [])
    keep_idx = [i for i, cls_id in enumerate(classes) if cls_id not in remove_classes]

    classes = classes[keep_idx]
    coords = coords[keep_idx]

    if len(classes) == 0:
        # 모든 객체가 제거되면 최소한 1개 노드 유지(더미 노드 추가)
        coords = np.array([[0.5, 0.5, 0.1, 0.1]])
        classes = np.array([0])  # FLAMMABLE_MATERIAL 더미
    # One-Hot + bbox
    enc = np.eye(NUM_CLASSES)[classes]
    node_features = np.hstack([enc, coords])
    x = torch.tensor(node_features, dtype=torch.float)

    # Edge 생성
    num_nodes = len(classes)
    edge_index = []
    edge_attr = []
    for i in range(num_nodes):
        for j in range(num_nodes):
            if i != j:
                edge_index.append([i, j])
                dist = np.sqrt((coords[i][0] - coords[j][0]) ** 2 +
                               (coords[i][1] - coords[j][1]) ** 2)
                edge_attr.append([dist])

    edge_index = torch.tensor(edge_index, dtype=torch.long).t().contiguous()
    edge_attr = torch.tensor(edge_attr, dtype=torch.float)

    y = torch.tensor([situation_label], dtype=torch.long)

    return Data(x=x, edge_index=edge_index, edge_attr=edge_attr, y=y)

# -----------------------------
# Dataset 클래스
# -----------------------------
class YoloGraphDataset(InMemoryDataset):
    def __init__(self, root, file_list, transform=None, pre_transform=None):
        self.file_list = file_list
        super().__init__(root, transform, pre_transform)
        self.data, self.slices = self.collate(self._load_data())

    def _load_data(self):
        data_list = []
        for file_path, situation_id in self.file_list:
            label = map_situation_id(situation_id)
            graph = parse_yolo_txt(file_path, label, situation_id)
            data_list.append(graph)
        return data_list

# -----------------------------
# 모델 정의
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
# 학습 함수
# -----------------------------
def train_epoch(model, loader, optimizer, criterion):
    model.train()
    total_loss = 0
    correct = 0
    total = 0

    for batch in tqdm(loader, desc="Training", leave=False):
        batch = batch.to(device)
        optimizer.zero_grad()
        out = model(batch.x, batch.edge_index, batch.batch)
        loss = criterion(out, batch.y)
        loss.backward()
        optimizer.step()

        total_loss += loss.item()
        preds = out.argmax(dim=1)
        correct += (preds == batch.y).sum().item()
        total += batch.y.size(0)

    acc = correct / total
    return total_loss / len(loader), acc

def validate_epoch(model, loader, criterion):
    model.eval()
    total_loss = 0
    correct = 0
    total = 0

    with torch.no_grad():
        for batch in tqdm(loader, desc="Validating", leave=False):
            batch = batch.to(device)
            out = model(batch.x, batch.edge_index, batch.batch)
            loss = criterion(out, batch.y)

            total_loss += loss.item()
            preds = out.argmax(dim=1)
            correct += (preds == batch.y).sum().item()
            total += batch.y.size(0)

    acc = correct / total
    return total_loss / len(loader), acc

# -----------------------------
# 실행
# -----------------------------
if __name__ == "__main__":
    train_labels_dir = r"C:\ssafy\AIoT-AI\S13P11D103\AI\yolo_dataset\labels\train"

    # 파일 리스트 생성
    file_list = []
    for filename in os.listdir(train_labels_dir):
        if filename.endswith(".txt"):
            file_path = os.path.join(train_labels_dir, filename)
            parts = filename.split("_")
            situation_id = parts[3]  # 예: UA-06
            file_list.append((file_path, situation_id))

    print(f"총 라벨 파일 개수: {len(file_list)}")

    # Train/Validation Split
    train_files, val_files = train_test_split(file_list, test_size=0.2, random_state=42)

    train_dataset = YoloGraphDataset(root=".", file_list=train_files)
    val_dataset = YoloGraphDataset(root=".", file_list=val_files)

    train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=64, shuffle=False)

    # 모델 생성
    in_channels = NUM_CLASSES + 4  # One-Hot + bbox
    model = SituationGCN(in_channels=in_channels, hidden_channels=64, num_classes=4).to(device)

    optimizer = torch.optim.Adam(model.parameters(), lr=0.01)
    criterion = nn.CrossEntropyLoss()

    best_val_acc = 0.0
    num_epochs = 50

    for epoch in range(1, num_epochs + 1):
        print(f"\nEpoch {epoch}/{num_epochs}")
        train_loss, train_acc = train_epoch(model, train_loader, optimizer, criterion)
        val_loss, val_acc = validate_epoch(model, val_loader, criterion)

        print(f"Train Loss: {train_loss:.4f}, Train Acc: {train_acc:.4f}")
        print(f"Val   Loss: {val_loss:.4f}, Val   Acc: {val_acc:.4f}")

        # Best 모델 저장
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            torch.save(model.state_dict(), "best_model.pth")
            print("✅ Best model updated and saved.")

import torch.nn as nn
from torch_geometric.nn import GCNConv, global_mean_pool

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
        x = global_mean_pool(x, batch)  # 그래프 단위 Pooling
        out = self.fc(x)
        return out
    
from torch_geometric.loader import DataLoader

# 예시: 단일 그래프 → 배치 처리 위해 리스트화
dataset = [graph_data]  # 실제는 여러 JSON → 여러 그래프
loader = DataLoader(dataset, batch_size=1, shuffle=True)

model = SituationGCN(in_channels=x.size(1), hidden_channels=64, num_classes=4)
optimizer = torch.optim.Adam(model.parameters(), lr=0.01)
criterion = nn.CrossEntropyLoss()

for epoch in range(50):
    for batch in loader:
        optimizer.zero_grad()
        out = model(batch.x, batch.edge_index, batch.batch)
        loss = criterion(out, batch.y)
        loss.backward()
        optimizer.step()
    if epoch % 10 == 0:
        print(f"Epoch {epoch}, Loss: {loss.item():.4f}")

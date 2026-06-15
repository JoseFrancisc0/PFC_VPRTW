import torch
import torch.nn as nn
import torch.nn.functional as F

class ALNS_DQN(nn.Module):
    def __init__(self, state_dim=105, num_actions=20):
        super(ALNS_DQN, self).__init__()
        # Arquitectura MLP para extraer características del estado
        self.fc1 = nn.Linear(state_dim, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, num_actions)

    def forward(self, x):
        # x shape esperado: (batch_size, state_dim)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        q_values = self.fc3(x)
        return q_values

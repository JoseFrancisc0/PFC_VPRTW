import os
import glob
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import pandas as pd
import numpy as np
from dqn_model import ALNS_DQN
import time

# Hiperparametros Ajustados
BATCH_SIZE = 1024
GAMMA = 0.99
LEARNING_RATE = 1e-3
EPOCHS = 20
SAMPLES_PER_FILE = 8000 # Carga hasta 8000 por archivo (si hay 56 inst * 3 runs = 168 archivos -> ~1.3M de muestras)

def load_offline_dataset(data_dir):
    print(f"Buscando archivos CSV en: {data_dir}")
    all_files = glob.glob(os.path.join(data_dir, "*.csv"))
    if not all_files:
        raise FileNotFoundError("No se encontraron archivos CSV de experiencias.")
    
    print(f"Se encontraron {len(all_files)} archivos. Cargando datos...")
    
    states_list, actions_list, rewards_list, next_states_list = [], [], [], []
    
    for f in all_files:
        try:
            df = pd.read_csv(f)
            if len(df) > SAMPLES_PER_FILE:
                df = df.sample(n=SAMPLES_PER_FILE, random_state=42)
            
            if df.shape[1] < 212:
                continue
                
            S = df.iloc[:, 0:105].values.astype(np.float32)
            A = df['action'].values.astype(np.int64)
            R = df['reward'].values.astype(np.float32)
            S_next = df.iloc[:, 107:212].values.astype(np.float32)
            
            states_list.append(S)
            actions_list.append(A)
            rewards_list.append(R)
            next_states_list.append(S_next)
            
        except Exception as e:
            pass
            
    print("Concatenando matrices de memoria...")
    all_states = np.concatenate(states_list, axis=0)
    all_actions = np.concatenate(actions_list, axis=0)
    all_rewards = np.concatenate(rewards_list, axis=0)
    all_next_states = np.concatenate(next_states_list, axis=0)
    
    print(f"Dataset final consolidado con {len(all_states)} transiciones.")
    
    dataset = TensorDataset(
        torch.tensor(all_states),
        torch.tensor(all_actions),
        torch.tensor(all_rewards),
        torch.tensor(all_next_states)
    )
    
    return DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True)

def train_offline_rl():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Utilizando dispositivo: {device}")
    
    data_dir = os.path.join("..", "Results", "CLASSIC", "experiences")
    dataloader = load_offline_dataset(data_dir)
    
    policy_net = ALNS_DQN(state_dim=105, num_actions=20).to(device)
    target_net = ALNS_DQN(state_dim=105, num_actions=20).to(device)
    target_net.load_state_dict(policy_net.state_dict())
    target_net.eval()
    
    optimizer = optim.Adam(policy_net.parameters(), lr=LEARNING_RATE)
    criterion = nn.MSELoss()
    
    # Scheduler: Reduce el LR si la métrica (loss) se estanca por 3 epochs seguidos
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=3, verbose=True)
    
    print("\n--- INICIANDO ENTRENAMIENTO OFFLINE AVANZADO ---")
    start_time = time.time()
    
    best_loss = float('inf')
    best_model_state = None
    
    for epoch in range(EPOCHS):
        policy_net.train()
        total_loss = 0.0
        
        for batch_idx, (states, actions, rewards, next_states) in enumerate(dataloader):
            states, actions = states.to(device), actions.to(device)
            rewards, next_states = rewards.to(device), next_states.to(device)
            
            q_values = policy_net(states)
            current_q = q_values.gather(1, actions.unsqueeze(1)).squeeze(1)
            
            with torch.no_grad():
                next_q_values = target_net(next_states)
                max_next_q = next_q_values.max(1)[0]
                target_q = rewards + GAMMA * max_next_q
                
            loss = criterion(current_q, target_q)
            
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item()
            
        avg_loss = total_loss / len(dataloader)
        current_lr = optimizer.param_groups[0]['lr']
        print(f"Epoch [{epoch+1}/{EPOCHS}] | Loss: {avg_loss:.4f} | LR: {current_lr}")
        
        # Checkpointing
        if avg_loss < best_loss:
            best_loss = avg_loss
            best_model_state = policy_net.state_dict()
            print(f"  -> ¡Nuevo mejor modelo encontrado! (Loss: {best_loss:.4f})")
            
        scheduler.step(avg_loss)
        
        # Soft Update periodico (o Hard Update cada N epochs)
        # Haremos Hard Update cada 2 epochs
        if (epoch + 1) % 2 == 0:
            target_net.load_state_dict(policy_net.state_dict())
        
    end_time = time.time()
    print(f"\nEntrenamiento completado en {end_time - start_time:.2f} segundos.")
    
    print("Cargando el mejor modelo encontrado para la exportación...")
    if best_model_state is not None:
        policy_net.load_state_dict(best_model_state)
    
    print("Exportando modelo Experto a TorchScript (C++)...")
    policy_net.eval()
    policy_net.to("cpu")
    example_input_cpu = torch.rand(1, 105)
    traced_script_module = torch.jit.trace(policy_net, example_input_cpu)
    
    output_path = os.path.join("..", "alns_dqn_expert.pt")
    traced_script_module.save(output_path)
    print(f"Modelo Experto exportado exitosamente a: {output_path}")

if __name__ == "__main__":
    train_offline_rl()

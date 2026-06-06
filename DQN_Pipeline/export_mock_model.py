import torch
from dqn_model import ALNS_DQN

def create_mock_model():
    print("Creando modelo DQN Dummy...")
    model = ALNS_DQN(state_dim=105, num_actions=20)
    model.eval()
    
    # Entrada de ejemplo (batch_size=1, state_dim=105)
    example_input = torch.rand(1, 105)
    
    # Trace the model
    traced_script_module = torch.jit.trace(model, example_input)
    
    # Save the model en la carpeta base del proyecto
    model_path = "../alns_dqn_expert.pt"
    traced_script_module.save(model_path)
    print(f"Mock model guardado exitosamente en: {model_path}")

if __name__ == "__main__":
    create_mock_model()

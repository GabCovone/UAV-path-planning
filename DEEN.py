import torch
import torch.nn as nn
import torch.optim as optim
import h5py
from torch.utils.data import Dataset, DataLoader
import os # Importato per gestire il salvataggio dei file

#stato = voxel: 1000, velocità lineare: 3, velocità angolare: 3, orientamento: 4 utilizzo dei quaternioni per efficienza
input_dim = 2024 #(1010 stato attuale + 4 motori + 1010 stato successivo)

# 1. DEEN
class DEEN_Network(nn.Module):
    def __init__(self, input_dim, dropout_rate=0.2):
        super().__init__()
        
        self.net = nn.Sequential(
            nn.Linear(input_dim, 1024),
            nn.LayerNorm(1024),           # Ottimizzazione 1: Stabilizza i gradienti
            nn.SiLU(),                   # Ottimizzazione 2: Attivazione fluida (Swish)
            nn.Dropout(dropout_rate),    # Ottimizzazione 3: Previene l'overfitting sui voxel
            
            nn.Linear(1024, 1024),
            nn.LayerNorm(1024),
            nn.SiLU(),
            nn.Dropout(dropout_rate),
            
            nn.Linear(1024, 512),
            nn.LayerNorm(512),
            nn.SiLU(),
            nn.Dropout(dropout_rate),
            
            nn.Linear(512, 128),
            nn.LayerNorm(128),
            nn.SiLU(),
            nn.Dropout(dropout_rate),

            nn.Linear(128, 1)            # Output: Singolo scalare (Energia)
        )

    def forward(self, x):
        return self.net(x)

# 2. LOSS FUNCTION
def deen_loss_function(ebm_model, x_clean, sigma=0.1):
    # Aggiungo rumore
    noise = torch.randn_like(x_clean) * sigma
    
    y_corrupted = (x_clean + noise).detach().requires_grad_(True)
    
    # Calcolo Energia
    energy = ebm_model(y_corrupted)
    
    # Calcolo Gradienti (Score Function)
    score_function = torch.autograd.grad(
        outputs=energy.sum(),
        inputs=y_corrupted,
        create_graph=True, # FONDAMENTALE per la backpropagation successiva
        retain_graph=True
    )[0]
    
    # E. Loss Function del paper (Denoising Score Matching)
    error = x_clean - y_corrupted + (sigma**2) * score_function
    loss = torch.mean(error**2)
    
    return loss

# 3. SETUP DI ADDESTRAMENTO
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = DEEN_Network(input_dim, dropout_rate=0.2).to(device)

optimizer = optim.AdamW(model.parameters(), lr=1e-3, weight_decay=1e-4)

scheduler = optim.lr_scheduler.ReduceLROnPlateau(
    optimizer, mode='min', factor=0.5, patience=5
)

# 4. IL LOOP DI ADDESTRAMENTO (CON CHECKPOINTING)
def train_deen(model, dataloader, num_epochs=100, sigma=0.1, save_path="best_deen_model.pth"):
    model.train() 
    
    # Inizializzo la loss migliore a infinito
    best_loss = float('inf')
    
    for epoch in range(num_epochs):
        epoch_loss = 0.0
        
        for batch_x_clean in dataloader:
            batch_x_clean = batch_x_clean.to(device)
            
            optimizer.zero_grad()
            
            loss = deen_loss_function(model, batch_x_clean, sigma)
            loss.backward()
            
            torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
            
            optimizer.step()
            epoch_loss += loss.item()
            
        avg_loss = epoch_loss / len(dataloader)
        
        # --- LOGICA DI SALVATAGGIO ---
        # Se la loss di questa epoca è migliore di quella storica, salva i pesi
        if avg_loss < best_loss:
            best_loss = avg_loss
            torch.save(model.state_dict(), save_path)
            salvato_msg = f" (Miglioramento! Modello salvato in {save_path})"
        else:
            salvato_msg = ""
            
        print(f"Epoca {epoch+1}/{num_epochs} - Loss: {avg_loss:.6f}{salvato_msg}")
        
        scheduler.step(avg_loss)
        
    print(f"Addestramento completato. Miglior Loss ottenuta: {best_loss:.6f}")
    
    # A fine addestramento, ricarico automaticamente i pesi migliori prima di proseguire
    model.load_state_dict(torch.load(save_path))
    print("Modello migliore caricato per l'esportazione.")
    return model

# 5. DATASET E CARICAMENTO
class DroneVoxelDataset(Dataset):
    def __init__(self, h5_filepath):
        print(f"Caricamento dataset da {h5_filepath}...")
        with h5py.File(h5_filepath, 'r') as f:
            # Ricorda di controllare che la matrice MATLAB sia effettivamente di dimensione (N, input_dim)
            dati_numpy = f['transitions'][:]

            if dati_numpy.shape[0] == 2024:
                dati_numpy = dati_numpy.T
            
        self.data = torch.tensor(dati_numpy, dtype=torch.float32)
        print(f"Dataset caricato! Totale campioni: {len(self.data)}")

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return self.data[idx]

# ESECUZIONE
dataset = DroneVoxelDataset('dataset_traiettorie_voxel.h5')
dataloader = DataLoader(dataset, batch_size=256, shuffle=True)
model = train_deen(model, dataloader, num_epochs=100, sigma=0.1, save_path="best_deen_weights.pth")

# 6. ESPORTAZIONE
def esporta_in_onnx(model, nome_file="deen_addestrato.onnx"):
    model.eval() 
    
    dummy_input = torch.randn(1, input_dim, requires_grad=True).to(device)
    
    print(f"Esportazione del modello in {nome_file}...")
    
    torch.onnx.export(
        model,                      
        dummy_input,                
        nome_file,                  
        export_params=True,         
        opset_version=11,           
        do_constant_folding=True,   
        input_names=['input_transition'], 
        output_names=['energy_score']
    )
    print("Esportazione completata con successo! Ora puoi importarlo in MATLAB.")

# --- ESPORTAZIONE (DA SCOMMENTARE) ---
# esporta_in_onnx(model, "deen_addestrato.onnx")
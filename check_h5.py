import h5py
import numpy as np

file_path = './../dataset_singola_citta.h5'

print(f"🔍 Apertura del file: {file_path}...")

with h5py.File(file_path, 'r') as f:
    # 1. Estrazione dei dati
    data = f['transitions'][:]
    
    # 2. Controllo Trasposizione (Il trucco MATLAB -> Python)
    if data.shape[0] == 2024:
        print("🔄 Rilevata matrice in stile MATLAB. Eseguo la trasposizione...")
        data = data.T
        
    print(f"✅ Forma finale del dataset: {data.shape} (Dovrebbe essere [N_campioni, 2024])")
    
    # 3. Suddivisione logica del vettore da 2024 (basata sul tuo script)
    # [1000 voxel_t, 10 din_t, 4 act_t, 1000 voxel_t1, 10 din_t1]
    voxels_t = data[:, 0:1000]
    dyn_t    = data[:, 1000:1010]
    act_t    = data[:, 1010:1014]
    
    print("\n--- 🔬 SANITY CHECK DELLE NORMALIZZAZIONI ---")
    
    # Voxel (devono essere solo 0 o 1)
    print(f"🧱 Voxel (min / max)   : {np.min(voxels_t)} / {np.max(voxels_t)} (Atteso: 0.0 / 1.0)")
    
    # Dinamica (devono essere tra -1 e 1)
    print(f"🚀 Dinamica (min / max): {np.min(dyn_t):.3f} / {np.max(dyn_t):.3f} (Atteso: tra -1.0 e 1.0)")
    
    # Motori (devono essere tra -1 e 1)
    print(f"⚙️ Motori (min / max)  : {np.min(act_t):.3f} / {np.max(act_t):.3f} (Atteso: tra -1.0 e 1.0)")
    
    # 4. Controllo Matematico sui Quaternioni
    # I quaternioni sono gli ultimi 4 valori della dinamica (indici da 6 a 10)
    quats = dyn_t[:, 6:10]
    norma_media = np.mean(np.linalg.norm(quats, axis=1))
    print(f"🧭 Norma Quaternioni   : {norma_media:.6f} (Atteso: ~1.000000)")
    
    # 5. Controllo Densità Ostacoli
    ostacoli_totali = np.sum(voxels_t == 1.0)
    percentuale = (ostacoli_totali / (data.shape[0] * 1000)) * 100
    print(f"\n📊 Statistiche Ostacoli:")
    print(f"   Voxel 'pieni' totali: {ostacoli_totali}")
    print(f"   Densità del dataset : {percentuale:.2f}% (Se è > 0%, il drone ha visto i palazzi!)")
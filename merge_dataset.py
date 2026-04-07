import h5py
import numpy as np
import glob
import os

file_output = "dataset_deen_completo.h5"

print("🔄 Inizio procedura di fusione dei dataset urbani...")

# FIX: Usiamo glob per trasformare il pattern in una vera lista di percorsi
percorso_ricerca = './../*.h5'
lista_file = sorted(glob.glob(percorso_ricerca))

# Controlliamo di non includere il file di output se esiste già
output_path = os.path.join('./..', file_output)
if output_path in lista_file:
    lista_file.remove(output_path)
elif file_output in lista_file: # Controllo di sicurezza extra
    lista_file.remove(file_output)

if len(lista_file) == 0:
    print(f"❌ Errore: Nessun file .h5 trovato in {percorso_ricerca}!")
    exit()

print(f"📂 Trovati {len(lista_file)} file da unire.")

tutte_le_transizioni = []
tutti_gli_id = [] 
totale_campioni = 0

for city_id, file_path in enumerate(lista_file, start=1):
    with h5py.File(file_path, 'r') as f:
        dati = f['transitions'][:]
        
        if dati.shape[0] == 2024:
            dati = dati.T
            
        campioni_file = dati.shape[0]
        
        # 1. Aggiungiamo i dati fisici
        tutte_le_transizioni.append(dati)
        
        # 2. Creiamo il vettore ID
        array_id_citta = np.full((campioni_file, 1), city_id, dtype=np.int32)
        tutti_gli_id.append(array_id_citta)
        
        totale_campioni += campioni_file
        print(f"  ✔️ {file_path} caricato ({campioni_file} righe) -> Assegnato ID: {city_id}")

print(f"\n🧩 Impilamento delle matrici in corso...")

dataset_completo = np.vstack(tutte_le_transizioni)
id_completo = np.vstack(tutti_gli_id) 

print(f"✅ Forma della matrice dati: {dataset_completo.shape}")
print(f"✅ Forma del vettore ID: {id_completo.shape}")

# =====================================================================
# SALVATAGGIO A DOPPIO BINARIO
# =====================================================================
print(f"💾 Creazione del file {file_output}...")
if os.path.exists(file_output):
    os.remove(file_output)

with h5py.File(file_output, 'w') as f:
    f.create_dataset('transitions', data=dataset_completo.astype(np.float32))
    f.create_dataset('city_ids', data=id_completo)

print("🎉 MERGE COMPLETATO CON ETICHETTE! Dataset pronto.")
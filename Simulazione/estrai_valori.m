% --- 1. ESTRAZIONE DELLA CINEMATICA (Forziamo il formato numerico puro 'double') ---
t  = double(squeeze(simout.pos_vel.x.Time));
x  = double(squeeze(simout.pos_vel.x.Data));
y  = double(squeeze(simout.pos_vel.y.Data));
z  = double(squeeze(simout.pos_vel.z.Data));
vx = double(squeeze(simout.pos_vel.vx.Data));
vy = double(squeeze(simout.pos_vel.vy.Data));
vz = double(squeeze(simout.pos_vel.vz.Data));

% --- 2. PREPARAZIONE DEI MOTORI (Estraiamo i numeri dalla timeseries) ---
% Usiamo .Data per tirare fuori i numeri dalla timeseries 'Motors'
if isa(Motors, 'timeseries')
    motori_raw = Motors.Data;
else
    motori_raw = Motors;
end

% Appiattiamo e capovolgiamo per avere N righe x 4 colonne
motori = double(squeeze(motori_raw))'; 

% Controllo di sicurezza sulle dimensioni: se è 4xN, lo giriamo
if size(motori, 2) ~= 4 && size(motori, 1) == 4
    motori = motori';
end

% --- 3. ALLINEAMENTO TEMPORALE E FUSIONE ---
N = min(length(t), size(motori, 1));
dataset_finale = [t(1:N), x(1:N), y(1:N), z(1:N), vx(1:N), vy(1:N), vz(1:N), motori(1:N, :)];

% --- 4. SALVATAGGIO IN CSV ---
% Creiamo i nomi delle colonne
nomi_colonne = {'Tempo', 'X', 'Y', 'Z', 'Vx', 'Vy', 'Vz', 'Motore1', 'Motore2', 'Motore3', 'Motore4'};

% Trasformiamo la matrice in una tabella
tabella_dataset = array2table(dataset_finale, 'VariableNames', nomi_colonne);

% Salviamo il nuovo file CSV con le intestazioni
writetable(tabella_dataset, 'dataset_DEEN_MinimumSnap_completo.csv');
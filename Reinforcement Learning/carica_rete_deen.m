function net = carica_rete_deen()
    % La variabile 'persistent' fa sì che ogni worker di MATLAB carichi 
    % il file dal disco fisso UNA SOLA VOLTA al primo avvio, tenendolo in 
    % memoria per tutti i 5000 episodi successivi.
    persistent rete_salvata;
    
    if isempty(rete_salvata)
        % Questa riga calcola magicamente il percorso assoluto corretto in base 
        % a dove si trova questo script, evitando l'errore del percorso sbagliato
        cartella_progetto = fileparts(mfilename('fullpath'));
        percorso_file = fullfile(cartella_progetto, 'deen_network.mat');
        
        % Carica i dati
        dati = load(percorso_file);
        
        % Assicurati che 'deen_net' sia il nome corretto della variabile salvata nel tuo file .mat!
        rete_salvata = dati.deen_net; 
    end
    
    net = rete_salvata;
end
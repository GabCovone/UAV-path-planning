function S = pack_struct(varargin)
    % PACK_STRUCT Raggruppa le variabili passate in ingresso in una struct.
    % I nomi dei campi della struct corrisponderanno ai nomi delle variabili.
    
    S = struct(); % Inizializza una struttura vuota
    
    for i = 1:nargin
        % Ottiene il nome testuale della variabile passata come i-esimo argomento
        nome_variabile = inputname(i);
        
        % Controllo di sicurezza: inputname restituisce vuoto se passi un 
        % valore diretto (es. pack_struct(5)) o un'espressione (es. pack_struct(a+1))
        if isempty(nome_variabile)
            error('L''argomento %d non è una variabile nominata.', i);
        end
        
        % Assegna il valore al campo corrispondente nella struct
        S.(nome_variabile) = varargin{i};
    end
end
function invia_log_esterno(msg, is_validation)
    % Questa funzione estrae la DataQueue dal workspace del worker 
    % e le invia il messaggio, senza interferire col compilatore C di Simulink
    if is_validation
        dq = evalin('base', 'dq_valid');
    else
        dq = evalin('base', 'dq_train');
    end
    
    if ~isempty(dq)
        send(dq, msg);
    end
end
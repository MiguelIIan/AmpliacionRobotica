function ErrorCuadMedio(ejemplo)

    load(ejemplo) % Carga el mapa con los landmarks
    
    data = ekfslam_sim(lm,wp); % Realiza la simulación del vehículo
    
    sumatoriad = 0;
    sumatoriao = 0;
    % Bucle for para hacer el sumatorio
    for i=1:data.i
        % El sumatorio en distancia debemos hacerlo calculando las
        % distancias utilizando pitágoras tanto en la matriz true como en
        % la matriz path
        sumatoriad = sumatoriad + (sqrt(data.true(1,i)^2+data.true(2,i)^2)-sqrt(data.path(1,i)^2+data.path(2,i)^2))^2;
        % El sumatorio en ángulo solo tiene una componente de cada matriz
        sumatoriao = sumatoriao + (data.true(3,i)-data.path(3,i))^2;
    end
    
    % Dividimos ambos sumatorios entre el número de elementos y le hacemos
    % la raíz cuadrada
    RMSEd = sqrt(sumatoriad/data.i)
    RMSEo = sqrt(sumatoriao/data.i)
end
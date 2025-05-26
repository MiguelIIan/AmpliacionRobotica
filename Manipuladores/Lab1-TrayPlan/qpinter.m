function [pr,qlam]= qpinter(Pa,Pb,lambda)
    
    % Interpolación de la posición
    pr = Pa(1:3,4) + lambda*(Pb(1:3,4) - Pa(1:3,4));

    % Interpolación de la orientación
    qa = tr2q(Pa);
    qb = tr2q(Pb);
    qc = qqmul(qinv(qa),qb);
    % Calculas el ángulo y la n en función del quaternio al que quiero
    % llegar
    theta = 2*acos(qc(1));
    n = qc(2:4)/sin(theta/2);
    thetal = lambda*theta;
    % Definimos el cuaternio de rotación
    qr = [cos(thetal/2) n*sin(thetal/2)];
    
    % Multiplicamos el cuaternio del punto a por el cuaternio de rotación
    % para conseguir como resultado el cuaternio del punto intermedio que
    % depende de lambda. Siendo el cuaternio qa cuando lambda es 0 y el
    % cuaternio qb cuando lambda es 1
    qlam = qqmul(qa,qr);
end
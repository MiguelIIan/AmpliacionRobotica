function [P,Q] = generate_smooth_path(P1,P2,P3,tau,T,t)
    
    if (t<-T || t>T)
        disp('Parameter t out of range');
    else
        if (t<=-tau) % Primer Segmento
            [P,Q] = qpinter(P1,P2,(t+T)/T);

        elseif (t>=tau) % Segundo Segmento
            [P,Q] = qpinter(P2,P3,t/T);

        else % Segundo Segmento
            % Interpolación de la posición
            InP2 = P2(1:3,4)-P1(1:3,4);
            InP3 = P3(1:3,4)-P2(1:3,4);
            P = P2(1:3,4) - InP2*(tau-t)^2/(4*tau*T) + InP3*(tau+t)^2/(4*tau*T);
            
            % Interpolación de la orientación
            q1 = tr2q(P1);
            q2 = tr2q(P2);
            q3 = tr2q(P3);

            % Cálculo de qk1
            q12 = qqmul(qinv(q1),q2);
            theta12 = 2*acos(q12(1));
            n12 = q12(2:4)/sin(theta12/2);
            thetak1 = -theta12*(tau-t)^2/(4*tau*T);
            qk1 = [cos(thetak1/2),n12*sin(thetak1/2)];
            
            % Cálculo de qk2
            q23 = qqmul(qinv(q2),q3);
            theta23 = 2*acos(q23(1));
            n23 = q23(2:4)/sin(theta23/2);
            thetak2 = theta23*(tau+t)^2/(4*tau*T);
            qk2 = [cos(thetak2/2),n23*sin(thetak2/2)];

            qk = qqmul(qk1,qk2);
            Q = qqmul(q2,qk);
        end
    end    
end
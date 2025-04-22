function n = P2_PerPura(x0,y0,o0)
    x = x0; % Posición en x
    y = y0; % Posición en y
    o = o0; % Orientación
    v = 0.3; % Velocidad máxima del robot
    d_obj = 1; % Distancia al punto objetivo del camino
    L = 10; % Ancho del pasillo en el eje y
    ruedas = 0.8; % Distanccia entre las ruedas en metros
    r = 0.1; % Radio de las ruedas en metros
    dt = 0.025; % Tiempo de muestreo
    slaser = 0; % Variable para el tiempo de obtenció de datos del laser

    % Entorno cerrado definido por una lista de puntos
    paredx=[0 30 30 0 0]';
    paredy=[0 0 10 10 0]';
    
    t = 70;
    N = 40*t; % Define el tiempo de simulación ya que los saltos son de 0.025s
    wi = zeros(1,N); % Vector con los valores reales de la velocidad angular izquierda
    wd = zeros(1,N); % Vector con los valores reales de la velocidad angular derecha
    wid = zeros(1,N);
    wdd = zeros(1,N);
    v1 = zeros(1,N);
    w1 = zeros(1,N);

    figure
    plot(x0,y0,"*b")
    hold on
    for k = 2:N
        % Vamos recibiendo cada 0.5 segundos la información del laser
        if(slaser < 0.5)
            slaser = slaser + dt;
        else
            slaser = 0;
            rangos= laser2D(paredx,paredy, x, y, o);
            % rangos(19)--> distancia en la dirección y
            % rangos(55)--> distancia en la dirección -y
            dist = rangos(19) + rangos(55);
            if(rangos(19)>rangos(55))
                o = real(acos(L/dist))*sign(o);
            else
                o = -real(acos(L/dist))*(-sign(o));
            end
        end

        dL = L/2 - y;
        
        % Pasamos el punto al que queremos ir a coordenadas locales
        deltax = dL*sin(o)+d_obj*cos(o);
        deltay = dL*cos(o)-d_obj*sin(o);

        distancia = sqrt(deltax^2 + deltay^2);
           
        % Calculamos la curvatura que va a tener el robot en
        % cada instante yendo a por el punto
        curva = 2*deltay/distancia^2;

        % Modelo cinemático inverso
        if (deltax==0 && deltay==0)
            wid(k) = 0;
            wid(k) = 0;
        else
            wid(k) = v*(1-curva*ruedas/2)/r;
            wdd(k) = v*(1+curva*ruedas/2)/r;
        end

        % Modelo del motor. Sacamos las ecuaciones en diferencias de la
        % función de transferencia en Z:
        % G = tf((1/tau),[1 1/tau]);
        % Ghc = ((1-exp(-dt*s))/s);
        % 
        % Gz = c2d(G,dt); -----> Y/X = 0.1881/(z-0.8119)
        %------------------
        % w(k + 1) = 0.1881*u(k) + 0.8119*w(k)
        % w(k) = 0.1881*u(k-1) + 0.8119*w(k-1)
        wi(k) = 0.1881*wid(k-1)+0.8119*wi(k-1);
        wd(k) = 0.1881*wdd(k-1)+0.8119*wd(k-1);

        % Comprobamos que la velocidad de las ruedas no supere el límite
        if (wi(k)>15)
            wi(k) = 15;
        end
        if (wi(k)<-15)
            wi(k) = -15;
        end
        if (wd(k)>15)
            wd(k) = 15;
        end
        if (wd(k)<-15)
            wd(k) = -15;
        end

        % Modelo Cinemático Directo
        v1(k) = (wi(k)+wd(k))*r/2;
        w1(k) = (wd(k)-wi(k))*r/ruedas;
        
        % Ecuaciones que llevan a cabo la simulación del movimiento del
        % robot
        ds = v1(k)*dt;
        do = w1(k)*dt;
        
        o = o + do;
        dx = cos(o).*ds;
        dy = sin(o).*ds;
        x = x + dx;
        y = y + dy;
        plot(x,y,"*r")
    end

    dibujaBarrido(paredx, paredy, x, y, o, rangos);
    
    n = [wi;wd;v1;w1];    
end
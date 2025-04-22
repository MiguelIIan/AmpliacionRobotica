function n = P2_PaP(gain,t)
    x = 0; % Posición en x
    y = 0; % Posición en y
    o=0; % Orientación
    xgps = 0; % Posición en x según el gps
    ygps = 0; % Posición en y según el gps
    ogps = 0; % Orientación según el gps
    ruedas = 0.8; % Distancia entre las ruedas (m)
    r = 0.1; % Radio de las ruedas (m)
    dt = 0.025; % Tiempo de muestreo (s)
    sgps = 0; % Variable que calcula cuando recibimos la información del gps
    i = 1; % Indicador del punto objetivo
    v = 1.2; % Velocidad del robot (m/s)
    puntos = [0 0;20 0;20 20;-10 30;-10 -10;0 -30;0 0]; % Puntos de la ruta

    N = 40*t; % Define el tiempo de simulación ya que los saltos son de 0.025s
    wi = zeros(1,N); % Vector con los valores de velocidad angular de la rueda izquierda
    wd = zeros(1,N); % Vector con los valores de velocidad angular de la rueda izquierda
    wid = zeros(1,N); 
    wdd = zeros(1,N);

    figure
    subplot(2,1,1)
    plot(0,0,"*b")
    hold on
    subplot(2,1,2)
    plot(0,0,"*k")
    hold on

    for k = 2:N
        % Vamos recibiendo cada 0.3 segundos la información del gps
        if(sgps < 0.3)
            sgps = sgps + dt;
        else
            sgps = 0;
            pos = DGPS(x,y,o);
            xgps = pos(1);
            ygps = pos(2);
            ogps = pos(3);
        end
        
        % Pasamos el punto al que queremos ir a coordenadas locales
        puntox = (puntos(i,1)-xgps)*cos(ogps)+(puntos(i,2)-ygps)*sin(ogps);
        puntoy = (puntos(i,2)-ygps)*cos(ogps)-(puntos(i,1)-xgps)*sin(ogps);
        
        % Calculamos la distancia al punto para pasar al siguiente objetivo 
        % cuando estemos a menos de un metro del punto
        distancia = sqrt(puntox^2+puntoy^2);
        if (distancia <= 1)
            i = i + 1;
        end
        
        % Calculamos el ángulo y la curvatura que va a tener el robot en
        % cada instante yendo a por el punto
        anguloe = atan2(puntoy,puntox);
        curva = 2*anguloe;

        % Modelo cinemático inverso
        wid(k) = v*(1-curva*ruedas/2)/r;
        wdd(k) = v*(1+curva*ruedas/2)/r;

        % Modelo del motor. Sacamos las ecuaciones en diferencias de la
        % función de transferencia en Z:
        % G = tf((k/tau),[1 1/tau]);
        % Ghc = ((1-exp(-dt*s))/s);
        % 
        % Gz = c2d(G,dt); -----> Y/X = (k*0.1881)/(z-0.8119)
        %------------------
        % w(k + 1) = k*0.1881*u(k) + 0.8119*w(k)
        % w(k) = k*0.1881*u(k-1) + 0.8119*w(k-1)
        wi(k) = gain*0.1881*wid(k-1)+0.8119*wi(k-1);
        wd(k) = gain*0.1881*wdd(k-1)+0.8119*wd(k-1);

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
        v1 = (wi(k)+wd(k))*r/2;
        w1 = (wd(k)-wi(k))*r/ruedas;
        subplot(2,1,2)
        plot(k,v1,"*b")
        
        % Ecuaciones que llevan a cabo la simulación del movimiento del
        % robot
        ds = v1*dt;
        do = w1*dt;
        
        o = o + do;
        dx = cos(o).*ds;
        dy = sin(o).*ds;
        x = x + dx;
        y = y + dy;
        subplot(2,1,1)
        plot(x,y,"*r")
    end
end
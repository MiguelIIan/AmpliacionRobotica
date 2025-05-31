# Lab Session 1: Cartesian Trayectory Planning

### Interpolación de Cuaterniones
<div align="justify">
En este primer ejercicio vamos a completar la función “qpinter” de matlab que va a realizar la interpolación entre un punto inicial y uno final. Para ello nos vamos a basar en el método de Taylor para hacer la interpolación de la posición. Este método consiste en crear cada una de las posiciones que hay entre el punto inicial y el final usando una ley de temporal λ que define en qué punto de la trayectoria se encuentra el robot. Esta ley la vemos en la ecuación 1.
</div>

$$
\begin{align}
\lambda=\frac{T-t}{T} \qquad\qquad (1)
\end{align}
$$

<div align="justify">
Siendo t el tiempo actual y T el tiempo final de la trayectoria. Con esta ley, conseguimos que cuando λ=0 la posición es la inicial, y cuando λ=1 la posición es la final. Este comportamiento lo veríamos en la ecuación 2.
</div>

$$
\begin{align}
p(t)=p_1-\lambda(t)·(p_1-p_0) \qquad\qquad (2)
\end{align}
$$

<div align="justify">
Siendo p la matriz de posición de los puntos de la trayectoria. En nuestro caso, vamos a usar una ley temporal más adecuada. La vemos en la ecuación 3.
</div>

$$
\begin{align}
\lambda=\frac{t-t_i}{t_f-t_i} \qquad\qquad (3)
\end{align}
$$

<div align="justify">
Siendo t el tiempo actual y tf y ti los tiempos final e inicial de la trayectoria. Con esta ley temporal, nuestra nueva ecuación sería la 4.
</div>

$$
\begin{align}
p(t)=p_0+\lambda(t)·(p_1-p_0) \qquad\qquad (4)
\end{align}
$$

<div align="justify">
Sabiendo esto, ya podemos verlo en el código de la Imagen 1.
</div>
  
<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/e10f1ca1-1fc2-488c-8fa6-440cf0a63bca">
  <br>
  <i>Imagen 1.- Código de la función de interpolación de la posición</i>
</p>

<div align="justify">
Ahora debemos calcular la interpolación de la orientación. Para empezar, un cuaternio se puede ver como una rotación de un ángulo θ con respecto a un eje n. Esto lo vemos en la Imagen 2.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/b2b25e80-2647-456f-9140-5acfa7853b01">
  <br>
  <i>Imagen 2.- Cuaternión calculado a partir de una matriz de rotación</i>
</p>

<div align="justify">
Viendo la Imagen 2, podemos calcular entonces los cuaternios de los puntos iniciales y finales de nuestra trayectoria. En el código, este cálculo lo hace ya la función “tr2q” para calcular los cuaternios de los puntos inicial y final. Ahora, debemos calcular cuál es el “cuaternio de paso” entre el cuaternio inicial y el final, para ello hacemos uso de la ecuación 5.
</div>

$$
\begin{align}
q_A·q_C=q_B → q_C=(q_A)-1·q_B \qquad\qquad (5)
\end{align}
$$

<div align="justify">
Siendo $q_C$ el cuaternio que, compuesto con el cuaternio inicial nos da el final.
  
Conociendo el “cuaternio de paso”, podemos calcular el ángulo y el eje en el que se realiza el giro con las ecuaciones 6 y 7.
</div>

$$
\begin{align}
\theta = 2·acos(q_C(1)) \qquad\qquad (6)\\
n = q_C(2)·sin(\frac{\theta}{2}) \qquad\qquad  (7)
\end{align}
$$

<div align="justify">
Ahora, debemos calcular el “cuaternio de paso” en función de lambda para obtener la orientación en cada punto de la trayectoria. Esto lo hacemos en la ecuación 8.
</div>

$$
\begin{align}
\theta_{\lambda} = \lambda(t)·\theta \qquad\qquad (8)
\end{align}
$$

<div align="justify">
Entonces podemos ahora calcular el cuaternio de rotación en las ecuaciones 9, 10 y 11.
</div>

$$
\begin{align}
w_{rot}(\lambda)=cos(\frac{\theta_{\lambda}(\lambda)}{2}) \qquad\qquad (9)\\
v_{rot}(\lambda)=n·sin(frac{\theta_{\lambda}(\lambda)}{2}) \qquad\qquad  (10)\\
q_{rot}(\lambda)=[w_{rot}(\lambda) , v_{rot}(\lambda)] \qquad\qquad (11)
\end{align}
$$

<div align="justify">
Si ahora aplicamos este cuaternión de rotación al cuaternio inicial ($q_A$), obtenemos la interpolación del cuaternio. Esto lo vemos en la ecuación 12.
</div>

$$
\begin{align}
q_\lambda(\lambda)= q_A·q_{rot}(\lambda) \qquad\qquad (12)
\end{align}
$$

<div align="justify">
Ya tendríamos la interpolación de la posición y de la orientación, por lo que el código Matlab quedaría como lo vemos en la Imagen 3.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/23a1c4bc-04c5-4bd7-8393-b6875c35c22c">
  <br>
  <i>Imagen 3.- Código de la función de interpolación de cuaterniones</i>
</p>

### Generación de una Trayectoria Suavizada
<div align="justify">
Ahora, en este apartado, queremos generar una trayectoria suavizada a lo largo del camino generado por los puntos $p_0$, $p_1$ y $p_2$ como podemos ver en la Imagen 4. Para ello, definimos tres segmentos, el segmento que va desde el tiempo inicial hasta un valor , anterior al punto $p_1$, luego un segmento entre el  anterior y el  posterior a $p_1$ y por último, el segmento desde el  posterior hasta el tiempo final. En la Imagen 4 lo podemos ver más claro.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/a16a16fb-69b8-499e-a11d-aa380dc47f18">
  <br>
  <i>Imagen 4.- Trayectoria suavizada que queremos seguir</i>
</p>

<div align="justify">
Teniendo esto en cuenta, debemos empezar ha hacer los cálculos de la posición y la orientación que debe tener nuestro robot en cada momento.

Para empezar, el primer segmento y el tercero son muy sencillos de calcular, ya que consistiría en la interpolación entre el punto inicial y final de las rectas, definidas por los puntos $p_0p_1$ y $p_1p_2$, teniendo en cuenta que el tiempo no iría desde el inicial hasta el final, si no que acabaría o empezaría en cierto tiempo . Por tanto, comprobamos si estamos dentro del tiempo de la recta, y si es así, llamamos a la función que hicimos con anterioridad, “qpinter” siendo el valor lambda el tiempo actual.

Ahora debemos calcular el segmento intermedio. Para ello, vamos a empezar por el suavizado de la posición y luego terminaremos con el suavizado de la orientación. 

Para el suavizado de la posición, queremos encontrar la función de la Imagen 5, la cuál cumple las condiciones de posición y velocidad que también aparecen en esta imagen.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/cdc4be59-4657-47a9-8be7-275999446186">
  <br>
  <i>Imagen 5.- Función de la posición suavizada y las condiciones que debe cumplir</i>
</p>

<div align="justify">
Para obtener entonces la función de la posición, vamos a empezar desde la aceleración que sabemos que va a ser constante. La podemos ver en la ecuación 13.
</div>

$$
\begin{align}
p''(t)=\frac{\Delta p'}{2·\tau}=\frac{p'(\tau)-p'(-\tau)}{2·\tau}=\frac{\Delta p_2}{2·\tau·T_2}-\frac{\Delta p_1}{2·\tau·T_1} \qquad\qquad (13)
\end{align}
$$

<div align="justify">
Si integramos la aceleración con respecto al tiempo, obtendremos la velocidad en el segmento con respecto al tiempo en la ecuación 14.
</div>

$$
\begin{align}
p'(t)=\int p''(t)dt=\frac{\Delta p_2}{2·\tau·T_2}t-\frac{\Delta p_1}{2·\tau·T_1}t + C_1 \qquad\qquad (14)
\end{align}
$$

<div align="justify">
Donde C es la constante de integración que calcularemos en la ecuación 15 usando las condiciones de la velocidad de la Imagen 5.
</div>

$$
\begin{align}
p'(-\tau)=\frac{\Delta p_1}{T_1}=-\frac{\Delta p_2}{2·\tau·T_2}\tau + \frac{\Delta p_1}{2·\tau·T_1}\tau + C_1\\
p'(\tau)=\frac{\Delta p_2}{T_2}=\frac{\Delta p_2}{2·\tau·T_2}\tau - \frac{\Delta p_1}{2·\tau·T_1}\tau + C_1\\
C1=\frac{\Delta p_2}{2·\tau·T_2}\tau + \frac{\Delta p_1}{2·\tau·T_1}\tau \qquad\qquad (15)
\end{align}
$$

<div align="justify">
Teniendo calculada la constante C, ya podemos definir la velocidad en la ecuación 16.
</div>

$$
\begin{align}
p'(t)=\frac{\Delta p_2}{2·\tau·T_2}(\tau+t) + \frac{\Delta p_1}{2·\tau·T_1}(\tau-t) \qquad\qquad (16)
\end{align}
$$

<div align="justify">
Repetimos el proceso para obtener la posición suavizada en función del tiempo. Calculamos la integral y calculamos la constante de integración usando las condiciones de la Imagen 5 para obtener la posición suavizada respecto del tiempo en la ecuación 17.
</div>

$$
\begin{align}
p(t)=\int p'(t)dt=\frac{2·\tau·t+t^2}{4·\tau·T_2}\Delta p_2 - \frac{2·\tau·t-t^2}{4·\tau·T_1}\Delta p_1 + C_2\\
p(-\tau)=p_1-\frac{\Delta p_1}{T_1}=\frac{-2·\tau^2+\tau^2}{4·\tau·T_2}\Delta p_2 - \frac{-2·\tau^2-\tau^2}{4·\tau·T_1}\Delta p_1 + C_2\\
p(\tau)=p_1+\frac{\tau·\Delta p_2}{T_2}=\frac{2·\tau^2+\tau^2}{4·\tau·T_2}\Delta p_2 - \frac{2·\tau^2-\tau^2}{4·\tau·T_1}\Delta p_1 + C_2\\
C_2=p_1+\frac{\tau^2}{4·\tau·T_2}\Delta p_2 - \frac{\tau^2}{4·\tau·T_1}\Delta p_1\\
p(t)=p1 -\frac{(\tau-t)^2}{4·\tau·T_1}\Delta p_1 - \frac{(\tau+t)^2}{4·\tau·T_2}\Delta p_2 \qquad\qquad (17)
\end{align}
$$

<div align="justify">
Con la ecuación 17, ya hemos obtenido la trayectoria suavizada de la posición que va a seguir nuestro robot en el segmento intermedio con respecto al tiempo.

Ahora, debemos calcular la trayectoria suavizada de la orientación que va a seguir nuestro robot. El cálculo de esta trayectoria viene dado en la Imagen 6.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/9324171f-a00c-42e5-8262-503232432881">
  <br>
  <i>Imagen 6.- Cálculo de la trayectoria suavizada de la orientación</i>
</p>

<div align="justify">
Como vemos en la Imagen 6, necesitamos los cuaternios de la primera y segunda interpolación suavizante. Para calcularlas, vamos a denominar a la primera $q_{k1}$ y a la segunda $q_{k2}$, y vamos a usar las ecuaciones 18 y 19. 
</div>

$$
\begin{align}
q_{01} = q_0^{-1}·q_1\\
\theta_{01}=2·acos(w_{01})\\
n_{01}=\frac{v_{01}}{sin(\frac{\theta_{01}}{2})}\\
\theta_{k1}=-\frac{(\tau-t)^2}{4·\tau·T_1}\theta_{01}\\
q_{k1}=[cos(\frac{\theta_{k1}}{2}) , n_{01}·sin(\frac{\theta_{k1}}{2})] \qquad\qquad (18)
\end{align}
$$

$$
\begin{align}
q_{12}=q_1^{-1}·q_2\\
\theta_{12}=2·acos(w_{12})\\
n_{12}=\frac{v_{12}}{sin(\frac{\theta_{12}}{2})}\\
\theta_{k2}=\frac{(\tau+t)^2}{4·\tau·T_1}\theta_{12}\\
q_{k2}=[cos(\frac{\theta_{k2}}{2}) , n_{12}·sin(\frac{\theta_{k2}}{2})] \qquad\qquad  (19)
\end{align}
$$

<div align="justify">
Habiendo calculado estos dos cuaternios, ya solo nos queda seguir la Imagen 6 y multiplicarlos con el cuaternio del punto donde se va a suavizar la trayectoria. Habiendo hecho todo esto, ya podemos crear el código Matlab en la Imagen 7.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/02ab9a4e-433d-43a2-805e-c7375acfdf4a">
  <br>
  <i>Imagen 7.- Código Matlab para calcular la trayectoria suavizada</i>
</p>

### Representación Gráfica
<div align="justify">
Para la representación gráfica, solo debemos ejecutar el nombre del código principal en la ventana de comandos. Esto ejecutará una simulación en la que vemos un brazo robótico haciendo la trayectoria suavizada que hemos calculado. Esta simulación la podemos ver en el vídeo.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/71d4a337-91c4-4844-bd79-321e9ecef75e">
  <br>
</p>

<div align="justify">
Con esta simulación, obtenemos las siguientes gráficas, que representan las variables de posición y de orientación a lo largo del tiempo. Estas gráficas, las vemos en las Imágenes 8 y 9.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/b4592728-c7b6-4c5d-8044-9da9e46f22e6">
  <br>
  <i>Imagen 8.- Posición cartesiana del efector final del robot a lo largo del tiempo</i>
</p>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/68e3c51e-dc90-4d5e-a3e9-e9768ddcae71">
  <br>
  <i>Imagen 9.- Ángulos de Euler del efector final del robot a lo largo del tiempo</i>
  <br><br>
</p>

<div align="justify">
Viendo las gráficas de orientación, podemos ver algo extraño, y es que el ángulo “gamma” justo en el punto medio de la trayectoria llega a valores altísimos. Este comportamiento se debe a que justo el punto “P1” de la trayectoria produce una singularidad en la orientación del manipulador, pero al hacer una trayectoria suavizada, pasamos cerca, pero no por el punto. Lo que hace que la orientación no tienda a infinito pero sí que sea muy alta.
</div>

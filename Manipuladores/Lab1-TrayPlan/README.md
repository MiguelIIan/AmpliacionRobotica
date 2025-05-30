# Lab Session 1: Cartesian Trayectory Planning

### Interpolación de Cuaterniones
En este primer ejercicio vamos a completar la función “qpinter” de matlab que va a realizar la interpolación entre un punto inicial y uno final. Para ello nos vamos a basar en el método de Taylor para hacer la interpolación de la posición. Este método consiste en crear cada una de las posiciones que hay entre el punto inicial y el final usando una ley de temporal λ que define en qué punto de la trayectoria se encuentra el robot. Esta ley la vemos en la ecuación 1.

$`\lambda=\frac{T-t}{T}`$   (1)

Siendo t el tiempo actual y T el tiempo final de la trayectoria. Con esta ley, conseguimos que cuando λ=0 la posición es la inicial, y cuando λ=1 la posición es la final. Este comportamiento lo veríamos en la ecuación 2.

$`p(t)=p_1-\lambda(t)·(p_1-p_0)`$    (2)

Siendo p la matriz de posición de los puntos de la trayectoria. En nuestro caso, vamos a usar una ley temporal más adecuada. La vemos en la ecuación 3.

$`\lambda=\frac{t-t_i}{t_f-t_i}`$    (3)

Siendo t el tiempo actual y tf y ti los tiempos final e inicial de la trayectoria. Con esta ley temporal, nuestra nueva ecuación sería la 4.

$`p(t)=p_0+\lambda(t)·(p_1-p_0)`$    (4)

Sabiendo esto, ya podemos verlo en el código de la Imagen 1.


Imagen 1.- Código de la función de interpolación de la posición


Ahora debemos calcular la interpolación de la orientación. Para empezar, un cuaternio se puede ver como una rotación de un ángulo θ con respecto a un eje n. Esto lo vemos en la Imagen 2.



Imagen 2.- Cuaternión calculado a partir de una matriz de rotación


Viendo la Imagen 2, podemos calcular entonces los cuaternios de los puntos iniciales y finales de nuestra trayectoria. En el código, este cálculo lo hace ya la función “tr2q” para calcular los cuaternios de los puntos inicial y final. Ahora, debemos calcular cuál es el “cuaternio de paso” entre el cuaternio inicial y el final, para ello hacemos uso de la ecuación 5.

$`q_A·q_C=q_B → q_C=(q_A)-1·q_B`$    (5)

Siendo qC el cuaternio que, compuesto con el cuaternio inicial nos da el final. 
Conociendo el “cuaternio de paso”, podemos calcular el ángulo y el eje en el que se realiza el giro con las ecuaciones 6 y 7.

$`\theta = 2·acos(q_C(1))`$       (6)

$`n = q_C(2)·sin(\frac{\theta}{2})`$         (7)

Ahora, debemos calcular el “cuaternio de paso” en función de lambda para obtener la orientación en cada punto de la trayectoria. Esto lo hacemos en la ecuación 8.

$`\theta_{\lambda} = \lambda(t)·\theta`$        (8)

Entonces podemos ahora calcular el cuaternio de rotación en las ecuaciones 9, 10 y 11.

wrot()=cos(()2)        (9)
vrot()=n·sin(()2)        (10)
qrot()=[wrot() , vrot()]        (11)

Si ahora aplicamos este cuaternión de rotación al cuaternio inicial (qA), obtenemos la interpolación del cuaternio. Esto lo vemos en la ecuación 12.

q()= qA·qrot()       (12)

Ya tendríamos la interpolación de la posición y de la orientación, por lo que el código Matlab quedaría como lo vemos en la Imagen 3.

Imagen 3.- Código de la función de interpolación de cuaterniones


Generación de una Trayectoria Suavizada
Ahora, en este apartado, queremos generar una trayectoria suavizada a lo largo del camino generado por los puntos p0, p1 y p2 como podemos ver en la Imagen 4. Para ello, definimos tres segmentos, el segmento que va desde el tiempo inicial hasta un valor  , anterior al punto p1, luego un segmento entre el  anterior y el  posterior a p1 y por último, el segmento desde el  posterior hasta el tiempo final. En la Imagen 4 lo podemos ver más claro.

Imagen 4.- Trayectoria suavizada que queremos seguir


Teniendo esto en cuenta, debemos empezar ha hacer los cálculos de la posición y la orientación que debe tener nuestro robot en cada momento.

Para empezar, el primer segmento y el tercero son muy sencillos de calcular, ya que consistiría en la interpolación entre el punto inicial y final de las rectas, definidas por los puntos p0p1 y p1p2, teniendo en cuenta que el tiempo no iría desde el inicial hasta el final, si no que acabaría o empezaría en cierto tiempo . Por tanto, comprobamos si estamos dentro del tiempo de la recta, y si es así, llamamos a la función que hicimos con anterioridad, “qpinter” siendo el valor lambda el tiempo actual.

Ahora debemos calcular el segmento intermedio. Para ello, vamos a empezar por el suavizado de la posición y luego terminaremos con el suavizado de la orientación. 

Para el suavizado de la posición, queremos encontrar la función de la Imagen 5, la cuál cumple las condiciones de posición y velocidad que también aparecen en esta imagen.


Imagen 5.- Función de la posición suavizada y las condiciones que debe cumplir


Para obtener entonces la función de la posición, vamos a empezar desde la aceleración que sabemos que va a ser constante. La podemos ver en la ecuación 13.
p''(t)=p'2·=p'()-p'(-)2·=p22··T2-p12··T1        (13)
Si integramos la aceleración con respecto al tiempo, obtendremos la velocidad en el segmento con respecto al tiempo en la ecuación 14.
p'(t)=p''(t)dt=p22··T2t-p12··T1t+C1      (14)

Donde C es la constante de integración que calcularemos en la ecuación 15 usando las condiciones de la velocidad de la Imagen 5.
p'(-)=p1T1=-p22··T2+p12··T1+C1
p'()=p2T2=p22··T2-p12··T1+C1
C1=p22··T2+p12··T1           (15)

Teniendo calculada la constante C, ya podemos definir la velocidad en la ecuación 16.
p'(t)=p22··T2(+t)+p12··T1(-t)      (16)

Repetimos el proceso para obtener la posición suavizada en función del tiempo. Calculamos la integral y calculamos la constante de integración usando las condiciones de la Imagen 5 para obtener la posición suavizada respecto del tiempo en la ecuación 17.
p(t)=p'(t)dt=2··t+t24··T2p2-2··t-t24··T1p1+C2
p(-)=p1-p1T1=-2·2+24··T2p2--2·2-24··T1p1+C2
p()=p1+·p2T2=2·2+24··T2p2-2·2-24··T1p1+C2
C2=p1+24··T2p2-24··T1p1
p(t)=p1 -(-t)24··T1p1+(+t)24··T2p2        (17)

Con la ecuación 17, ya hemos obtenido la trayectoria suavizada de la posición que va a seguir nuestro robot en el segmento intermedio con respecto al tiempo.

Ahora, debemos calcular la trayectoria suavizada de la orientación que va a seguir nuestro robot. El cálculo de esta trayectoria viene dado en la Imagen 6.

Imagen 6.- Cálculo de la trayectoria suavizada de la orientación


Como vemos en la Imagen 6, necesitamos los cuaternios de la primera y segunda interpolación suavizante. Para calcularlas, vamos a denominar a la primera qk1 y a la segunda qk2, y vamos a usar las ecuaciones 18 y 19. 

q01=q0-1·q1
01=2·acos(w01)
n01=v01sin(01/2)
k1=-(-t)24··T101
qk1=[cos(k12) , n01·sin(k12)]        (18)

q12=q1-1·q2
12=2·acos(w12)
n12=v12sin(12/2)
k2=(+t)24··T112
qk2=[cos(k22) , n12·sin(k22)]        (19)

Habiendo calculado estos dos cuaternios, ya solo nos queda seguir la Imagen 6 y multiplicarlos con el cuaternio del punto donde se va a suavizar la trayectoria. Habiendo hecho todo esto, ya podemos crear el código Matlab en la Imagen 7.

Imagen 7.- Código Matlab para calcular la trayectoria suavizada




![Vídeo-sin-título-‐-Hecho-con-Clipchamp](https://github.com/user-attachments/assets/71d4a337-91c4-4844-bd79-321e9ecef75e)

Siendo las gráficas de la posición y la orientación del manipulador las siguientes:

![image](https://github.com/user-attachments/assets/b4592728-c7b6-4c5d-8044-9da9e46f22e6)

![image](https://github.com/user-attachments/assets/68e3c51e-dc90-4d5e-a3e9-e9768ddcae71)

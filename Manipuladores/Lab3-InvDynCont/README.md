# Lab Session 3: Inverse Dynamics Control

<div align="justify">
En esta sesión se ha aprendido a implementar controladores de dinámica inversa para compensar la no linealidad de las dinámicas del manipulador y poder imponerle el comportamiento dinámico que se desee.
</div>

## Gravity compensation

### Implementación

<div align="justify">
En este caso donde la única fuerza que influye en el movimiento es la de la gravedad, el controlador de dinámica inversa es el más simple, al solo enviar a los actuadores el torque necesario para cancelar la gravedad. En la ecuación 1
se pueden ver los torques que se le van a enviar a los motores.
</div>

$$
\begin{align}
\tau = g(q) \qquad (1)
\end{align}
$$

<div align="justify">
Despues de crear el nodo "gravity_compensation.cpp" y copiar el código dado, se debe rellenar la función "gravity_compensation()", que va a calcular los torques necesarios que se les deben proporcionar a los motores. Este código se puede observar en la Imagen 1.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/d980ea52-8ed9-4edd-94d4-721b18a98679">
  <br>
  <i>Imagen 1.- Calculo del torque de los motores para compensar la gravedad.</i>
</p>

### Ejecución del controlador

<div align="justify">
Teniendo el código completo, falta, antes de poder simular, crear el archivo launch con el que se lanzarán los nodos y modificar el archivo CMakeList. Habiendo hecho esto, ya se pueden lanzar los siguientes comandos y ver que ocurre en la Imagen 2.
<br><br>
</div>

    ros2 launch uma_arm_description uma_arm_visualization.launch.py 

    ros2 launch uma_arm_control gravity_compensation_launch.py 

    ros2 launch uma_arm_control uma_arm_dynamics_launch.py

<div align="justify">
Cada uno de estos comandos se debe ejecutar en una terminal a parte. Podemos ver entonces el resultado en RViz en la Imagen 2.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/562c3661-bc98-4fec-a8f2-7627951279b2">
  <br>
  <i>Imagen 2.- Visualización del comportamiento del manipulador contrarrestando la gravedad.</i>
</p>


<div align="justify">
En la Imagen 2 podemos observar como el manipulador se queda totalmente estático en la posición inicial dada. Esto se debe a que nuestro código está contrarrestando el efecto de la gravedad en el movimiento del manipulador.
</div>

### Simulación de fuerzas 

<div align="justify">
Como en la simulación no hay manera de incluir sensores de fuerza, se van a aplicar fuerzas virtuales al manipulador para ver como se comporta. Esto se puede hacer gracias a que, en la sesión anterior, en el nodo de la dinámica del manipulador, se incluyeron los torques producidos por fuerzas externas. Para entenderlo mejor, tenemos el Video 1.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/b80af15f-e25c-4824-9e5e-4a1039006c48">
  <br>
  <i>Video 1.- Comportamiento del manipulador al aplicarle fuerzas en distintas direcciones.</i>
</p>


<div align="justify">
En el Video 1 se puede apreciar como las fuerzas en las distintas direcciones generan un movimiento del robot. El problema está en que al cesar esas fuerzas, la inercia del manipulador se mantiene, por lo que se sigue moviendo por un tiempo aunque nuestro controlador esté contrarrestando la fuerza de la gravedad. 
<br><br>
En la Imagen 3 se pueden ver las relaciones entre los nodos y los tópicos que se están ejecutando.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/f14b28ad-b319-4416-abbf-647e71107151">
  <br>
  <i>Imagen 3.- Nodos y tópicos activos en la simulación con fuerzas externas.</i>
</p>

## Linealización mediante Control de Dinámica Inversa

### Implementación

<div align="justify">
Ahora, se puede compensar las dinámicas no lineales del manipulador usando el método de linealización por realimentación. De esta manera, se podrá ser capaz de alcanzar un comportamiento dinámico deseado sin que las propias dinámicas del manipulador afecten a su movimiento. Para llevar a cabo esto, se debe seguir el esquema de la Imagen 4.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/667c7818-1b74-4ed9-9721-017def9eb8c4">
  <br>
  <i>Imagen 4.- Esquema de control de las dinámicas del manipulador.</i>
</p>

<div align="justify">
Se necesita entonces crear un nuevo nodo con el que calcular la cancelación de la dinámica no lineal del manipulador, basado en las aceleraciones articulares deseadas y el estado actual de las articulaciones. Entonces, en la Ecuación 2, tendremos el valor de los torques articulares que enviaremos al manipulador.
</div>

$$
\begin{align}
n(q,q') = C(q,q')q' + F_bq' + g(q) \\
\tau = M(q)q_D'' + n(q,q') \qquad\qquad (2)
\end{align}
$$

<div align="justify">
Entonces, se crea el nodo "dynamics_cancellation", se copia el código dado en la práctica y se rellena la función "cancel_dynamics" con el código de la Imagen 5.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/cca9561e-fdba-4a45-ad79-d7635408dd0b">
  <br>
  <i>Imagen 5.- Código del cálculo de la cancelación de la dinámica.</i>
</p>

### Ejecutando el controlador

<div align="justify">
Para simular el controlador se deben ejecutar los siguientes comandos, cada uno en un terminal.
</div>

    ros2 launch uma_arm_description uma_arm_visualization.launch.py 

    ros2 launch uma_arm_control dynamics_cancellation_launch.py 

    ros2 launch uma_arm_control uma_arm_dynamics_launch.py

    cd src/uma_arm_control/utils && python3 cubic_trajectory.py

<div align="justify">
Usando cada uno de estos comandos se consigue el comportamiento del Video 2. En la Imagen 6 se pueden ver también las relaciones entre los nodos activos.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/9462387a-f26a-4ed8-ab3b-d2f5bcbf4cef">
  <br>
  <i>Video 2.- Trayectoria del manipulador con cancelación de su dinámica.</i>
</p>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/80a5edca-cf3b-4824-b679-76c3a095a215">
  <br>
  <i>Imagen 6.- Relación entre los nodos en la cancelación de la dinámica.</i>
</p>

<div align="justify">
Para ver con más claridad este comportamiento, se puede grabar usando las rosbags y graficar el comportamiento en Plotjuggler en la Imagen 7.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/07b5e460-adae-471f-b90a-f7c6a018421c">
  <br>
  <i>Imagen 7.- Comportamiento del manipulador a lo largo de la trayectoria.</i>
</p>

## Experimentos

<div align="justify">
¿Qué ocurre si el modelo de compensación de la dinámica no es igual a la dinámica del manipulador?. En este experimento, se ha cambiado las masas del modelo de cancelación de la dinámica, la masa 1 de 3 a 5 kilogramos y la masa 2 de 2 a 3 kilogramos. El resultado se puede observar en la Imagen 8.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/58817260-c343-4792-9440-3ff531a2cfe1">
  <br>
  <i>Imagen 8.- Comportamiento del manipulador cambiando las masas en la cancelación de la dinámica.</i>
</p>

<div align="justify">
El comportamiento del manipulador es totalmente aleatorio, sin control. Esto se debe a que la cancelación de la dinámica envía el torque necesario para mantener 5 kilogramos en suspensión cuando en realidad son 3, por lo que sobreoscila. 
<br><br>
Probemos ahora a cambiar la compensación de la gravedad, la longitud 1 pasa a ser de 2 metros en vez de uno, y la longitud 2 pasa a ser de 2 en vez de 0.6. En la Imagen 9 se ven los resultados.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/7cb869d1-65e6-4b6c-8452-dece3e4b6418">
  <br>
  <i>Imagen 9.- Comportamiento del manipulador cambiando las longitudes en la compensación gravitatoria.</i>
</p>

<div align="justify">
En este caso se puede ver que el manipulador calcula un torque de los motores mayor al que realmente necesita para contrarrestar la gravedad, lo que provoca q el manipulador suba hacia arriba.
<br><br>
Por último, se va a observar el comportamiento del manipulador cuando cancela su dinámica pero le aplicamos fuerzas. Este experimento se puede ver en el Video 3. El manipulador no se mantiene estable ni se frena, esto se debe a que, al cancelar su propia dinámica, contrarresta los torques producidos por las inercias de los eslabones, lo que no quiere decir que se mantenga quieto, sino que si se está moviendo a una velocidad, va a seguir moviendose a esa velocidad constantemente sin tener cambios de torque en los motores.
</div>

<p align="center">
  <br>
  <img src="https://github.com/user-attachments/assets/54f6809d-aaad-42e2-9e60-6e9f8fa1e30a">
  <br>
  <i>Video 3.- Comportamiento del manipulador al ejercerle fuerzas externas.</i>
</p>





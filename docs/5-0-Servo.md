# 6. Servo 

<p style="text-align: justify;">
Con los pasos requeridos para ejecutar el robot diferencial, se procedió a realizar el mismo proceso para un servomotor. Para esto, como bien se menciona anteriormente, se pueden utilizar dos métodos: 
</p style="text-align: justify;">

1. Constituir el servo con figuras volumétricas básicas
2. Extraer el URDF con el plugin de solidworks como se mostró en el ejemplo anterior. 

<p style="text-align: justify;">
Un servomotor es un dispositivo que permite controlar con máxima precisión  la posición y movimiento de su eje. Esto quiere decir que se puede mover en un ángulo, posición y a una velocidad determinada en cada momento, cosa que no puede hacer con normalidad un motor eléctrico. Industrialmente, se utilizan los servos en todas aquellas aplicaciones de automatización industrial y robótica en las que se necesitan un exhaustivo control del par, del posicionamiento y de la velocidad para mejorar la calidad y la productividad. <br> <br>

Los servomotores pueden ser industriales o de electrónica básica, su principal diferencia son los límites de rotación que pueden llegar a tener, un servo industrial puede tener un giro de 360 grados sin problema, mientras un servo de electrónica básica solamente tiene libertad de 180 grados. En este caso se estará trabajando con un servo de electrónica básica como el siguiente:
</p style="text-align: justify;">

![](img/servo.jpg)


</p style="text-align: justify;">

En terminos de mecanismos internos, un servo es un dispositivo que se puede modelar muy fácilmente, solo contiene un _chasis_ que sería la _caja azul_ y tendría la parte superior que rota sobre un eje que se denomina como _plumilla_. Ante esta situación, se decidió realizar toda la aplicación del servo con figuras volumétricas básicas, toda esta configuración se encuentra en el siguiente [enlace](https://gitlab.com/semillero-ares/ros2_servo). <br><br>

</p style="text-align: justify;">
<p style="text-align: justify;">

En las siguientes páginas se explicará brevemente los pasos para controlar un servomotor que posee 1 DOF y el contenido del repositorio que contiene los códigos utilizados en la solución del problema.<br>
Se controla primeramente un servomotor con la intención de entender cómo funciona robóticamente ROS 2 y cómo se controla cada grado de libertad, posteriormente se implementará lo aprendido para controlar el robot SCARA. 
<p style="text-align: justify;">

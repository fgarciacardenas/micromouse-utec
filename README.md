# Concurso Róbotica: Virtual Maze

Repositorio oficial de Virtual Maze, concurso virtual de robots micromouse.

## Primeros pasos

Este repositorio contiene los archivos URDF de los micromouse básicos y nodos de control de velocidad,vistos en los talleres. Además, este es necesario para la creación de laberintos aleatorios. Para empezar, seguir los siguientes pasos.

- Dirigirse a la esquina superior derecha de este repositorio, donde se encuentra el botón verde llamado "Code".
- Presionar dicho botón y copiar el enlace HTTPS.
- En su terminal, ejecutar el siguiente comando:

``` bash
git clone https://github.com/fgarciacardenas/micromouse-utec.git nombre_carpeta
```

el enlace es el mismo que se copió en el paso anterior y *nombre_carpeta* es el nombre de la carpeta en la que se clonará el repositorio (si no se especifica, el nombre de la carpeta será micromouse-utec por defecto).

- Entrar a la carpeta del repositorio clonado:

``` bash
cd nombre_carpeta
```

Dentro se podrá ver una carpeta adicional llamada `src` y un este archivo `README.md`.

- Ejecutar el siguiente comando para compilar el espacio de trabajo de ROS:

``` bash
catkin_make
```

se generarán 2 carpetas adicionales `devel` y `build`.

- Agregar el *path* del espacio de trabajo al `.bashrc`

``` bash
echo -e "source /{PATH a nombre_carpeta}/devel/setup.bash" >> ~/.bashrc 
```
- Cerrar el terminal y volver a abrirlo. Si se tiene otros terminales, también deben de cerrarse, de lo contrario todos los terminales que estaban abiertos anteriormente no habrán guardado los cambios anteriores.

- Comprobar que funcione correctamente

``` bash
roscd maze_generator/
```

Si el comando anterior se puede ejecutar sin ninguna falla y te localiza en otra dirección de carpeta, cada vez que se abra un terminal, este será capaz de encontrar el espacio de trabajo *nombre_carpeta* sin que se tenga que definir explícitamente.

Si el comando anterior genera algún error, aún se puede definir explícitamente el *path* al espacio de trabajo con el siguiente comando:

``` bash
source /{PATH a nombre_carpeta}/devel/setup.bash
```

Nota: Solo el terminal en el que se ejecute el comando anterior podrá encontrar el espacio de trabajo. Si se quiere utilizar más terminales, en estos también se tiene que ejecutar este comando.

 ***

## Generar laberinto aleatorio

- Ejecutar los siguientes comandos para dirigirse al ejecutable encargado de generar un laberinto aleatorio

``` bash
roscd maze_generator/
cd src
cd generator
```

- Correr el ejecutable

``` bash
./generate_maze
```

El siguiente mensaje aparecerá

``` bash
A new maze was generated
```

- Ejecutar los siguientes comandos para comprobar que se generó un laberinto:

``` bash
roslaunch maze_generator maze.launch
```

- Presionar ctrl+c en el terminal donde se ejecutó el comando anterior para cerrar las herramientas de simulación. Importante: No cerrar el terminal ni presionar ctrl+z. De lo contrario, los servicios no se detendrán correctamente y seguirán consumiendo recursos de manera innecesaria.

- Opcional: Repetir los pasos anteriores para comprobar que efectivamente cada laberinto generado es único, es decir, cada uno es diferente al otro.

***

## Controlar manualmente al robot

- Al igual que el paso anterior, correr el comando para empezar la simulación. Adicionalmente a esto, se puede elegir el tipo de micromouse que se quiere simular.

``` bash
roslaunch maze_generator maze.launch robot:=tipo_micromouse
```

donde *tipo_micromouse* es el nombre de uno de los 3 archivos URDF de micromouse disponibles en la carpeta `urdf`.

1. mouse (no tiene sensores)
2. mouse_ir (tiene un sensor infrarrojo)
3. mouse_lidar (tiene un LiDAR)

Nota: Colocar el nombre sin la extensión. Ejemplo:

``` bash
roslaunch maze_generator maze.launch robot:=mouse_ir
```

Si no se especifica un robot, se utiliza al *mouse_lidar*.

- Luego de ejecutar el comando anterior, se debe ejecutar en otro terminal el nodo `cmd_vel.py` con el siguiente comando:

``` bash
rosrun maze_generator cmd_vel.py
```

Aparecerá el siguiente mensaje:

``` bash
Control Your Micromouse!
---------------------------
Moving around:
        w
   a    s    d

space key : force stop

CTRL-C to quit
```

En ese mismo terminal presionar las teclas indicadas por el mensaje para controlar el robot y ctrl+c para terminar la ejecución el nodo.

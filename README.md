# Robotica-Movil

## TP1

Primero copiar la carpeta mav0 de este [zip](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip) en la carpeta tp1 y luego ingresar a este directorio.

Para instalar las depencias utilizar:

```
pip install -r requirements
```

Para generar el archivo CSV del camino ground-truth de la camara izquierda utilizar:

```
python3 gen_cam0.py
```

Para poder obtener el grafico de ambos ground-truth utilizar:

```
python3 gen_plot.py NUMERO_DE_POSES NUMERO_DE_PASOS
```

Por defecto se utilizan:

```
NUMERO_DE_POSES: 10
NUMERO_DE_PASOS: 1
```

## TP2

Para instalar las dependencias utilizar:

```
pip install -r tp2/requirements
```

### Generacion de graficos segun archivos de odom

Para poder grabar el registro de odom utilizar, siendo `dump_odom.py` provisto por la catedra:

```
python3 ./dump_odom.py > log.txt
```

Para poder generar los graficos correr:

```
python3 tp2/dump_parser.py ARCHIVO_DUMP_ODOM
```

El script toma los siguientes parametros opcionales:

- `--path` grafica el camino recorrido
- `--pose` grafica la pose del robot
  - `--step n` cada cuantos puntos grafica una pose
- `--linear` grafica la velocidad lineal a lo largo del tiempo
- `--angular` grafica la velocidad angular a lo largo del tiempo
- `-x` grafica la posicion en x a lo largo del tiempo
- `-y` grafica la posicion en y a lo largo del tiempo
- `--orientation` grafica la orientacion a lo largo del tiempo
- `-p [--points] f f f...` toma una serie de porcentajes y los marca en todos los graficos. Ejemplo: `-p 0.1 0.3 0.5 0.75 0.8` este marcara los puntos de interes al 10%, 30%, 50%, 75% y 80% del camino en todos los graficos

### Obtencion de posicion de cilindros

Para poder obtener el registro de scan utilizar:

```
python3 ./dump_scan.py > scan.txt
```

Para poder obtener la posicion de los cilindros correr:

```
python3 tp2/scan_parser.py ARCHIVO_DUMP_SCAN
```

El script toma los siguientes parametros opcionales:

- `-x f` toma un numero decimal y lo utiliza como la coordena x del robot. Por defecto es `0`.
- `-y f` toma un numero decimal y lo utiliza como la coordena y del robot. Por defecto es `0`.
- `-o [--orientation] f` toma un numero decimal y lo utiliza como la orientacion en radianes del robot. Por defecto es `0`.
- `-m [--min-range] f` toma un numero decimal y lo utiliza como el rango minimo del LIDAR, cualquier medicion inferior es descartada. Por defecto es `0`.
- `-M [--max-range] f` toma un numero decimal y lo utiliza como el rango maximo del LIDAR, cualquier medicion superior es descartada. Por defecto es `11.0`.
- `-a [--angle-min] f` toma un numero decimal y lo utiliza como el angulo minimo del barrido. Por defecto es `0`.
- `-i [--angle-increment] f` toma un numero decimal y lo utiliza como el angulo en el que diferencian un haz del barrido del otro. Por defecto es `0.01749303564429283`.

Algunos de estos valores como el min-range, max-range, angle-min y angle-increment se pueden obtener corriendo:

```
ros2 topic echo /scan
```

El archivo `mesurements.txt` contiene un ejemplo de la salida de este comando.

Para poder determinar la posicion y orientacion se puede utilizar el script de `dump_odom.py`. El archivo `pose.txt` contiene un ejemplo de la salida del mismo. De la misma forma se puede utilizar Gazebo para determinar estos valores.

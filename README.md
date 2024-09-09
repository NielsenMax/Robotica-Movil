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

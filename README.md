# treeSLAM: Reconstrucción 3D de Árboles a partir de Nubes de Puntos

Este repositorio contiene un conjunto de scripts de MATLAB diseñados para procesar, analizar y reconstruir modelos 3D de árboles a partir de datos de nubes de puntos. El proyecto implementa un enfoque de Simultaneous Localization and Mapping (SLAM) para alinear múltiples escaneos y generar un modelo cohesivo de una plantación.

Este trabajo es un material complementario a la tesis Doctoral titulada **"Reconstrucción espacio-temporal de cultivos utilizando estrategias de sensado activo"**, del Instituto de Automática (INAUT).

## Requisitos

*   **MATLAB:** Es necesario tener instalado MATLAB.
*   **Toolboxes de MATLAB:**
    *   Computer Vision Toolbox
    *   Statistics and Machine Learning Toolbox

## Guía de Inicio Rápido

Para utilizar este proyecto, sigue los siguientes pasos para descargar los datos y ejecutar los scripts principales.

### 1. Descarga de los Datos

El primer paso es descargar el conjunto de datos de nubes de puntos. Los datos no están incluidos en este repositorio debido a su gran tamaño.

*   **Enlace de descarga:** [Descargar datos de Nube de Puntos (plantacion_grande_59_mat.tar.xz)](https://nextcloud.unsj.edu.ar/index.php/s/ABmR3HcBbSgAwom)

### 2. Descompresión

Una vez descargado el archivo, debes descomprimirlo para extraer el fichero `.mat` que contiene las nubes de puntos.

*   Desde una terminal en Linux/macOS, puedes usar el siguiente comando:
    ```bash
    tar -xvf plantacion_grande_59_mat.tar.xz
    ```
*   En Windows, puedes usar un programa como 7-Zip o WinRAR para extraer el contenido del archivo `plantacion_grande_59_mat.tar.xz`.

Esto generará el archivo `plantacion_grande_59.mat`.

### 3. Ejecución de la Reconstrucción Principal

Este script es el punto de partida del proceso. Carga las nubes de puntos y ejecuta el algoritmo SLAM para reconstruir la escena.

1.  Abre MATLAB.
2.  Asegúrate de que el archivo `plantacion_grande_59.mat` esté en el mismo directorio que los scripts, o añade su ubicación al path de MATLAB.
3.  Carga el archivo en el workspace de MATLAB con el comando:
    ```matlab
    load('plantacion_grande_59.mat');
    ```
4.  Ejecuta el script de reconstrucción principal:
    ```matlab
    reconstruccion_arboles_v5.m
    ```

Este script procesará los datos y guardará los resultados, incluyendo las transformaciones calculadas y las nubes de puntos alineadas.

### 4. Visualización basada en Incertidumbre

Este segundo script permite visualizar y analizar la reconstrucción filtrando los puntos en función de la incertidumbre asociada a su registro.

1.  Abre el script `pcSelected_part2.m` en el editor de MATLAB.
2.  **Configura los parámetros** dentro del script según tus necesidades. Puedes ajustar umbrales de incertidumbre o seleccionar qué partes de la reconstrucción deseas visualizar.
3.  Ejecuta el script:
    ```matlab
    pcSelected_part2.m
    ```
Esto generará una visualización 3D de la nube de puntos reconstruida, coloreada o filtrada según la incertidumbre calculada en el paso anterior.

## Descripción de Archivos

A continuación se describe la función de los scripts más importantes de este repositorio:

*   `reconstruccion_arboles_v5.m`: Script principal que orquesta todo el proceso de SLAM. Carga los datos, itera sobre las nubes de puntos, detecta árboles, calcula las transformaciones entre escaneos sucesivos y fusiona los datos en un mapa global.
*   `pcSelected_part2.m`: Script de visualización y análisis. Permite inspeccionar la reconstrucción final, aplicando filtros basados en la incertidumbre de la estimación de la pose para evaluar la calidad del mapa generado.
*   `get1scan.m`: Función auxiliar para extraer un único escaneo (nube de puntos) del archivo de datos cargado.
*   `get_matches.m`: Se encarga de encontrar correspondencias entre dos nubes de puntos, un paso fundamental para el registro y la alineación.
*   `get_transform_matrix.m`: Calcula la matriz de transformación (rotación y traslación) que alinea dos nubes de puntos a partir de las correspondencias encontradas.
*   `project_on_plane.m`: Proyecta una nube de puntos 3D sobre un plano 2D, útil para la detección de troncos de árboles o para simplificar el problema de correspondencia.
*   `tree_detector.m`: Implementa un algoritmo para detectar la posición de los árboles en una nube de puntos 2D (proyectada). Probablemente utiliza técnicas de clustering o ajuste de círculos.

## Flujo de Trabajo del Algoritmo

1.  **Carga de Datos**: Se carga el archivo `.mat` con la secuencia de nubes de puntos.
2.  **Procesamiento Iterativo (SLAM)**: El script `reconstruccion_arboles_v5.m` procesa los escaneos de forma secuencial.
    *   Para cada par de escaneos consecutivos, se detectan los árboles en cada uno.
    *   Se establecen correspondencias entre los árboles detectados en ambos escaneos.
    *   Se calcula la transformación 3D (pose del sensor) que alinea los árboles correspondientes.
    *   La nueva nube de puntos se integra en el mapa global utilizando la transformación calculada.
3.  **Análisis de Incertidumbre**: El script `pcSelected_part2.m` utiliza las matrices de covarianza y otros estimadores de incertidumbre generados durante el proceso SLAM para visualizar la fiabilidad de las diferentes partes del mapa reconstruido.

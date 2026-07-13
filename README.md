# PFC1: ALNSQ para VRPTW

### Archivos de implementación de los algoritmos
* `'VRPTW Environment'/`: Estructuras de datos para los elementos del problema VRPTW (cliente, ruta, instancia) y clase Solution para las soluciones del problema a optimizar
* `Operators/`: operadores de destrucción y reparación
* `ALNS/`: implementación de algoritmos ALNS y ALNS-Q
* `Utils/`: funciones de utilidad para la ejecución de experimentos
* `solomon-100`: benchmark de las 56 instancias de Solomon de 100 clientes

### Resultados y análisis de experimentos computacionales
* `Results/`: archivo de resultados y rutas solución de ALNS y ALNS-Q para cada ejecución
* `Experiments/`:
    * `automate.py`: para automatizar los experimentos (1120 ejecuciones)
    * `verify.py`: verificador de todas las ejecuciones de la experimentación
    * `analisis.ipynb`: notebook de análisis
    * `df_master_cache.csv`: archivo CSV del acumulado de todas las ejecuciones de los experimentos
    * `sintef.csv`: archivo CSV de los BKS del SINTEF
    * `Tabla_Promedio_56_instancias.csb`: archivo CSV de las soluciones promedio + métricas de cada algoritmo por instancia

### Utilidades opcionales
* `Graficos`: gráficos de comparación de la best run de cada algoritmo
* `Mapper`: visualización de rutas de cierta ejecución
No se aplicaron al proyecto puesto que el análisis se realizó sobre el promedio de métricas de las 10 ejecuciones por instancia, en lugar de ejecuciones puntuales.

### Abandonado / Próximo a eliminar
* `DQN_Pipeline`: archivos residuales de una prueba con DQN.
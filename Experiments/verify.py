import os
import glob
import math

BENCHMARK_DIR = "../solomon-100"
RESULTS_DIR = "../Results"

def cargar_instancia_solomon(ruta_archivo):
    """Lee el archivo Solomon y extrae la capacidad y los datos de los clientes."""
    clientes = {}
    capacidad_maxima = 0
    
    with open(ruta_archivo, 'r') as f:
        lineas = f.readlines()
        
        for i, linea in enumerate(lineas):
            if "VEHICLE" in linea:
                partes = lineas[i+2].split()
                if len(partes) >= 2:
                    capacidad_maxima = float(partes[1])
                break
                
        inicio_clientes = 0
        for i, linea in enumerate(lineas):
            if "CUST NO." in linea:
                inicio_clientes = i + 1
                break
                
        for linea in lineas[inicio_clientes:]:
            partes = linea.split()
            if len(partes) >= 7:
                id_cliente = int(partes[0])
                clientes[id_cliente] = {
                    'x': float(partes[1]),
                    'y': float(partes[2]),
                    'demanda': float(partes[3]),
                    'ready_time': float(partes[4]),
                    'due_date': float(partes[5]),
                    'service_time': float(partes[6])
                }
    return capacidad_maxima, clientes

def distance(c1, c2):
    return math.sqrt((c1['x'] - c2['x'])**2 + (c1['y'] - c2['y'])**2)

def verificar_ruta(ruta_nodos, capacidad_maxima, clientes):
    """Verifica capacidad y ventanas de tiempo para una sola ruta."""
    carga_actual = 0.0
    tiempo_actual = 0.0
    
    for i in range(len(ruta_nodos) - 1):
        actual = ruta_nodos[i]
        siguiente = ruta_nodos[i+1]
        
        nodo_actual = clientes[actual]
        nodo_siguiente = clientes[siguiente]
        
        if siguiente != 0:
            carga_actual += nodo_siguiente['demanda']
            if carga_actual > capacidad_maxima:
                return False, f"Exceso de capacidad: {carga_actual} > {capacidad_maxima}"
        
        distancia = distance(nodo_actual, nodo_siguiente)
        tiempo_llegada = tiempo_actual + nodo_actual['service_time'] + distancia
        tiempo_inicio_servicio = max(tiempo_llegada, nodo_siguiente['ready_time'])

        if tiempo_inicio_servicio > nodo_siguiente['due_date']:
            return False, f"Ventana violada en nodo {siguiente}: Llegó {tiempo_inicio_servicio:.2f}, Cierre {nodo_siguiente['due_date']}"
            
        tiempo_actual = tiempo_inicio_servicio
        
    return True, "OK"

def auditar_experimentos():
    print("=== Iniciando Auditoría Externa de Factibilidad ===")
    rutas_csv = glob.glob(os.path.join(RESULTS_DIR, "**", "routes", "*.csv"), recursive=True)
    
    total_archivos = len(rutas_csv)
    archivos_corruptos = 0
    
    for archivo_csv in rutas_csv:
        nombre_base = os.path.basename(archivo_csv)
        inst_name = nombre_base.split('_')[1]
        
        txt_path = glob.glob(os.path.join(BENCHMARK_DIR, "**", f"{inst_name}.txt"), recursive=True)
        if not txt_path:
            continue
            
        capacidad, clientes = cargar_instancia_solomon(txt_path[0])
        factible_global = True
        
        with open(archivo_csv, 'r') as f:
            for linea in f:
                partes = linea.strip().split(',')
                if len(partes) < 3: 
                    continue
                    
                ruta_nodos = [int(x) for x in partes[1:]]
                
                es_valida, mensaje = verificar_ruta(ruta_nodos, capacidad, clientes)
                if not es_valida:
                    print(f"[!] INFACTIBLE en {nombre_base} | {partes[0]} | Error: {mensaje}")
                    factible_global = False
                    archivos_corruptos += 1
                    break
                    
    print("\n=== Resumen de Auditoría ===")
    print(f"Total de corridas analizadas: {total_archivos}")
    if archivos_corruptos == 0:
        print("RESULTADO: PERFECTO. El 100% de las soluciones generadas son estrictamente factibles.")
    else:
        print(f"RESULTADO: Se encontraron {archivos_corruptos} soluciones infactibles ({(archivos_corruptos/total_archivos)*100:.2f}%).")

if __name__ == "__main__":
    auditar_experimentos()
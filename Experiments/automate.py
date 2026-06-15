import os
import subprocess
import time
import glob
import re
import csv
from concurrent.futures import ProcessPoolExecutor, as_completed

EXEC_PATH = "../build/ALNS_vrptw.exe"
BENCHMARK_DIR = "../solomon-100"
RESULTS_DIR = "../Results"

ALGORITMOS = ["CLASSIC", "QLEARNING"]
ITERACIONES = 25000
RUNS = 10
MAX_WORKERS = 8

def preparar_directorios():
    print("Verificando estructura de directorios...")
    for algo in ALGORITMOS:
        metrics_dir = os.path.join(RESULTS_DIR, algo, "metrics")
        routes_dir = os.path.join(RESULTS_DIR, algo, "routes")
        os.makedirs(metrics_dir, exist_ok=True)
        os.makedirs(routes_dir, exist_ok=True)
        print(f"  -> Creado/Verificado: {metrics_dir}")
        print(f"  -> Creado/Verificado: {routes_dir}")

def obtener_instancias():
    patron = os.path.join(BENCHMARK_DIR, "**", "*.txt")
    instancias = glob.glob(patron, recursive=True)
    return sorted([inst for inst in instancias if os.path.basename(inst).lower().startswith(('c', 'r', 'rc'))])

def ejecutar_run_individual(inst_path, inst_name, algo, run):
    comando = [
        EXEC_PATH,
        inst_path,
        algo,
        str(ITERACIONES),
        str(run)
    ]

    try:
        resultado = subprocess.run(comando, capture_output=True, text=True, check=True)
        stdout = resultado.stdout 
        match = re.search(r"Tiempo de CPU real:\s*([0-9.]+)\s*segundos", stdout)
        if match:
            tiempo_cpu = float(match.group(1))
        else:
            tiempo_cpu = -1.0 
        return {
            "Exito": True,
            "Datos": {"Instancia": inst_name.lower(), "Algoritmo": algo, "Run": run, "Tiempo_s": tiempo_cpu},
            "Error": None
        }
    
    except subprocess.CalledProcessError as e:
        return {
            "Exito": False,
            "Datos": {"Instancia": inst_name.lower(), "Algoritmo": algo, "Run": run, "Tiempo_s": 0.0},
            "Error": e.stderr
        }

def ejecutar_experimentos():
    preparar_directorios()
    instancias = obtener_instancias()
    if not instancias:
        print(f"[ERROR] No se encontraron instancias válidas en {BENCHMARK_DIR}")
        return

    tareas = []
    for inst_path in instancias:
        inst_name = os.path.basename(inst_path).replace('.txt', '')
        for algo in ALGORITMOS:
            for run in range(1, RUNS + 1):
                tareas.append((inst_path, inst_name, algo, run))

    total_runs = len(tareas)
    registro_tiempos = []
    
    print(f"\n=== Iniciando experimentos: {len(instancias)} instancias | {total_runs} ejecuciones ===")
    print(f"=== Utilizando {MAX_WORKERS} procesos en paralelo ===")
    start_total = time.time()

    completados = 0
    with ProcessPoolExecutor(max_workers=MAX_WORKERS) as executor:
        futuros = {executor.submit(ejecutar_run_individual, *args): args for args in tareas}
        
        for futuro in as_completed(futuros):
            completados += 1
            resultado = futuro.result()
            
            if resultado["Exito"]:
                datos = resultado["Datos"]
                registro_tiempos.append(datos)
                print(f"[{completados}/{total_runs}] OK -> {datos['Algoritmo']} | {datos['Instancia']} | Run: {datos['Run']} | {datos['Tiempo_s']}s")
            else:
                args = futuros[futuro]
                print(f"[{completados}/{total_runs}] ERROR en {args[2]} | {args[1]} | Run: {args[3]}\nDetalle: {resultado['Error']}")

    end_total = time.time()

    csv_path = os.path.join(RESULTS_DIR, "tiempos_ejecucion_limpios.csv")
    with open(csv_path, mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=["Instancia", "Algoritmo", "Run", "Tiempo_s"])
        writer.writeheader()
        writer.writerows(registro_tiempos)

    horas, rem = divmod(end_total - start_total, 3600)
    minutos, segundos = divmod(rem, 60)
    print(f"\n=== Experimentos terminados en {int(horas)}h {int(minutos)}m {segundos:.2f}s ===")
    print(f"-> Archivo de tiempos guardado con éxito en: {csv_path}")

if __name__ == "__main__":
    ejecutar_experimentos()
import os
import subprocess
import time
import glob
import re
import csv

EXEC_PATH = "../build/ALNS_vrptw.exe"
BENCHMARK_DIR = "../solomon-100"
RESULTS_DIR = "../Results"

ALGORITMOS = ["CLASSIC", "QLEARNING"]
ITERACIONES = 25000
RUNS = 10

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

    total_instancias = len(instancias)
    total_runs = total_instancias * len(ALGORITMOS) * RUNS
    run_actual = 0
    
    registro_tiempos = []

    print(f"\n=== Iniciando experimentos SECUENCIALES: {total_instancias} instancias | {total_runs} ejecuciones ===")
    start_total = time.time()
    
    for inst_path in instancias:
        inst_name = os.path.basename(inst_path).replace('.txt', '')
        
        for algo in ALGORITMOS:
            for run in range(1, RUNS + 1):
                run_actual += 1
                
                comando = [
                    EXEC_PATH,
                    inst_path,
                    algo,
                    str(ITERACIONES),
                    str(run)
                ]
                
                print(f"[{run_actual}/{total_runs}] Ejecutando {algo} | Instancia: {inst_name} | Run: {run}...", end="", flush=True)
                
                try:
                    resultado = subprocess.run(comando, capture_output=True, text=True, check=True)
                    
                    match = re.search(r"Tiempo de CPU real:\s*([0-9.]+)\s*segundos", resultado.stdout)
                    tiempo_cpu = float(match.group(1)) if match else -1.0
                    
                    print(f" [OK] ({tiempo_cpu}s CPU)")

                    registro_tiempos.append({
                        "Instancia": inst_name.lower(),
                        "Algoritmo": algo,
                        "Run": run,
                        "Tiempo_s": tiempo_cpu
                    })
                    
                except subprocess.CalledProcessError as e:
                    print(f" [ERROR FATAL]")
                    print(e.stderr)
                    continue 

    end_total = time.time()

    csv_path = os.path.join(RESULTS_DIR, "tiempos_ejecucion_limpios.csv")
    with open(csv_path, mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=["Instancia", "Algoritmo", "Run", "Tiempo_s"])
        writer.writeheader()
        writer.writerows(registro_tiempos)

    horas, rem = divmod(end_total - start_total, 3600)
    minutos, segundos = divmod(rem, 60)
    print(f"\n=== Experimentos terminados exitosamente en {int(horas)}h {int(minutos)}m {segundos:.2f}s (Tiempo Wall-Clock Total) ===")
    print(f"-> Archivo de tiempos guardado con éxito en: {csv_path}")

if __name__ == "__main__":
    ejecutar_experimentos()
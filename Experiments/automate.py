import os
import subprocess
import time
import glob
import csv

EXEC_PATH = "../build/Release/ALNS_VRPTW.exe"
BENCHMARK_DIR = "../solomon-100"
RESULTS_DIR = "../Results"

ALGORITMOS = ["CLASSIC", "QLEARNING", "DQN"]
ITERACIONES = 25000
RUNS = 5

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
    return sorted(instancias)

def ejecutar_experimentos():
    preparar_directorios()
    instancias = obtener_instancias()
    if not instancias:
        print(f"[ERROR] No se encontraron instancias en {BENCHMARK_DIR}")
        return

    total_instancias = len(instancias)
    total_runs = total_instancias * len(ALGORITMOS) * RUNS
    run_actual = 0
    
    registro_tiempos = []

    print(f"\n=== Iniciando experimentos: {total_instancias} instancias | {total_runs} ejecuciones ===")
    start_total = time.time()
    
    for inst_path in instancias:
        inst_name = os.path.basename(inst_path).replace('.txt', '')
        
        for algo in ALGORITMOS:
            for run in range(1, RUNS + 1):
                run_actual += 1
                start_run = time.perf_counter()
                
                print(f"[{run_actual}/{total_runs}] Ejecutando {algo} | Instancia: {inst_name} | Run: {run}...", end="", flush=True)
                
                comando = [
                    EXEC_PATH,
                    inst_path,       # argv[1]: Ruta a la instancia
                    algo,            # argv[2]: CLASSIC o QLEARNING
                    str(ITERACIONES),# argv[3]: 25000
                    str(run)         # argv[4]: ID de la corrida
                ]
                
                try:
                    resultado = subprocess.run(comando, capture_output=True, text=True, check=True)
                    end_run = time.perf_counter()
                    time_run = end_run - start_run
                    print(f" [OK] ({(end_run - start_run):.2f} seg)")

                    registro_tiempos.append({
                        "Instancia": inst_name.lower(),
                        "Algoritmo": algo,
                        "Run": run,
                        "Tiempo_s": round(time_run, 4)
                    })
                    
                except subprocess.CalledProcessError as e:
                    print(f" [ERROR FATAL]")
                    print(e.stderr)
                    continue 

    end_total = time.time()

    horas, rem = divmod(end_total - start_total, 3600)
    minutos, segundos = divmod(rem, 60)
    print(f"\n=== Experimentos terminados en {int(horas)}h {int(minutos)}m {segundos:.2f}s ===")

if __name__ == "__main__":
    ejecutar_experimentos()
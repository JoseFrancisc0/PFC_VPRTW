import os
import subprocess
import time
import glob
import csv
import re
import numpy as np

EXEC_PATH = "../build/Release/ALNS_VRPTW.exe"
BENCHMARK_DIR = "../solomon-100"
RESULTS_DIR = "../Results"
SINTEF_CSV = "sintef.csv"

ALGORITMOS = ["CLASSIC", "QLEARNING", "DQN"]
ITERACIONES = 25000
RUNS = 10

def cargar_optimos_sintef():
    optimos = {}
    if not os.path.exists(SINTEF_CSV):
        print(f"[ERROR] No se encontró {SINTEF_CSV}")
        return optimos
    
    with open(SINTEF_CSV, 'r', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f, delimiter=';')
        for row in reader:
            inst = row['Instancia'].lower().strip()
            veh = float(row['Vehículos'])
            dist = float(row['Distancia'])
            optimos[inst] = {'opt_veh': veh, 'opt_dist': dist}
            
    if 'r110' in optimos and optimos['r110']['opt_dist'] == 118.84:
        optimos['r110']['opt_dist'] = 1118.84
    return optimos

def obtener_instancias():
    patron = os.path.join(BENCHMARK_DIR, "**", "*.txt")
    instancias = glob.glob(patron, recursive=True)
    return sorted(instancias)

def ejecutar_experimentos():
    optimos = cargar_optimos_sintef()
    instancias = obtener_instancias()
    if not instancias:
        print(f"[ERROR] No se encontraron instancias en {BENCHMARK_DIR}")
        return

    # Preparar archivos CSV
    csv_files = {}
    for algo in ALGORITMOS:
        filename = f"{algo}_Evaluation_vs_SINTEF.csv"
        f = open(filename, 'w', newline='', encoding='utf-8')
        writer = csv.writer(f)
        
        headers = ['Instancia', 'Opt_Veh', 'Opt_Dist']
        for i in range(1, RUNS + 1):
            headers.extend([f'Run{i}_Veh', f'Run{i}_Dist'])
        headers.extend(['Best_Veh', 'Avg_Veh', 'Std_Veh', 'Best_Dist', 'Avg_Dist', 'Std_Dist', 'Veh_GAP', 'Dist_GAP', 'Unified_GAP', 'Avg_Time(s)'])
        
        writer.writerow(headers)
        csv_files[algo] = (f, writer)

    total_instancias = len(instancias)
    total_runs = total_instancias * len(ALGORITMOS) * RUNS
    run_actual = 0
    start_total = time.time()
    
    print(f"\n=== Iniciando experimentos rapidos: {total_instancias} instancias | {total_runs} ejecuciones ===")

    for inst_path in instancias:
        inst_name = os.path.basename(inst_path).replace('.txt', '').lower()
        if inst_name not in optimos:
            continue
            
        opt_veh = optimos[inst_name]['opt_veh']
        opt_dist = optimos[inst_name]['opt_dist']

        for algo in ALGORITMOS:
            run_results = []
            
            for run in range(1, RUNS + 1):
                run_actual += 1
                start_run = time.perf_counter()
                
                print(f"[{run_actual}/{total_runs}] {algo} | {inst_name.upper()} | Run: {run}...", end="", flush=True)
                
                comando = [
                    EXEC_PATH,
                    inst_path,       
                    algo,            
                    str(ITERACIONES),
                    "BENCHMARK"      # Flag para saltar el guardado de experiencias y CSVs iterativos en disco
                ]
                
                try:
                    resultado = subprocess.run(comando, capture_output=True, text=True, check=True)
                    end_run = time.perf_counter()
                    
                    # Parsear stdout buscando la etiqueta [FINAL_RESULT]
                    match = re.search(r'\[FINAL_RESULT\] Veh: (\d+), Dist: ([\d.]+)', resultado.stdout)
                    if match:
                        v = int(match.group(1))
                        d = float(match.group(2))
                        run_time = end_run - start_run
                        run_results.append((v, d, run_time))
                        print(f" [OK] ({run_time:.2f}s) -> v:{v} d:{d:.2f}")
                    else:
                        print(f" [ERR PARSE] ({(end_run - start_run):.2f}s)")
                        
                except subprocess.CalledProcessError as e:
                    print(f" [ERROR FATAL]")
                    continue 

            if not run_results:
                continue

            # Calcular estadisticas
            vehs = [r[0] for r in run_results]
            dists = [r[1] for r in run_results]
            times = [r[2] for r in run_results]
            
            avg_veh = np.mean(vehs)
            std_veh = np.std(vehs, ddof=0)
            avg_dist = np.mean(dists)
            std_dist = np.std(dists, ddof=0)
            avg_time = np.mean(times)
            
            # Ordenamiento Lexicografico
            sorted_results = sorted(run_results, key=lambda x: (x[0], x[1]))
            best_veh, best_dist = sorted_results[0][0], sorted_results[0][1]
            
            veh_gap = avg_veh - opt_veh
            dist_gap = ((avg_dist - opt_dist) / opt_dist) * 100 if opt_dist > 0 else 0
            unified_gap = (veh_gap * 100) + dist_gap
            
            # Generar fila
            row = [inst_name.upper(), opt_veh, opt_dist]
            for r in run_results:
                row.extend([r[0], f"{r[1]:.2f}"])
            row.extend([
                best_veh, f"{avg_veh:.2f}", f"{std_veh:.3f}",
                f"{best_dist:.2f}", f"{avg_dist:.2f}", f"{std_dist:.2f}",
                f"{veh_gap:.2f}", f"{dist_gap:.2f}", f"{unified_gap:.2f}",
                f"{avg_time:.2f}"
            ])
            
            # Escribir y forzar el volcado a disco para evitar perdidas
            csv_files[algo][1].writerow(row)
            csv_files[algo][0].flush()

    for algo in ALGORITMOS:
        csv_files[algo][0].close()

    end_total = time.time()
    horas, rem = divmod(end_total - start_total, 3600)
    minutos, segundos = divmod(rem, 60)
    print(f"\n=== Experimentos terminados en {int(horas)}h {int(minutos)}m {segundos:.2f}s ===")

if __name__ == "__main__":
    ejecutar_experimentos()
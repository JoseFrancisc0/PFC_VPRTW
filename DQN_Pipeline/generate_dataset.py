import os
import subprocess
import time
import glob

EXEC_PATH = "../build/Release/ALNS_VRPTW.exe"
BENCHMARK_DIR = "../solomon-100"
EXPERIENCES_DIR = "experiences"

ALGORITMO = "CLASSIC"
ITERACIONES = 25000

def preparar_directorios():
    os.makedirs(EXPERIENCES_DIR, exist_ok=True)
    print(f"-> Creado/Verificado: {EXPERIENCES_DIR}")

def obtener_instancias():
    patron = os.path.join(BENCHMARK_DIR, "**", "*.txt")
    instancias = glob.glob(patron, recursive=True)
    return sorted(instancias)

def generar_datos():
    preparar_directorios()
    instancias = obtener_instancias()
    if not instancias:
        print(f"[ERROR] No se encontraron instancias en {BENCHMARK_DIR}")
        return

    print(f"\n=== Iniciando Extraccion de Datos para DQN ({len(instancias)} instancias) ===")
    
    for i, inst_path in enumerate(instancias, 1):
        inst_name = os.path.basename(inst_path).replace('.txt', '')
        
        print(f"[{i}/{len(instancias)}] Extrayendo experiencias de {inst_name}... ", end="", flush=True)
        start_run = time.perf_counter()
        
        comando = [
            EXEC_PATH,
            inst_path,
            ALGORITMO,
            str(ITERACIONES),
            "GENERATE_DATA"  # Este flag enciende la recoleccion profunda en C++
        ]
        
        try:
            subprocess.run(comando, capture_output=True, text=True, check=True)
            end_run = time.perf_counter()
            print(f"[OK] ({(end_run - start_run):.2f}s) -> experiences_{inst_name}_dataset.csv")
        except subprocess.CalledProcessError as e:
            print(f"[ERROR FATAL]")
            continue

if __name__ == "__main__":
    generar_datos()

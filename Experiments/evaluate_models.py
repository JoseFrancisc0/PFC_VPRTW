import os
import glob
import csv
import re
import matplotlib.pyplot as plt
import numpy as np

RESULTS_DIR = "../Results"
SINTEF_CSV = "sintef.csv"
ALGORITMOS = ["CLASSIC", "QLEARNING", "DQN"]
CLASSES = ["c1", "c2", "r1", "r2", "rc1", "rc2"]

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
            
    # FIX R110 typo if present
    if 'r110' in optimos and optimos['r110']['opt_dist'] == 118.84:
        optimos['r110']['opt_dist'] = 1118.84
        
    return optimos

def procesar_resultados():
    optimos = cargar_optimos_sintef()
    instancias_sintef = list(optimos.keys())
    
    datos_agregados = {algo: {} for algo in ALGORITMOS}
    
    for algo in ALGORITMOS:
        print(f"--- Procesando datos de {algo} ---")
        
        for inst in sorted(instancias_sintef):
            pattern = os.path.join(RESULTS_DIR, algo, "metrics", f"{algo}_{inst}_metrics_run*.csv")
            metrics_files = glob.glob(pattern)
            
            run_results = []
            
            for mf in metrics_files:
                match = re.search(r'_run(\d+)\.csv', mf)
                if not match: continue
                run_id = int(match.group(1))
                
                # IGNORAR corridas manuales (run0) u otras anómalas, solo nos interesan las automatizadas
                if run_id == 0: continue 
                
                best_veh_run = float('inf')
                best_dist_run = float('inf')
                
                try:
                    with open(mf, 'r', encoding='utf-8') as f:
                        reader = csv.reader(f)
                        next(reader, None) # skip header
                        for row in reader:
                            if len(row) >= 3:
                                v = float(row[1]) # best_veh
                                d = float(row[2]) # best_dist
                                # Guardar siempre la que tenga menos vehículos, o a igual vehículos, menor distancia
                                if v < best_veh_run or (v == best_veh_run and d < best_dist_run):
                                    best_veh_run = v
                                    best_dist_run = d
                                    
                    if best_veh_run != float('inf'):
                        run_results.append((best_veh_run, best_dist_run))
                except Exception as e:
                    print(f"Error procesando {mf}: {e}")
                    
            if not run_results:
                continue
                
            # Ordenamiento Lexicográfico (Mejor Vehiculos, luego Mejor Distancia entre todas las corridas)
            sorted_results = sorted(run_results, key=lambda x: (x[0], x[1]))
            best_veh = sorted_results[0][0]
            best_dist = sorted_results[0][1] # Esta es la "Mejor distancia PARA el menor número de vehículos"
            
            avg_veh = sum(r[0] for r in run_results) / len(run_results)
            avg_dist = sum(r[1] for r in run_results) / len(run_results)
            
            opt_veh = optimos[inst]['opt_veh']
            opt_dist = optimos[inst]['opt_dist']
            
            veh_gap = avg_veh - opt_veh
            dist_gap = 0.0
            if opt_dist > 0:
                dist_gap = ((avg_dist - opt_dist) / opt_dist) * 100
                
            unified_gap = (veh_gap * 100) + dist_gap
            
            datos_agregados[algo][inst] = {
                'avg_veh': avg_veh,
                'avg_dist': avg_dist,
                'best_veh': best_veh,
                'best_dist': best_dist, # Guardado para el gráfico individual
                'opt_veh': opt_veh,
                'opt_dist': opt_dist,
                'veh_gap': veh_gap,
                'dist_gap': dist_gap,
                'unified_gap': unified_gap,
                'hits': 1 if avg_veh == opt_veh else 0 # Evaluacion perfecta de vehiculos
            }

    return datos_agregados

def imprimir_y_guardar_resumen(datos_agregados):
    print("\n" + "="*70)
    print(" VEREDICTO FINAL: ALNS CLÁSICO vs Q-LEARNING vs DQN")
    print("="*70)
    
    # Encontrar instancias comunes en los 3 algoritmos
    instancias_comunes = set(datos_agregados[ALGORITMOS[0]].keys())
    for algo in ALGORITMOS[1:]:
        instancias_comunes &= set(datos_agregados[algo].keys())
    
    instancias_comunes = sorted(list(instancias_comunes))
    total_inst = len(instancias_comunes)
    
    if total_inst == 0:
        print("[!] No hay instancias en común entre los algoritmos para comparar.")
        return None, None
        
    print(f"Instancias evaluadas en conjunto: {total_inst}")
    print("-" * 70)
    
    resumen_graficos = {algo: {} for algo in ALGORITMOS}
    
    for name in ALGORITMOS:
        data = datos_agregados[name]
        
        avg_veh_gap = sum(data[inst]['veh_gap'] for inst in instancias_comunes) / total_inst
        avg_dist_gap = sum(data[inst]['dist_gap'] for inst in instancias_comunes) / total_inst
        avg_unif_gap = sum(data[inst]['unified_gap'] for inst in instancias_comunes) / total_inst
        hits = sum(data[inst]['hits'] for inst in instancias_comunes)
        
        resumen_graficos[name] = {
            'avg_veh_gap': avg_veh_gap,
            'avg_dist_gap': avg_dist_gap,
            'avg_unif_gap': avg_unif_gap
        }
        
        print(f"[{name}]")
        print(f"  - Hits de Vehículo Optimo:                    {hits}/{total_inst} ({hits/total_inst*100:.1f}%)")
        print(f"  - Castigo de Vehículos (Media Avg_Veh_GAP):   +{avg_veh_gap:.3f} vehículos extras")
        print(f"  - Castigo de Distancia (Media Avg_Dist_GAP):  {avg_dist_gap:.2f} % de exceso")
        print(f"  - GAP UNIFICADO GLOBAL (Promedio):            {avg_unif_gap:.2f} Puntos (Menor es mejor)")
        print("-" * 70)
        
    return resumen_graficos, instancias_comunes

def generar_graficos(resumen_graficos, datos_agregados, instancias_comunes):
    os.makedirs("graficos", exist_ok=True)
    
    # 1. Gráfico Combinado (GAPs)
    labels = ALGORITMOS
    x = np.arange(len(labels))
    width = 0.25
    
    veh_gaps = [resumen_graficos[algo]['avg_veh_gap'] for algo in ALGORITMOS]
    dist_gaps = [resumen_graficos[algo]['avg_dist_gap'] for algo in ALGORITMOS]
    unif_gaps = [resumen_graficos[algo]['avg_unif_gap'] for algo in ALGORITMOS]
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    rects1 = ax1.bar(x - width, veh_gaps, width, label='Vehicles GAP (+x extras)', color='lightblue')
    rects2 = ax1.bar(x, dist_gaps, width, label='Distance GAP (%)', color='lightgreen')
    
    ax2 = ax1.twinx()
    rects3 = ax2.bar(x + width, unif_gaps, width, label='Unified GAP (Score)', color='salmon')
    
    ax1.set_ylabel('GAPs Físicos (Vehículos / % Distancia)')
    ax2.set_ylabel('GAP Unificado (Score)', color='darkred')
    ax1.set_title('Comparación de GAPs por Algoritmo (Menor es mejor)')
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels)
    
    lines, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines + lines2, labels1 + labels2, loc='upper left')
    
    plt.savefig('graficos/Comparacion_GAP_General.png', bbox_inches='tight', dpi=300)
    plt.close()
    
    # 2. Gráficos Individuales por Clase (C1, C2, R1, R2, RC1, RC2)
    for cls in CLASSES:
        insts_cls = [inst for inst in instancias_comunes if inst.startswith(cls)]
        if not insts_cls: continue
        
        x_inst = np.arange(len(insts_cls))
        width_cls = 0.25
        
        fig, ax = plt.subplots(figsize=(max(10, len(insts_cls)*0.8), 6))
        
        for i, algo in enumerate(ALGORITMOS):
            best_dists = [datos_agregados[algo][inst]['best_dist'] for inst in insts_cls]
            offset = (i - 1) * width_cls
            ax.bar(x_inst + offset, best_dists, width_cls, label=algo)
            
        opt_dists = [datos_agregados[ALGORITMOS[0]][inst]['opt_dist'] for inst in insts_cls]
        ax.plot(x_inst, opt_dists, 'kX', markersize=8, label='Óptimo (SINTEF)', zorder=5)
            
        ax.set_ylabel('Mejor Distancia Alcanzada')
        ax.set_title(f'Mejor Distancia (Mínimo N° Vehículos) - Clase {cls.upper()}')
        ax.set_xticks(x_inst)
        ax.set_xticklabels([i.upper() for i in insts_cls], rotation=45)
        ax.legend()
        
        plt.tight_layout()
        plt.savefig(f'graficos/Comparacion_Instancias_{cls.upper()}.png', bbox_inches='tight', dpi=300)
        plt.close()
        
    print("-> Gráficos generados exitosamente en la subcarpeta 'graficos/'.")

if __name__ == "__main__":
    datos = procesar_resultados()
    resumen, comunes = imprimir_y_guardar_resumen(datos)
    if resumen:
        generar_graficos(resumen, datos, comunes)

import os
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from adjustText import adjust_text

def leer_instancia_solomon(ruta_archivo):
    nodos = {}
    leyendo_clientes = False
    
    with open(ruta_archivo, 'r') as f:
        for linea in f:
            partes = linea.strip().split()
            if len(partes) >= 7 and partes[0].isdigit():
                leyendo_clientes = True
                
            if leyendo_clientes and len(partes) >= 7:
                id_nodo = int(partes[0])
                nodos[id_nodo] = {
                    'x': float(partes[1]),
                    'y': float(partes[2]),
                    'ready': int(float(partes[4])),
                    'due': int(float(partes[5]))
                }
    return nodos

def leer_rutas_csv(ruta_archivo):
    rutas = []
    with open(ruta_archivo, 'r') as f:
        for linea in f:
            if linea.strip():
                nodos_str = linea.strip().split(',')[1:]
                rutas.append([int(n) for n in nodos_str])
    return rutas

def plotear_mapa_2d(instancia_txt, ruta_csv, archivo_salida):
    print(f"Generando mapa para: {os.path.basename(instancia_txt)}")
    nodos = leer_instancia_solomon(instancia_txt)
    rutas = leer_rutas_csv(ruta_csv)
    plt.figure(figsize=(16, 10))
    
    # Dibujando nodos
    xs = [n['x'] for id_n, n in nodos.items() if id_n != 0]
    ys = [n['y'] for id_n, n in nodos.items() if id_n != 0]
    plt.scatter(xs, ys, c='#888888', s=30, zorder=2)
    
    # Dibujando clientes
    plt.scatter(nodos[0]['x'], nodos[0]['y'], c='red', s=150, marker='s', edgecolors='black', label='Depósito', zorder=4)
    plt.text(nodos[0]['x'], nodos[0]['y'] + 1.5, "DEPÓSITO\n[0, 0]", 
             ha='center', fontsize=9, fontweight='bold', color='red', zorder=5)
    
    # Etiquetas (Time Windows)
    textos_a_dibujar = []
    for id_nodo, datos in nodos.items():
        if id_nodo != 0:
            etiqueta = f"[{datos['ready']}, {datos['due']}]"
            obj_texto = plt.text(datos['x'], datos['y'], etiqueta, 
                                 fontsize=6, color='black', alpha=0.9)
            textos_a_dibujar.append(obj_texto)

    ax = plt.gca()
    
    print("Ajustando posiciones de etiquetas (esto puede tomar unos segundos)...")
    adjust_text(textos_a_dibujar,
                ax=ax,
                arrowprops=dict(arrowstyle="-", color='gray', lw=0.5, alpha=0.5),
                expand_points=(1.2, 1.2),
                expand_text=(1.05, 1.05))
    
    # Recorridos
    colores = cm.tab20(np.linspace(0, 1, len(rutas)))
    
    for i, recorrido in enumerate(rutas):
        color = colores[i]
        
        rx = [nodos[n]['x'] for n in recorrido]
        ry = [nodos[n]['y'] for n in recorrido]
        
        plt.plot(rx, ry, color=color, linewidth=1.5, alpha=0.6, label=f'Vehículo {i+1}')

        for j in range(len(recorrido) - 1):
            nodo_actual = nodos[recorrido[j]]
            nodo_siguiente = nodos[recorrido[j+1]]
            
            dx = nodo_siguiente['x'] - nodo_actual['x']
            dy = nodo_siguiente['y'] - nodo_actual['y']
            
            plt.arrow(nodo_actual['x'], nodo_actual['y'], dx, dy, 
                      color=color, alpha=0.7, length_includes_head=True, 
                      head_width=0.5, head_length=0.5, zorder=3)
            
    plt.title(f'Topología de Rutas y Ventanas de Tiempo (VRPTW)\nInstancia: {os.path.basename(instancia_txt)}', fontsize=14, fontweight='bold')
    plt.xlabel('Coordenada X', fontsize=11)
    plt.ylabel('Coordenada Y', fontsize=11)
    
    plt.legend(bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0.)
    plt.grid(True, linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(archivo_salida, dpi=300)
    plt.close()
    print(f"-> Imagen guardada exitosamente como: {archivo_salida}")

if __name__ == "__main__":
    ARCHIVO_INSTANCIA = "../solomon-100/c1/c101.txt"
    
    ARCHIVO_RUTAS = "../Results/QLEARNING/routes/QLEARNING_c101_metrics_run1.csv"
    
    ARCHIVO_SALIDA = "Mapa_Rutas_C101_QLearning.png"
    
    plotear_mapa_2d(ARCHIVO_INSTANCIA, ARCHIVO_RUTAS, ARCHIVO_SALIDA)
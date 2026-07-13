# Guía de parámetros — Follow the Gap

Apuntes de tuning para los dos controladores del tutorial: **Parte 1** (`gap_node.py`, FTG normal) y **Parte 2** (`gap_rebase_node.py`, FTG con obstáculos estáticos y dinámicos).

| Nodo | Archivo | Comando |
|------|---------|---------|
| Parte 1 — FTG Normal | `gap_node.py` | `ros2 run controllers gap_node` |
| Parte 2 — FTG con Obstáculos | `gap_rebase_node.py` | `ros2 run controllers gap_rebase_node` |

> Recuerda: tras editar un archivo → `colcon build && source install/setup.bash`

**Cómo usar esta guía:** los parámetros viven al inicio de cada nodo (en `__init__`). Cambia **uno a la vez**, recompila, corre el nodo y observa el tiempo por vuelta. Las tablas de abajo te dicen qué esperar al subir o bajar cada valor. Si buscas la explicación conceptual del algoritmo, mírala primero en el [README](README.md).

---

## Parte 1 — `gap_node.py`

### 1. Geometría del LIDAR (base de todo)

- **FOV total:** 4.70 rad ≈ 270° · **1080 rayos** · `angle_increment ≈ 0.00435 rad`
- **Índice → ángulo:** `angulo = angle_min + indice * angle_increment`

| Índice | Ángulo | Dirección |
|--------|--------|-----------|
| 0 | -2.35 rad (-135°) | derecha |
| 540 | 0 rad (0°) | **frente** |
| 1079 | +2.35 rad (+135°) | izquierda |

Convención de manejo: `steering_angle` positivo = izquierda, negativo = derecha. Máximo físico ≈ **±0.4 rad**.

### 2. Cómo funciona Follow the Gap

**Idea:** encontrar el espacio libre más grande al frente y dirigirse a su centro.

**Los pasos:**
1. **Preprocesar** — recortar rango máx, suavizar (media móvil), limpiar inf/NaN.
2. **Punto más cercano** — el obstáculo más peligroso (`argmin`).
3. **Burbuja de seguridad** — poner a cero los rayos vecinos al más cercano (el carro tiene ancho → no cabe en huecos diminutos).
4. **Gap más grande** — la secuencia continua de rayos "libres" más larga.
5. **Punto objetivo** — el centro del gap (más estable que el punto más lejano).
6. **Velocidad proporcional** — interpolar entre `vel_recta` y `vel_curva` según el giro.

**Puntos clave:**
- **Reactivo = sin memoria.** Cada scan se procesa desde cero.
- La burbuja existe porque **el robot no es un punto.**
- La oscilación en rectas se combate con **zona muerta + filtro pasa-bajos.**

### 3. Tabla de parámetros — Navegación

| Parámetro | Valor | Si lo SUBES | Si lo BAJAS |
|-----------|-------|-------------|-------------|
| `rango_max` | 3.0 m | Ve gaps más lejanos, anticipa más | Más "miope", solo reacciona a lo cercano |
| `radio_burbuja` | 80 | Se aleja de paredes, curvas abiertas; puede tapar gaps válidos | Pasa más cerca, aprovecha huecos chicos; riesgo de raspar |
| `ventana_suavizado` | 5 | Scan más limpio, borra detalles finos | Más sensible al ruido |
| `umbral_gap` | 1.0 m | Más exigente, gaps cortos/conservadores | Más permisivo, entra a espacios estrechos |
| `fov_recorte` | 100° | Ve más a los lados (curvas cerradas); puede desviarse | Más enfocado al frente, estable; tarda en ver salida de curva |

### 4. Tabla de parámetros — Velocidad y suavizado

| Parámetro | Valor | Si lo SUBES | Si lo BAJAS |
|-----------|-------|-------------|-------------|
| `vel_recta` | 7.5 m/s | Más rápido en recta; menos reacción | Más seguro, vueltas lentas |
| `vel_curva` | 2.15 m/s | Curvas más rápidas; riesgo de salirse | Curvas seguras pero lentas |
| `zona_muerta` | 20° | Rectas firmes; ignora giros pequeños (peligroso si muy alto) | Reacciona a giros mínimos; más serpenteo |
| `alpha_suavizado` | 0.4 | Reacciona rápido pero brusco/oscilante | Más suave y estable; reacciona tarde en curvas |

> **La velocidad proporcional NO se ajusta directo.** Se recalcula sola entre `vel_recta` y `vel_curva` según el ángulo de giro:
> `speed = vel_recta − (vel_recta − vel_curva) · (|steering| / 0.4)`

### 5. Tabla de parámetros — Contador de vueltas

| Parámetro | Valor | Efecto |
|-----------|-------|--------|
| `umbral_lejos` | 5.0 m | Distancia mínima a alejarse antes de validar vuelta. Súbelo si cuenta vueltas falsas |
| `umbral_cerca` | 2.0 m | Qué tan cerca debe volver para contar. Muy bajo = puede no detectarla |

### 6. Relaciones entre parámetros (se compensan entre sí)

> Leyenda: `A ↑ → B ↓` significa "si subes A, normalmente conviene bajar B para mantener el equilibrio". `↔` = definen juntos un rango.

| Relación | Cómo se compensan |
|----------|-------------------|
| `vel_recta` ↑ → `alpha_suavizado` ↓ | A más velocidad, más suavizado para no serpentear |
| `vel_curva` ↑ → `radio_burbuja` ↑ | Curvas rápidas → aléjate más de las paredes |
| `fov_recorte` ↑ → `zona_muerta` ↑ | Más visión lateral mete ruido; compénsalo con zona muerta |
| `radio_burbuja` ↑ → `umbral_gap` ↓ | Si la burbuja tapa mucho, baja el umbral para hallar gaps |
| `vel_recta` ↔ `vel_curva` | Definen el rango de la velocidad proporcional |

### 7. Método de tuning recomendado

1. Cambia **un parámetro a la vez.**
2. Mide el **tiempo por vuelta** (lo imprime el nodo en la terminal) — métrica objetiva, no "se siente mejor".
3. Los 3 de mayor impacto en el tiempo: **`vel_curva`**, **`alpha_suavizado`**, **`radio_burbuja`**.
4. Para `vel_curva`: sube de a poco (2.15 → 2.5 → 3.0) hasta que se salga, luego retrocede un paso.

---

## 8. Parte 2 — `gap_rebase_node.py`

Mismo algoritmo y misma geometría de LIDAR que la Parte 1 (secciones 1 y 2 arriba). Lo que cambia son los **valores** de los parámetros y, sobre todo, **qué NO tiene**: no hay ninguna capa que frene por estar cerca de un obstáculo o del rival — la velocidad depende solo del giro (ver [README, sección 3](README.md#3-parte-2--ftg-con-obstáculos-estáticos-y-dinámicos-gap_rebase_nodepy)).

### 8.1 Tabla de parámetros — Navegación

| Parámetro | Valor | Si lo SUBES | Si lo BAJAS |
|-----------|-------|-------------|-------------|
| `rango_max` | 6.8 m | Anticipa el obstáculo/rival con más tiempo | Reacciona tarde, esquiva más brusco |
| `radio_burbuja` | 52 | Se aleja más del rival/obstáculo; puede tapar el hueco de rebase | Pasa más cerca; aprovecha mejor el carril |
| `ventana_suavizado` | 3 | Scan más limpio | Más sensible al ruido del LIDAR |
| `umbral_gap` | 1.7 m | Más exigente para considerar un hueco "libre" | Más permisivo, entra a espacios estrechos |
| `fov_recorte` | 85° | Ve más a los lados | Más enfocado al frente |

### 8.2 Tabla de parámetros — Velocidad y suavizado

| Parámetro | Valor (defaults) | Si lo SUBES | Si lo BAJAS |
|-----------|-------------------|-------------|-------------|
| `vel_recta` | 7.0 m/s (parámetro ROS) | Más rápido en recta; alcanza y rebasa antes | Más conservador |
| `vel_curva` | 1.30 m/s (parámetro ROS) | Curvas más rápidas; riesgo de rozar el obstáculo | Curvas más seguras pero lentas |
| `zona_muerta` | 1.5° | Rectas firmes | Reacciona a giros casi imperceptibles — necesario para maniobrar entre obstáculos cercanos |
| `alpha_suavizado` | 0.50 | Reacciona más rápido al hueco de rebase | Más suave; puede llegar tarde a esquivar |

> `vel_recta`/`vel_curva` son **parámetros ROS** (no están fijos en el código): se pasan por CLI. Así se lanza el mismo nodo dos veces — ego rápido y oponente lento — sin duplicar archivos:
> ```bash
> ros2 run controllers gap_rebase_node --ros-args -p vel_recta:=2.0 -p vel_curva:=1.0
> ```

### 8.3 Por qué estos valores son distintos a la Parte 1

| Diferencia | Motivo |
|------------|--------|
| `rango_max` mucho mayor (6.8 vs 3.0) | Con obstáculos y un rival dinámico, ver lejos da tiempo de elegir el hueco correcto antes de estar encima |
| `zona_muerta` mucho menor (1.5° vs 20°) | Esquivar obstáculos puntuales requiere correcciones finas que la zona muerta grande de la Parte 1 ignoraría |
| `umbral_gap` mayor (1.7 vs 1.0) | Con el mapa más "sucio" (obstáculos + rival), conviene ser más estricto sobre qué cuenta como hueco seguro |
| Sin freno por distancia/TTC | Es la razón estructural por la que **sí rebasa**: nada retiene al carro cerca del rival, así que la dirección (que ya apunta al hueco lateral) puede ejecutarse a velocidad plena |

### 8.4 Método de tuning recomendado (Parte 2)

1. Ajusta primero el **oponente** (`vel_recta`/`vel_curva` bajos) hasta tener un rival predecible y no demasiado lento.
2. Sube el `rango_max` del ego si el rebase se ve "tardío" o brusco.
3. Si el ego roza al rival al pasar, sube `radio_burbuja`; si se queda sin espacio para maniobrar, bájalo.
4. Si zigzaguea en recta entre obstáculos, sube ligeramente `zona_muerta` (con cuidado: si subes demasiado, deja de reaccionar a obstáculos angostos).

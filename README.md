# Follow the Gap — Controlador Reactivo F1TENTH

Tutorial e implementación de un controlador de navegación reactiva **Follow the Gap (FTG)** para el simulador [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym_ros) sobre **ROS 2 Humble**.

El vehículo recorre el circuito de forma autónoma usando **únicamente** los datos del LIDAR — sin mapa, sin planificación global y sin memoria entre lecturas. En cada barrido del sensor busca el espacio libre más grande frente a él y dirige el carro hacia su centro.

> **Autor:** Héctor La Mota · ESPOL
> **Nodo principal:** [`src/gap_node.py`](src/gap_node.py)

## 🎥 Demostración

El carro completando vueltas de forma autónoma con este controlador:

▶️ **[Ver video de la demostración](videos/FTG.webm)** (`videos/FTG.webm`)

<!-- GitHub reproduce el .webm al abrir el enlace. Para incrustarlo con reproductor
     dentro del README, arrastra el archivo a la edición del README en la web de GitHub. -->

---

## Tabla de contenidos

1. [Enfoque utilizado: ¿qué es Follow the Gap?](#1-enfoque-utilizado-qué-es-follow-the-gap)
2. [Cómo funciona el algoritmo (paso a paso)](#2-cómo-funciona-el-algoritmo-paso-a-paso)
3. [Estructura del código](#3-estructura-del-código)
4. [Parámetros ajustables](#4-parámetros-ajustables)
5. [Requisitos](#5-requisitos)
6. [Instrucciones de ejecución](#6-instrucciones-de-ejecución)
7. [Mejoras adicionales implementadas](#7-mejoras-adicionales-implementadas)
8. [Mejora propuesta: Disparity Extender](#8-mejora-propuesta-disparity-extender)

---

## 1. Enfoque utilizado: ¿qué es Follow the Gap?

**Follow the Gap** es un algoritmo de **navegación reactiva**: el robot decide su movimiento instante a instante a partir de lo que ve en *ese* momento, sin construir un mapa ni recordar lecturas anteriores. Es rápido, robusto y no necesita conocer la pista de antemano.

La idea central:

> En cada lectura del LIDAR, encuentra el **hueco (gap)** de espacio libre más grande frente al carro y apunta hacia su **centro**, evitando el obstáculo más cercano.

**Una analogía:** imagina que caminas por un pasillo con los ojos vendados, pero puedes estirar los brazos y sentir a qué distancia está la pared en muchas direcciones. En cada paso te giras hacia donde sientas **más espacio abierto** y caminas más rápido si el camino está despejado, más lento si hay una curva cerca. Eso es exactamente lo que hace este algoritmo, 50 veces por segundo.

### Geometría del sensor

El LIDAR del F1TENTH entrega **1080 rayos** cubriendo un FOV de **270°**:

| Índice | Ángulo | Dirección |
|--------|--------|-----------|
| 0 | −2.35 rad (−135°) | derecha |
| 540 | 0 rad (0°) | **frente** |
| 1079 | +2.35 rad (+135°) | izquierda |

Conversión índice → ángulo: `angulo = angle_min + indice * angle_increment` (con `angle_increment ≈ 0.00435 rad`).

Convención de manejo: `steering_angle` **positivo = izquierda**, **negativo = derecha**. Límite físico ≈ **±0.4 rad**.

---

## 2. Cómo funciona el algoritmo (paso a paso)

Cada vez que llega un scan (`lidar_callback`), se ejecutan estos 6 pasos:

### Paso 1 — Preprocesar el scan
- Reemplazar `inf` y `NaN` por `0` (se tratan como obstáculo).
- Recortar distancias grandes a `rango_max` (un hueco a 25 m no es más útil que uno a 3 m).
- Suavizar con una **media móvil** para eliminar ruido del sensor.

### Paso 2 — Encontrar el punto más cercano
El obstáculo más peligroso es simplemente el rayo de menor distancia (`argmin`).

### Paso 3 — Burbuja de seguridad
El robot **no es un punto**: tiene ancho. Alrededor del punto más cercano se ponen a cero `radio_burbuja` rayos a cada lado, "engordando" el obstáculo para no intentar pasar por huecos donde no cabe.

```
Distancias:  [3.0 3.0 0.5 3.0 3.0 3.0]   ← 0.5 es el más cercano
Burbuja:     [0.0 0.0 0.0 0.0 0.0 3.0]   ← se anulan sus vecinos
```

### Paso 4 — Gap más grande
Se busca la **secuencia continua más larga** de rayos "libres" (distancia > `umbral_gap`). Ese es el hueco hacia donde conviene ir.

### Paso 5 — Punto objetivo
Se elige el **centro del gap**. (El centro es más estable que el punto más lejano, que salta de lado a lado por ruido.)

### Paso 6 — Dirección y velocidad
El índice objetivo se convierte en `steering_angle`. La velocidad es **proporcional al giro**: rápido en recta, lento en curva.

### Diagrama del flujo

```
        LIDAR /scan (1080 rayos)
                 │
                 ▼
   [1] Preprocesar (limpiar + recortar + suavizar)
                 │
                 ▼
   [Recorte FOV] quedarse con el sector frontal (±100°)
                 │
                 ▼
   [2] Punto más cercano (argmin)
                 │
                 ▼
   [3] Burbuja de seguridad (anular vecinos)
                 │
                 ▼
   [4] Gap más grande (secuencia libre más larga)
                 │
                 ▼
   [5] Punto objetivo (centro del gap)
                 │
                 ▼
   [6] steering + velocidad proporcional
                 │
                 ▼
       /drive (AckermannDriveStamped)
```

---

## 3. Estructura del código

Todo el controlador vive en una sola clase, `ReactiveFollowGap(Node)`, dentro de [`src/gap_node.py`](src/gap_node.py).

```
gap_node.py
└── class ReactiveFollowGap(Node)
    ├── __init__()             Suscripciones, publicador y parámetros
    ├── preprocess_lidar()     Paso 1: limpia y suaviza el scan
    ├── find_max_gap()         Paso 4: secuencia libre continua más larga
    ├── find_best_point()      Paso 5: centro del gap
    ├── odom_callback()        Contador y cronómetro de vueltas (no conduce)
    └── lidar_callback()       Orquesta los 6 pasos y publica el comando
```

### Comunicación ROS 2

| Dirección | Tópico | Tipo | Uso |
|-----------|--------|------|-----|
| Suscribe | `/scan` | `sensor_msgs/LaserScan` | Datos del LIDAR (motor del algoritmo) |
| Suscribe | `/ego_racecar/odom` | `nav_msgs/Odometry` | Solo para contar vueltas |
| Publica | `/drive` | `ackermann_msgs/AckermannDriveStamped` | Dirección + velocidad |

### Detalle de cada método

- **`__init__`** — Crea la suscripción a `/scan` (el algoritmo corre cada vez que llega un scan, ~50 Hz), el publicador a `/drive`, y define todos los parámetros ajustables.
- **`preprocess_lidar(ranges)`** — Devuelve el scan limpio: sin `inf`/`NaN`, recortado a `rango_max` y suavizado con media móvil (`np.convolve`).
- **`find_max_gap(free_space_ranges)`** — Recorre la máscara booleana `distancia > umbral_gap` y devuelve `(inicio, fin)` del tramo libre más largo, contemplando el caso borde de un gap que llega al final del array.
- **`find_best_point(start_i, end_i)`** — Devuelve el índice central `(start_i + end_i) // 2`.
- **`lidar_callback(data)`** — El corazón: ejecuta los 6 pasos, aplica anti-oscilación (zona muerta + filtro pasa-bajos), calcula la velocidad proporcional y publica en `/drive`.
- **`odom_callback(msg)`** — Máquina de estados de 2 fases que cuenta vueltas y mide su tiempo. **No interviene en la conducción.**

---

## 4. Parámetros ajustables

Todos se definen en `__init__`. Los valores mostrados son los usados en esta implementación.

### Navegación

| Parámetro | Valor | Si lo SUBES | Si lo BAJAS |
|-----------|-------|-------------|-------------|
| `rango_max` | 3.0 m | Ve huecos más lejanos, anticipa más | Más "miope", solo reacciona a lo cercano |
| `radio_burbuja` | 80 | Se aleja de paredes; puede tapar gaps válidos | Pasa más cerca; riesgo de raspar |
| `ventana_suavizado` | 5 | Scan más limpio, borra detalles finos | Más sensible al ruido |
| `umbral_gap` | 1.0 m | Más conservador, gaps cortos | Más permisivo, entra a espacios estrechos |
| `fov_recorte` | 100° | Ve más a los lados (curvas cerradas) | Más enfocado al frente, estable |

### Velocidad y suavizado

| Parámetro | Valor | Si lo SUBES | Si lo BAJAS |
|-----------|-------|-------------|-------------|
| `vel_recta` | 8.0 m/s | Más rápido en recta | Más seguro, vueltas lentas |
| `vel_curva` | 2.5 m/s | Curvas más rápidas; riesgo de salirse | Curvas seguras pero lentas |
| `zona_muerta` | 20° | Rectas firmes; ignora giros pequeños | Más serpenteo |
| `alpha_suavizado` | 0.4 | Reacciona rápido pero brusco | Más suave y estable; reacciona tarde |

> **La velocidad proporcional no se ajusta directamente.** Se recalcula sola entre `vel_recta` y `vel_curva` según el giro:
> `speed = vel_recta − (vel_recta − vel_curva) · (|steering| / 0.4)`

Para la guía completa de tuning, ver [`guia_parametros.md`](guia_parametros.md).

---

## 5. Requisitos

- **Ubuntu 22.04**
- **ROS 2 Humble**
- **Simulador F1TENTH Gym + ROS bridge:** [`f1tenth_gym_ros`](https://github.com/f1tenth/f1tenth_gym_ros)
- Dependencias Python: `numpy`, `rclpy`, `ackermann_msgs`

---

## 6. Instrucciones de ejecución

Este nodo es un **controlador**; necesita el simulador F1TENTH corriendo para tener algo que conducir.

### Paso 1 — Tener el workspace del simulador

Sigue la instalación oficial de [`f1tenth_gym_ros`](https://github.com/f1tenth/f1tenth_gym_ros). Deberías terminar con un workspace de ROS 2 que contiene el paquete `f1tenth_gym_ros` y un paquete de `controllers`.

### Paso 2 — Colocar el nodo en tu paquete de controladores

Copia `src/gap_node.py` a la carpeta de tu paquete de controladores, por ejemplo:

```bash
cp src/gap_node.py  ~/tu_ws/src/controllers/controllers/gap_node.py
```

### Paso 3 — Registrar el ejecutable en `setup.py`

Dentro de `entry_points → console_scripts` de tu paquete `controllers`, agrega:

```python
'gap_node = controllers.gap_node:main',
```

### Paso 4 — Compilar y sourcear

```bash
cd ~/tu_ws
colcon build
source install/setup.bash
```

### Paso 5 — Lanzar el simulador (terminal 1)

```bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Se abrirá **RViz** con el carro en la pista.

### Paso 6 — Lanzar el controlador (terminal 2)

```bash
source install/setup.bash
ros2 run controllers gap_node
```

El carro empezará a dar vueltas solo. En la terminal del controlador verás el log de cada vuelta:

```
[INFO] Cronómetro iniciado — esperando primera vuelta...
[INFO] VUELTA 1 completada — tiempo: 24.83 s
[INFO] VUELTA 2 completada — tiempo: 23.91 s
```

> **Recuerda:** cada vez que edites `gap_node.py`, vuelve a ejecutar `colcon build && source install/setup.bash`.

### Solución de problemas

| Síntoma | Causa probable | Solución |
|---------|----------------|----------|
| El carro no se mueve | No sourceaste la terminal, o el nodo no publica en `/drive` | `source install/setup.bash` en **cada** terminal; verifica con `ros2 topic echo /drive` |
| `Package 'controllers' not found` | Olvidaste compilar o registrar el nodo | `colcon build` y revisa el `entry_point` en `setup.py` |
| El carro choca en las curvas | Va muy rápido para lo que reacciona | Baja `vel_curva`, o sube `radio_burbuja` y `alpha_suavizado` |
| El carro serpentea en las rectas | Reacciona a giros minúsculos por ruido | Sube `zona_muerta` y/o baja `alpha_suavizado` |
| Los cambios no tienen efecto | Corriste el nodo sin recompilar | `colcon build && source install/setup.bash` antes de `ros2 run` |

> Los detalles de cada parámetro están en la [guía de tuning](guia_parametros.md).

---

## 7. Mejoras adicionales implementadas

Más allá del FTG básico, este nodo incluye:

- **Recorte de FOV (`fov_recorte`)** — solo se procesa el sector frontal (±100°); los rayos hacia atrás no sirven para conducir hacia adelante.
- **Anti-oscilación** — combinación de:
  - **Zona muerta:** ángulos menores a `zona_muerta` se fuerzan a 0 → rectas firmes sin serpenteo.
  - **Filtro pasa-bajos:** `steering = α·nuevo + (1−α)·anterior` → la dirección cambia suave, no a saltos.
- **Velocidad proporcional** — interpola entre `vel_recta` y `vel_curva` según el giro: acelera en recta, frena al entrar a la curva.
- **Contador y cronómetro de vueltas** — `odom_callback` detecta cada vuelta (el carro debe alejarse de la salida y luego volver) y reporta su tiempo, dando una **métrica objetiva** para comparar configuraciones.

---

## 8. Mejora propuesta: Disparity Extender

Una mejora natural sobre este FTG básico es el **Disparity Extender**.

En un salto brusco de distancia entre dos rayos vecinos (una **disparidad**, típica de esquinas), el LIDAR mide hasta la pared del fondo, pero el borde físico del obstáculo está mucho más cerca. Sin corregir esto, el carro apunta a "huecos fantasma" detrás de las esquinas y puede rozar el borde.

La mejora toma el valor **más cercano** de la disparidad y lo **extiende** sobre los rayos vecinos:

```
ANTES:    [... 2.0  2.1  2.0  8.5  9.0  8.8 ...]   ← disparidad (salto brusco)
DESPUÉS:  [... 2.0  2.1  2.0  2.0  2.0  8.8 ...]   ← borde cercano extendido
```

El número de rayos a extender depende del ancho del carro y la distancia:

```
angulo = atan(ancho_carro / distancia)   →   más cerca = más rayos
```

**Beneficios:**
- Evita apuntar a gaps fantasma detrás de las esquinas.
- Toma las curvas más limpio → permite subir `vel_curva` sin salirse.
- Es un paso **adicional** al FTG básico: conserva burbuja, gap, anti-oscilación, etc.

---


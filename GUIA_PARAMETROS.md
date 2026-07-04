# Guía de parámetros — Follow the Gap (`gap_node.py`)

Apuntes de tuning del controlador reactivo **Follow the Gap** implementado en `gap_node.py`.

| Nodo | Archivo | Comando |
|------|---------|---------|
| Follow the Gap | `gap_node.py` | `ros2 run controllers gap_node` |

> Recuerda: tras editar el archivo → `colcon build && source install/setup.bash`

---

## 1. Geometría del LIDAR (base de todo)

- **FOV total:** 4.70 rad ≈ 270° · **1080 rayos** · `angle_increment ≈ 0.00435 rad`
- **Índice → ángulo:** `angulo = angle_min + indice * angle_increment`

| Índice | Ángulo | Dirección |
|--------|--------|-----------|
| 0 | -2.35 rad (-135°) | derecha |
| 540 | 0 rad (0°) | **frente** |
| 1079 | +2.35 rad (+135°) | izquierda |

Convención de manejo: `steering_angle` positivo = izquierda, negativo = derecha. Máximo físico ≈ **±0.4 rad**.

---

## 2. Cómo funciona Follow the Gap

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

---

## 3. Tabla de parámetros — Navegación

| Parámetro | Valor | Si lo SUBES | Si lo BAJAS |
|-----------|-------|-------------|-------------|
| `rango_max` | 3.0 m | Ve gaps más lejanos, anticipa más | Más "miope", solo reacciona a lo cercano |
| `radio_burbuja` | 80 | Se aleja de paredes, curvas abiertas; puede tapar gaps válidos | Pasa más cerca, aprovecha huecos chicos; riesgo de raspar |
| `ventana_suavizado` | 5 | Scan más limpio, borra detalles finos | Más sensible al ruido |
| `umbral_gap` | 1.0 m | Más exigente, gaps cortos/conservadores | Más permisivo, entra a espacios estrechos |
| `fov_recorte` | 100° | Ve más a los lados (curvas cerradas); puede desviarse | Más enfocado al frente, estable; tarda en ver salida de curva |

## 4. Tabla de parámetros — Velocidad y suavizado

| Parámetro | Valor | Si lo SUBES | Si lo BAJAS |
|-----------|-------|-------------|-------------|
| `vel_recta` | 8.0 m/s | Más rápido en recta; menos reacción | Más seguro, vueltas lentas |
| `vel_curva` | 2.5 m/s | Curvas más rápidas; riesgo de salirse | Curvas seguras pero lentas |
| `zona_muerta` | 20° | Rectas firmes; ignora giros pequeños (peligroso si muy alto) | Reacciona a giros mínimos; más serpenteo |
| `alpha_suavizado` | 0.4 | Reacciona rápido pero brusco/oscilante | Más suave y estable; reacciona tarde en curvas |

> **La velocidad proporcional NO se ajusta directo.** Se recalcula sola entre `vel_recta` y `vel_curva` según el ángulo de giro:
> `speed = vel_recta − (vel_recta − vel_curva) · (|steering| / 0.4)`

## 5. Tabla de parámetros — Contador de vueltas

| Parámetro | Valor | Efecto |
|-----------|-------|--------|
| `umbral_lejos` | 5.0 m | Distancia mínima a alejarse antes de validar vuelta. Súbelo si cuenta vueltas falsas |
| `umbral_cerca` | 2.0 m | Qué tan cerca debe volver para contar. Muy bajo = puede no detectarla |

---

## 6. Relaciones entre parámetros (se compensan entre sí)

| Relación | Cómo se compensan |
|----------|-------------------|
| `vel_recta` ↑ → `alpha_suavizado` ↓ | A más velocidad, más suavizado para no serpentear |
| `vel_curva` ↑ → `radio_burbuja` ↑ | Curvas rápidas → aléjate más de las paredes |
| `fov_recorte` ↑ → `zona_muerta` ↑ | Más visión lateral mete ruido; compénsalo con zona muerta |
| `radio_burbuja` ↑ → `umbral_gap` ↓ | Si la burbuja tapa mucho, baja el umbral para hallar gaps |
| `vel_recta` ↔ `vel_curva` | Definen el rango de la velocidad proporcional |

---

## 7. Método de tuning recomendado

1. Cambia **un parámetro a la vez.**
2. Mide el **tiempo por vuelta** (lo imprime el nodo en la terminal) — métrica objetiva, no "se siente mejor".
3. Los 3 de mayor impacto en el tiempo: **`vel_curva`**, **`alpha_suavizado`**, **`radio_burbuja`**.
4. Para `vel_curva`: sube de a poco (2.5 → 3.0 → 3.5) hasta que se salga, luego retrocede un paso.

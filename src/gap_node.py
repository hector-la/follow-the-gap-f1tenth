#!/usr/bin/env python3

# ============================================================
# gap_node.py — Follow the Gap (versión básica)
#
# Navegación reactiva: en cada scan del LIDAR el robot busca
# el espacio libre más grande frente a él y se dirige a su centro.
# Pasos: preprocesar → punto más cercano → burbuja → gap más
# grande → punto objetivo → publicar dirección y velocidad.
# ============================================================

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_follow_gap_node')
        self.get_logger().info("ReactiveFollowGap node has been started!")

        # --- SUSCRIPCIÓN Y PUBLICADOR ---
        # El algoritmo corre directamente cada vez que llega un scan (~50 Hz).
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Odometría: la usamos solo para contar vueltas y medir su tiempo.
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        # --- PARÁMETROS AJUSTABLES ---
        self.rango_max = 3.0          # m — distancias mayores se recortan a este valor
        self.radio_burbuja = 80      # nº de rayos a poner en cero alrededor del más cercano
        self.ventana_suavizado = 5    # nº de rayos a promediar para reducir ruido
        self.umbral_gap = 1.0         # m — un rayo cuenta como "libre" si supera esta distancia

        # Recorte del campo de visión: solo usamos el sector frontal (~±70°).
        # Los rayos que apuntan hacia atrás no sirven para conducir hacia adelante.
        self.fov_recorte = math.radians(100)   # medio-ángulo del sector frontal que conservamos

        # Velocidades
        self.vel_recta = 7.5         # m/s cuando el camino está recto
        self.vel_curva = 2.15        # m/s cuando el robot está girando fuerte

        # Anti-oscilación
        self.zona_muerta = math.radians(20)   # ángulos menores a esto se fuerzan a 0 (recta)
        self.alpha_suavizado = 0.4             # peso del ángulo nuevo (0=todo viejo, 1=todo nuevo)
        self.steering_previo = 0.0           # último ángulo publicado (para el filtro)

        # Índices del sector frontal — se calculan en el primer scan
        self.idx_inicio = None
        self.idx_fin = None

        # --- CONTADOR Y CRONÓMETRO DE VUELTAS ---
        # Detectamos una vuelta así: el carro debe ALEJARSE de la posición de
        # salida (más de umbral_lejos) y luego VOLVER cerca de ella (menos de
        # umbral_cerca). Ese ciclo de ida y vuelta cuenta como 1 vuelta.
        self.pos_salida = None        # (x, y) donde arrancó el carro
        self.num_vuelta = 0           # vueltas completadas
        self.tiempo_vuelta_ini = None # marca de tiempo del inicio de la vuelta actual
        self.salio_de_salida = False  # True una vez que se alejó del punto de salida
        self.umbral_lejos = 5.0       # m — debe alejarse al menos esto
        self.umbral_cerca = 2.0       # m — y volver a menos de esto para contar la vuelta

    # ----------------------------------------------------------
    def preprocess_lidar(self, ranges):
        """
        Limpia el scan crudo:
        - Recorta valores grandes a self.rango_max.
        - Suaviza con una media móvil para eliminar ruido.
        - Reemplaza inf/NaN por 0 (se tratan como obstáculo).
        """
        proc = np.array(ranges, dtype=np.float64)

        # inf y NaN → 0 (los tratamos como obstáculo, no como espacio libre)
        proc[np.isinf(proc)] = 0.0
        proc[np.isnan(proc)] = 0.0

        # Recortar distancias grandes: un gap a 25 m no es más útil que uno a 3 m
        proc[proc > self.rango_max] = self.rango_max

        # Suavizado: promedio en ventana para eliminar lecturas erráticas
        if self.ventana_suavizado > 1:
            kernel = np.ones(self.ventana_suavizado) / self.ventana_suavizado
            proc = np.convolve(proc, kernel, mode='same')

        return proc

    # ----------------------------------------------------------
    def find_max_gap(self, free_space_ranges):
        """
        Encuentra la secuencia continua más larga de valores 'libres'
        (mayores que self.umbral_gap). Devuelve (inicio, fin) índices.
        """
        # Máscara booleana: True donde hay espacio libre
        libre = free_space_ranges > self.umbral_gap

        mejor_inicio, mejor_fin = 0, 0   # mejor gap encontrado hasta ahora
        mejor_largo = 0
        inicio_actual = None             # inicio del gap que estamos recorriendo

        for i, es_libre in enumerate(libre):
            if es_libre:
                # Si estamos empezando un gap nuevo, marcamos su inicio
                if inicio_actual is None:
                    inicio_actual = i
            else:
                # Se cortó el gap: evaluamos si fue el más largo
                if inicio_actual is not None:
                    largo = i - inicio_actual
                    if largo > mejor_largo:
                        mejor_largo = largo
                        mejor_inicio, mejor_fin = inicio_actual, i - 1
                    inicio_actual = None

        # Caso borde: el gap llega hasta el final del array
        if inicio_actual is not None:
            largo = len(libre) - inicio_actual
            if largo > mejor_largo:
                mejor_inicio, mejor_fin = inicio_actual, len(libre) - 1

        return mejor_inicio, mejor_fin

    # ----------------------------------------------------------
    def find_best_point(self, start_i, end_i):
        """
        Dentro del gap [start_i, end_i] elige el punto objetivo.
        Estrategia: el CENTRO del gap (más estable que el punto más lejano).
        """
        return (start_i + end_i) // 2

    # ----------------------------------------------------------
    def odom_callback(self, msg):
        """
        Cuenta vueltas y mide su duración usando la posición del carro.
        No interviene en la conducción — solo observa.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # En el primer mensaje guardamos el punto de salida y arrancamos el reloj
        if self.pos_salida is None:
            self.pos_salida = (x, y)
            self.tiempo_vuelta_ini = time.time()
            self.get_logger().info("Cronómetro iniciado — esperando primera vuelta...")
            return

        # Distancia actual al punto de salida
        dx = x - self.pos_salida[0]
        dy = y - self.pos_salida[1]
        dist_salida = math.sqrt(dx * dx + dy * dy)

        # Máquina de estados de la vuelta:
        if not self.salio_de_salida:
            # Fase 1: esperamos a que el carro se aleje de la salida.
            # (Si no, contaríamos una vuelta apenas arranca.)
            if dist_salida > self.umbral_lejos:
                self.salio_de_salida = True
        else:
            # Fase 2: ya se alejó; si vuelve cerca de la salida → vuelta completa.
            if dist_salida < self.umbral_cerca:
                ahora = time.time()
                tiempo_vuelta = ahora - self.tiempo_vuelta_ini
                self.num_vuelta += 1
                self.get_logger().info(
                    f"VUELTA {self.num_vuelta} completada — tiempo: {tiempo_vuelta:.2f} s"
                )
                # Reiniciamos para la siguiente vuelta
                self.tiempo_vuelta_ini = ahora
                self.salio_de_salida = False

    # ----------------------------------------------------------
    def lidar_callback(self, data):
        """ Ejecuta el algoritmo Follow the Gap y publica el comando de manejo. """

        angle_min = data.angle_min
        angle_increment = data.angle_increment

        # En el primer scan calculamos qué índices forman el sector frontal
        if self.idx_inicio is None:
            centro = len(data.ranges) // 2
            n_rayos = int(self.fov_recorte / angle_increment)
            self.idx_inicio = centro - n_rayos
            self.idx_fin = centro + n_rayos

        # 1) PREPROCESAR el scan completo
        proc = self.preprocess_lidar(data.ranges)

        # Nos quedamos solo con el sector frontal
        frente = proc[self.idx_inicio:self.idx_fin]

        # 2) PUNTO MÁS CERCANO: el obstáculo más peligroso dentro del sector frontal
        idx_cercano = np.argmin(frente)

        # 3) BURBUJA DE SEGURIDAD: poner a cero los rayos vecinos al más cercano.
        #    El robot tiene ancho, así que "engordamos" el obstáculo para no
        #    intentar pasar por huecos donde no cabe.
        ini_burbuja = max(0, idx_cercano - self.radio_burbuja)
        fin_burbuja = min(len(frente), idx_cercano + self.radio_burbuja)
        frente[ini_burbuja:fin_burbuja] = 0.0

        # 4) GAP MÁS GRANDE: la secuencia continua de espacio libre más larga
        gap_inicio, gap_fin = self.find_max_gap(frente)

        # 5) PUNTO OBJETIVO dentro del gap
        idx_objetivo = self.find_best_point(gap_inicio, gap_fin)

        # Convertir índice (relativo al sector frontal) a ángulo de dirección.
        # Sumamos idx_inicio para volver al índice global del scan completo.
        idx_global = idx_objetivo + self.idx_inicio
        steering_angle = angle_min + idx_global * angle_increment

        # --- ANTI-OSCILACIÓN ---
        # (a) Zona muerta: en rectas, ángulos minúsculos saltan de lado a lado
        #     por ruido. Si el ángulo es muy pequeño, lo forzamos a 0 → recta firme.
        if abs(steering_angle) < self.zona_muerta:
            steering_angle = 0.0

        # (b) Filtro pasa-bajos: mezclamos el ángulo nuevo con el anterior para
        #     que la dirección cambie de forma suave en vez de a saltos bruscos.
        #     alpha bajo = más suave (más memoria del valor previo).
        steering_angle = (self.alpha_suavizado * steering_angle +
                          (1 - self.alpha_suavizado) * self.steering_previo)
        self.steering_previo = steering_angle

        # Limitar el ángulo al máximo físico del carro (±0.4 rad aprox.)
        steering_angle = max(-0.4, min(0.4, steering_angle))

        # 6) VELOCIDAD PROPORCIONAL: interpolación lineal entre vel_recta y vel_curva
        #    según cuánto gira el carro. Sin escalón brusco:
        #      steering = 0     → vel_recta
        #      steering = ±0.4  → vel_curva
        #    Esto suaviza la entrada/salida de curvas en vez de frenar de golpe.
        factor_giro = abs(steering_angle) / 0.4          # 0 en recta, 1 en giro máximo
        speed = self.vel_recta - (self.vel_recta - self.vel_curva) * factor_giro

        # Publicar comando
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = speed
        self.drive_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    reactive_follow_gap_node = ReactiveFollowGap()
    rclpy.spin(reactive_follow_gap_node)

    reactive_follow_gap_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
Actividad A - Control de drones desde ESP32
Compatible con gym-pybullet-drones 2.2.x

Comandos que recibe por USB/Serial:
    A  -> mover la formación a la zona A
    B  -> mover la formación a la zona B
    C  -> mover la formación a la zona C
    STOP -> detener la simulación

La ESP32 manda únicamente la orden; PyBullet y el controlador
DSLPIDControl ejecutan la simulación.
"""

import time
import numpy as np
import pybullet as p
import serial

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync


# ============================================================
# CONFIGURACIÓN
# ============================================================

NUM_DRONES = 6

SIMULATION_FREQ_HZ = 240
CONTROL_FREQ_HZ = 48

DRONE_MODEL = DroneModel.CF2X
PHYSICS = Physics.PYB

# CAMBIA ESTE PUERTO por el de tu ESP32
SERIAL_PORT = "COM3"
BAUD_RATE = 115200

# Tiempo para desplazar toda la formación entre zonas
TRANSITION_TIME = 4.0

# Altura de vuelo
FLIGHT_Z = 1.0

# Posiciones centrales de las tres zonas.
# Puedes ajustarlas después para que coincidan con tu escenario.
ZONES = {
    "A": np.array([0.0, 0.0, FLIGHT_Z]),
    "B": np.array([2.0, 0.0, FLIGHT_Z]),
    "C": np.array([2.0, 2.0, FLIGHT_Z]),
}

# Formación de 6 drones alrededor del centro.
FORMATION_OFFSETS = np.array([
    [-0.45, -0.30, 0.0],
    [ 0.00, -0.30, 0.0],
    [ 0.45, -0.30, 0.0],
    [-0.45,  0.30, 0.0],
    [ 0.00,  0.30, 0.0],
    [ 0.45,  0.30, 0.0],
])

INIT_XYZS = ZONES["A"] + FORMATION_OFFSETS
INIT_RPYS = np.zeros((NUM_DRONES, 3))


def draw_zone_markers(client_id):
    """Dibuja A, B y C en el suelo para visualizar la misión."""
    for name, pos in ZONES.items():
        x, y, _ = pos

        # Cruz alrededor del centro de la zona
        p.addUserDebugLine(
            [x - 0.35, y, 0.02],
            [x + 0.35, y, 0.02],
            lineWidth=3,
            lifeTime=0,
            physicsClientId=client_id
        )
        p.addUserDebugLine(
            [x, y - 0.35, 0.02],
            [x, y + 0.35, 0.02],
            lineWidth=3,
            lifeTime=0,
            physicsClientId=client_id
        )

        p.addUserDebugText(
            f"ZONA {name}",
            [x, y, 0.15],
            textSize=1.5,
            lifeTime=0,
            physicsClientId=client_id
        )


def read_serial_command(ser):
    """Lee, sin bloquear, un comando enviado por la ESP32."""
    command = None

    while ser.in_waiting:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()

        if raw:
            raw = raw.upper()

            # La ESP32 manda exactamente A, B, C o STOP.
            if raw in ("A", "B", "C", "STOP"):
                command = raw

    return command


def main():
    print("=" * 60)
    print("ACTIVIDAD A - DRONES CONTROLADOS DESDE ESP32")
    print("=" * 60)
    print(f"Puerto ESP32: {SERIAL_PORT}")
    print("Comandos: A / B / C / STOP")
    print("=" * 60)

    # --------------------------------------------------------
    # Conexión con la ESP32
    # --------------------------------------------------------
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=0
        )
        # La apertura del puerto puede reiniciar algunas ESP32.
        time.sleep(2.0)
        print("ESP32 conectada correctamente.")
    except serial.SerialException as exc:
        print("\nERROR: no se pudo abrir el puerto serie.")
        print(f"Revisa SERIAL_PORT = '{SERIAL_PORT}'.")
        print("También cierra el Monitor Serial del Arduino IDE.")
        raise SystemExit(exc)

    # --------------------------------------------------------
    # Crear simulación
    # --------------------------------------------------------
    env = CtrlAviary(
        drone_model=DRONE_MODEL,
        num_drones=NUM_DRONES,
        initial_xyzs=INIT_XYZS,
        initial_rpys=INIT_RPYS,
        physics=PHYSICS,
        neighbourhood_radius=10,
        pyb_freq=SIMULATION_FREQ_HZ,
        ctrl_freq=CONTROL_FREQ_HZ,
        gui=True,
        record=False,
        obstacles=True,
        user_debug_gui=False
    )

    client_id = env.getPyBulletClient()
    draw_zone_markers(client_id)

    # --------------------------------------------------------
    # Controladores PID: uno por dron
    # --------------------------------------------------------
    controllers = [
        DSLPIDControl(drone_model=DRONE_MODEL)
        for _ in range(NUM_DRONES)
    ]

    # --------------------------------------------------------
    # Estado de la misión
    # --------------------------------------------------------
    current_zone = "A"
    commanded_center = ZONES["A"].copy()

    # Variables para interpolar suavemente A -> B -> C.
    transition_start = commanded_center.copy()
    transition_goal = commanded_center.copy()
    transition_elapsed = TRANSITION_TIME

    # Acción inicial
    action = np.zeros((NUM_DRONES, 4))

    obs, _, terminated, truncated, _ = env.step(action)

    start_time = time.time()
    step = 0

    print("\nMisión iniciada en ZONA A.")
    print("Presiona A, B o C en la ESP32 para mover la formación.")
    print("Presiona STOP para detener el vuelo.\n")

    try:
        while not terminated and not truncated:

            # ------------------------------------------------
            # Leer ESP32
            # ------------------------------------------------
            command = read_serial_command(ser)

            if command is not None:
                if command == "STOP":
                    print("STOP recibido desde ESP32. Se detiene la simulación.")
                    break

                if command in ZONES:
                    if command != current_zone:
                        current_zone = command

                        # El movimiento empieza desde el centro actual.
                        transition_start = commanded_center.copy()
                        transition_goal = ZONES[current_zone].copy()
                        transition_elapsed = 0.0

                        print(f"ESP32 -> orden recibida: ir a ZONA {current_zone}")

            # ------------------------------------------------
            # Interpolación suave del centro de la formación
            # ------------------------------------------------
            if transition_elapsed < TRANSITION_TIME:
                transition_elapsed += env.CTRL_TIMESTEP

                alpha = min(
                    transition_elapsed / TRANSITION_TIME,
                    1.0
                )

                commanded_center = (
                    (1.0 - alpha) * transition_start
                    + alpha * transition_goal
                )
            else:
                commanded_center = transition_goal.copy()

            # ------------------------------------------------
            # Control PID de cada dron
            # ------------------------------------------------
            for drone_id in range(NUM_DRONES):

                target_position = (
                    commanded_center
                    + FORMATION_OFFSETS[drone_id]
                )

                action[drone_id, :], _, _ = controllers[
                    drone_id
                ].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs[drone_id],
                    target_pos=target_position,
                    target_rpy=INIT_RPYS[drone_id]
                )

            # ------------------------------------------------
            # Avanzar PyBullet
            # ------------------------------------------------
            obs, _, terminated, truncated, _ = env.step(action)

            env.render()

            sync(
                step,
                start_time,
                env.CTRL_TIMESTEP
            )

            step += 1

    finally:
        ser.close()
        env.close()

    print("\nSimulación finalizada.")
    print(f"Última zona seleccionada: {current_zone}")


if __name__ == "__main__":
    main()

import time
import numpy as np
import pybullet as p
import serial


SERIAL_PORT = "COM3"   # CAMBIA AL COM DE TU ESP32
BAUD_RATE = 115200

DT = 1.0 / 120.0

# Velocidades máximas
MAX_SPEED = 120          # m/s
MAX_TURN = 120           # rad/s

# Aceleraciones máximas
ACCEL = 250
TURN_ACCEL = 300

# Atlas del ejemplo original
ATLAS_START = [-2.0, 3.0, -0.5]


def ramp(current, target, max_change):
    error = target - current

    if error > max_change:
        return current + max_change

    if error < -max_change:
        return current - max_change

    return target


def parse_message(line):

    if not line.startswith("JOY,"):
        return None

    parts = line.split(",")

    if len(parts) != 4:
        return None

    try:
        x = float(parts[1])
        y = float(parts[2])
        button = int(parts[3])
    except ValueError:
        return None

    return (
        np.clip(x, -1.0, 1.0),
        np.clip(y, -1.0, 1.0),
        1 if button else 0
    )


def main():

    print("=" * 60)
    print("ACTIVIDAD C - ATLAS + ESP32")
    print("=" * 60)

    # --------------------------------------------------------
    # ESP32
    # --------------------------------------------------------

    try:
        ser = serial.Serial(
            SERIAL_PORT,
            BAUD_RATE,
            timeout=0
        )

        time.sleep(2)

        print(
            f"ESP32 conectada en {SERIAL_PORT}"
        )

    except serial.SerialException as error:

        print("ERROR SERIAL")
        print(error)

        return

    # --------------------------------------------------------
    # PYBULLET
    # --------------------------------------------------------

    p.connect(p.GUI)

    p.resetSimulation()

    p.setGravity(
        0,
        0,
        -10
    )

    p.setTimeStep(
        DT
    )

    # --------------------------------------------------------
    # ATLAS
    # --------------------------------------------------------

    atlas = p.loadURDF(
        "atlas/atlas_v4_with_multisense.urdf",
        ATLAS_START
    )

    # Postura inicial de articulaciones
    for joint in range(
        p.getNumJoints(atlas)
    ):

        p.setJointMotorControl2(
            atlas,
            joint,
            p.POSITION_CONTROL,
            targetPosition=0,
            force=120
        )

    # --------------------------------------------------------
    # BOTLAB
    # --------------------------------------------------------

    objects = p.loadSDF(
        "botlab/botlab.sdf",
        globalScaling=2.0
    )

    zero = [0, 0, 0]

    y2x = p.getQuaternionFromEuler(
        [
            np.pi / 2,
            0,
            np.pi / 2
        ]
    )

    for obj in objects:

        pos, orn = (
            p.getBasePositionAndOrientation(
                obj
            )
        )

        new_pos, new_orn = (
            p.multiplyTransforms(
                zero,
                y2x,
                pos,
                orn
            )
        )

        p.resetBasePositionAndOrientation(
            obj,
            new_pos,
            new_orn
        )

    # --------------------------------------------------------
    # CAJAS
    # --------------------------------------------------------

    p.loadURDF(
        "boston_box.urdf",
        [-2, 3, -2],
        useFixedBase=True
    )

    p.loadURDF(
        "boston_box.urdf",
        [0, 3, -2],
        useFixedBase=True
    )

    # --------------------------------------------------------
    # CÁMARA
    # --------------------------------------------------------

    p.resetDebugVisualizerCamera(
        cameraDistance=2.8,
        cameraYaw=145,
        cameraPitch=-12,
        cameraTargetPosition=[
            -2,
            3,
            -0.4
        ]
    )

    # --------------------------------------------------------
    # ESTADO DEL ATLAS
    # --------------------------------------------------------

    atlas_position = np.array(
        ATLAS_START,
        dtype=float
    )

    atlas_yaw = 0.0

    current_speed = 0.0
    current_turn = 0.0

    target_speed = 0.0
    target_turn = 0.0

    stop = False

    previous_button = 0

    last_serial = time.time()

    # --------------------------------------------------------
    # LOOP
    # --------------------------------------------------------

    while True:

        joystick_x = 0.0
        joystick_y = 0.0
        button = 0

        # ----------------------------------------------------
        # LEER ESP32
        # ----------------------------------------------------

        while ser.in_waiting:

            line = (
                ser.readline()
                .decode(
                    "utf-8",
                    errors="ignore"
                )
                .strip()
                .upper()
            )

            data = parse_message(
                line
            )

            if data is None:
                continue

            (
                joystick_x,
                joystick_y,
                button
            ) = data

            last_serial = time.time()

        # ----------------------------------------------------
        # WATCHDOG
        # ----------------------------------------------------

        if (
            time.time()
            - last_serial
            > 0.5
        ):

            joystick_x = 0
            joystick_y = 0

        # ----------------------------------------------------
        # BOTÓN STOP
        # ----------------------------------------------------

        if (
            previous_button == 0
            and button == 1
        ):

            stop = not stop

            if stop:
                print(
                    "[ESP32] STOP"
                )
            else:
                print(
                    "[ESP32] REANUDAR"
                )

        previous_button = button

        # ----------------------------------------------------
        # OBJETIVOS
        # ----------------------------------------------------

        if stop:

            target_speed = 0
            target_turn = 0

        else:

            target_speed = (
                joystick_y
                * MAX_SPEED
            )

            target_turn = (
                joystick_x
                * MAX_TURN
            )

        # ----------------------------------------------------
        # ACELERACIÓN SUAVE
        # ----------------------------------------------------

        current_speed = ramp(
            current_speed,
            target_speed,
            ACCEL * DT
        )

        current_turn = ramp(
            current_turn,
            target_turn,
            TURN_ACCEL * DT
        )

        # ----------------------------------------------------
        # MOVIMIENTO
        # ----------------------------------------------------

        atlas_yaw += (
            current_turn
            * DT
        )

        dx = (
            np.cos(atlas_yaw)
            * current_speed
            * DT
        )

        dy = (
            np.sin(atlas_yaw)
            * current_speed
            * DT
        )

        atlas_position[0] += dx
        atlas_position[1] += dy

        # ----------------------------------------------------
        # APLICAR POSICIÓN DIRECTAMENTE
        # ----------------------------------------------------

        orientation = p.getQuaternionFromEuler(
            [
                0,
                0,
                atlas_yaw
            ]
        )

        p.resetBasePositionAndOrientation(
            atlas,
            atlas_position.tolist(),
            orientation
        )

        # ----------------------------------------------------
        # VELOCIDAD CERO FÍSICA
        # ----------------------------------------------------

        p.resetBaseVelocity(
            atlas,
            linearVelocity=[0, 0, 0],
            angularVelocity=[0, 0, 0]
        )

        p.stepSimulation()

        # ----------------------------------------------------
        # IMPRIMIR JOYSTICK
        # ----------------------------------------------------

        if (
            abs(joystick_x) > 0.05
            or abs(joystick_y) > 0.05
        ):

            print(
                f"Joystick X={joystick_x:+.2f} "
                f"Y={joystick_y:+.2f} | "
                f"Vel={current_speed:+.2f} | "
                f"Giro={current_turn:+.2f}"
            )

        time.sleep(DT)


if __name__ == "__main__":
    try:
        main()

    except KeyboardInterrupt:

        print(
            "\nPrograma detenido."
        )
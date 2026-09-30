"""
ACTIVIDAD B - BAXTER + ESP32 + JOYSTICK
VERSIÓN CORREGIDA DE MESA Y OBJETO

Correcciones:
- La mesa queda centrada directamente bajo el brazo/objeto,
  no en una coordenada fija del escenario.
- La mesa es suficientemente grande para que el cubo no caiga.
- El cubo se coloca exactamente sobre la superficie de la mesa.
- El primer SW ejecuta PICK automático.
- El movimiento manual conserva mayor velocidad.
"""

import time
import numpy as np
import pybullet as p
import pybullet_data
import serial


# ============================================================
# CONFIGURACIÓN
# ============================================================

SERIAL_PORT = "COM5"       # <-- CAMBIAR AL COM DE LA ESP32
BAUD_RATE = 115200

DT = 1.0 / 120.0

# Movimiento más rápido
MOVE_SPEED_X = 0.75
MOVE_SPEED_Z = 0.65

JOINT_FORCE = 250.0
FINGER_FORCE = 35.0

# Tamaño del objeto
OBJECT_HALF = 0.035

# Mesa
TABLE_THICKNESS = 0.05
TABLE_HALF_X = 0.48
TABLE_HALF_Y = 0.38

# El cubo queda esta distancia por debajo de la mano
HAND_TO_OBJECT = 0.10

# Distancia máxima permitida para crear el agarre
GRAB_DISTANCE = 0.105

# Duración del PICK automático
APPROACH_TIME = 0.45
LOWER_TIME = 0.20
LIFT_TIME = 0.55

# Restricción del movimiento manual
LIMIT_X = 0.40
LIMIT_Z = 0.30


# ============================================================
# ESTADO
# ============================================================

gripper_open = True
grip_constraint = None


# ============================================================
# UTILIDADES
# ============================================================

def get_link_position(body_id, link_id):
    state = p.getLinkState(
        body_id,
        link_id,
        computeForwardKinematics=True
    )
    return np.array(state[4], dtype=float)


def get_joint_index(body_id, name):
    for i in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, i)
        joint_name = info[1].decode("utf-8")

        if joint_name == name:
            return i

    raise RuntimeError(
        f"No se encontró la articulación {name}"
    )


# ============================================================
# GRIPPER
# ============================================================

def set_gripper(body_id, opened):
    left_finger = get_joint_index(
        body_id,
        "l_gripper_l_finger_joint"
    )

    right_finger = get_joint_index(
        body_id,
        "l_gripper_r_finger_joint"
    )

    if opened:
        left_target = 0.020
        right_target = -0.020
    else:
        left_target = 0.0
        right_target = 0.0

    p.setJointMotorControl2(
        body_id,
        left_finger,
        p.POSITION_CONTROL,
        targetPosition=left_target,
        force=FINGER_FORCE
    )

    p.setJointMotorControl2(
        body_id,
        right_finger,
        p.POSITION_CONTROL,
        targetPosition=right_target,
        force=FINGER_FORCE
    )


# ============================================================
# IK
# ============================================================

def ik_solution(body_id, ee_id, target):
    orientation = p.getQuaternionFromEuler(
        [0.0, np.pi / 2.0, 0.0]
    )

    return p.calculateInverseKinematics(
        body_id,
        ee_id,
        targetPosition=target.tolist(),
        targetOrientation=orientation,
        maxNumIterations=50,
        residualThreshold=1e-3
    )


def apply_ik(body_id, solution):
    for jid in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, jid)

        joint_type = info[2]
        q_index = info[3]

        if joint_type not in (
            p.JOINT_REVOLUTE,
            p.JOINT_PRISMATIC
        ):
            continue

        idx = q_index - 7

        if 0 <= idx < len(solution):
            p.setJointMotorControl2(
                body_id,
                jid,
                p.POSITION_CONTROL,
                targetPosition=float(solution[idx]),
                force=JOINT_FORCE,
                positionGain=0.35,
                velocityGain=1.0
            )


def command_hand(body_id, ee_id, target):
    solution = ik_solution(
        body_id,
        ee_id,
        target
    )

    apply_ik(
        body_id,
        solution
    )


def move_hand_smooth(
    body_id,
    ee_id,
    start,
    end,
    duration
):
    steps = max(
        1,
        int(duration / DT)
    )

    for k in range(steps):
        alpha = (
            (k + 1)
            / steps
        )

        alpha = (
            alpha * alpha
            * (3.0 - 2.0 * alpha)
        )

        target = (
            (1.0 - alpha) * start
            + alpha * end
        )

        command_hand(
            body_id,
            ee_id,
            target
        )

        p.stepSimulation()
        time.sleep(DT)


# ============================================================
# MESA
# ============================================================

def create_table(center_x, center_y, center_z):
    """
    La mesa se coloca DIRECTAMENTE debajo del brazo y del objeto.
    Su centro se calcula a partir de la posición del objeto.
    """

    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[
            TABLE_HALF_X,
            TABLE_HALF_Y,
            TABLE_THICKNESS / 2.0
        ]
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[
            TABLE_HALF_X,
            TABLE_HALF_Y,
            TABLE_THICKNESS / 2.0
        ],
        rgbaColor=[
            0.25,
            0.30,
            0.35,
            1.0
        ]
    )

    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[
            center_x,
            center_y,
            center_z
        ]
    )


# ============================================================
# OBJETO
# ============================================================

def create_object(position):
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[
            OBJECT_HALF,
            OBJECT_HALF,
            OBJECT_HALF
        ]
    )

    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[
            OBJECT_HALF,
            OBJECT_HALF,
            OBJECT_HALF
        ],
        rgbaColor=[
            0.95,
            0.65,
            0.05,
            1.0
        ]
    )

    object_id = p.createMultiBody(
        baseMass=0.08,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position.tolist()
    )

    # Evita que el objeto rebote demasiado.
    p.changeDynamics(
        object_id,
        -1,
        lateralFriction=0.9,
        rollingFriction=0.05,
        spinningFriction=0.05,
        restitution=0.0,
        linearDamping=0.6,
        angularDamping=0.8
    )

    return object_id


# ============================================================
# AGARRE
# ============================================================

def grab_object(
    baxter_id,
    ee_id,
    obj_id
):
    global grip_constraint

    if grip_constraint is not None:
        return True

    hand = get_link_position(
        baxter_id,
        ee_id
    )

    obj = np.array(
        p.getBasePositionAndOrientation(
            obj_id
        )[0],
        dtype=float
    )

    distance = np.linalg.norm(
        hand - obj
    )

    print(
        f"[PICK] Distancia mano-objeto: "
        f"{distance:.3f} m"
    )

    if distance > GRAB_DISTANCE:
        print(
            "[PICK] No se pudo agarrar: "
            "mano demasiado lejos."
        )
        return False

    grip_constraint = p.createConstraint(
        parentBodyUniqueId=baxter_id,
        parentLinkIndex=ee_id,
        childBodyUniqueId=obj_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )

    print("[PICK] OBJETO AGARRADO")
    return True


def release_object():
    global grip_constraint

    if grip_constraint is not None:
        p.removeConstraint(
            grip_constraint
        )
        grip_constraint = None

    print("[GRIP] OBJETO SOLTADO")


# ============================================================
# PICK AUTOMÁTICO
# ============================================================

def automatic_pick(
    baxter_id,
    ee_id,
    obj_id,
    current_target
):
    object_pos = np.array(
        p.getBasePositionAndOrientation(
            obj_id
        )[0],
        dtype=float
    )

    # El objeto está en la mesa.
    # La mano se acerca justo por encima.
    approach = object_pos + np.array([
        0.0,
        0.0,
        0.050
    ])

    grasp = object_pos + np.array([
        0.0,
        0.0,
        0.025
    ])

    print("[PICK] Acercando la mano...")

    move_hand_smooth(
        baxter_id,
        ee_id,
        current_target.copy(),
        approach,
        APPROACH_TIME
    )

    print("[PICK] Bajando...")

    move_hand_smooth(
        baxter_id,
        ee_id,
        approach,
        grasp,
        LOWER_TIME
    )

    print("[PICK] Cerrando gripper...")

    set_gripper(
        baxter_id,
        False
    )

    for _ in range(30):
        p.stepSimulation()

    grabbed = grab_object(
        baxter_id,
        ee_id,
        obj_id
    )

    if not grabbed:
        return get_link_position(
            baxter_id,
            ee_id
        )

    print("[PICK] Levantando...")

    lift = grasp + np.array([
        0.0,
        0.0,
        0.12
    ])

    move_hand_smooth(
        baxter_id,
        ee_id,
        grasp,
        lift,
        LIFT_TIME
    )

    return lift


# ============================================================
# SERIAL
# ============================================================

def parse_joystick(line):
    if not line.startswith("JOY,"):
        return None

    parts = line.split(",")

    if len(parts) != 4:
        return None

    try:
        x = float(parts[1])
        z = float(parts[2])
        button = int(parts[3])
    except ValueError:
        return None

    return (
        float(np.clip(x, -1.0, 1.0)),
        float(np.clip(z, -1.0, 1.0)),
        1 if button else 0
    )


# ============================================================
# MAIN
# ============================================================

def main():
    global gripper_open

    print("=" * 65)
    print("ACTIVIDAD B - BAXTER + JOYSTICK")
    print("VERSIÓN CON MESA CENTRADA EN EL BRAZO")
    print("=" * 65)

    # --------------------------------------------------------
    # ESP32
    # --------------------------------------------------------

    try:
        ser = serial.Serial(
            SERIAL_PORT,
            BAUD_RATE,
            timeout=0
        )

        time.sleep(2.0)

        print(
            f"[OK] ESP32 conectada en {SERIAL_PORT}"
        )

    except serial.SerialException as exc:
        print("\nERROR DE PUERTO")
        print(
            f"No se pudo abrir {SERIAL_PORT}."
        )
        print(
            "Cambia SERIAL_PORT por el COM correcto."
        )
        print(
            "Cierra el Monitor Serial del Arduino IDE."
        )
        print(exc)
        return

    # --------------------------------------------------------
    # PYBULLET
    # --------------------------------------------------------

    p.connect(p.GUI)
    p.resetSimulation()

    p.setAdditionalSearchPath(
        pybullet_data.getDataPath()
    )

    p.setGravity(
        0,
        0,
        -9.81
    )

    p.setTimeStep(DT)

    p.loadURDF(
        "plane.urdf",
        [0, 0, -1.0],
        useFixedBase=True
    )

    # --------------------------------------------------------
    # BAXTER
    # --------------------------------------------------------

    baxter_id = p.loadURDF(
        "baxter_common/baxter_description/urdf/toms_baxter.urdf",
        basePosition=[0.5, -0.8, 0.0],
        baseOrientation=[0.0, 0.0, -1.0, -1.0],
        useFixedBase=True
    )

    ee_id = 48

    # Dejar estabilizar
    for _ in range(120):
        p.stepSimulation()

    hand_start = get_link_position(
        baxter_id,
        ee_id
    )

    print("\nEfector izquierdo:")
    print(
        np.round(
            hand_start,
            4
        )
    )

    # --------------------------------------------------------
    # OBJETO Y MESA
    # --------------------------------------------------------

    object_pos = hand_start.copy()

    # El objeto queda debajo de la mano.
    object_pos[2] -= HAND_TO_OBJECT

    # La cara inferior del cubo queda:
    # object_z - OBJECT_HALF
    object_bottom = (
        object_pos[2]
        - OBJECT_HALF
    )

    # La parte superior de la mesa queda 1 mm debajo
    # de la cara inferior del cubo.
    table_top = (
        object_bottom
        - 0.001
    )

    table_center_z = (
        table_top
        - TABLE_THICKNESS / 2.0
    )

    # MUY IMPORTANTE:
    # El centro X/Y de la mesa es el del objeto,
    # no el centro global del escenario.
    table_center_x = object_pos[0]
    table_center_y = object_pos[1]

    print("\nMESA:")
    print(
        "Centro X/Y =",
        round(table_center_x, 3),
        round(table_center_y, 3)
    )

    print(
        "Centro Z =",
        round(table_center_z, 3)
    )

    table_id = create_table(
        table_center_x,
        table_center_y,
        table_center_z
    )

    object_id = create_object(
        object_pos
    )

    # Dar un momento para que el objeto quede estable.
    for _ in range(30):
        p.stepSimulation()

    # Leer de nuevo su posición por si el solver lo ajustó.
    object_pos = np.array(
        p.getBasePositionAndOrientation(
            object_id
        )[0],
        dtype=float
    )

    print("\nOBJETO:")
    print(
        np.round(
            object_pos,
            4
        )
    )

    # --------------------------------------------------------
    # MARCADORES VISUALES
    # --------------------------------------------------------

    p.addUserDebugText(
        "MESA",
        [
            table_center_x,
            table_center_y,
            table_top + 0.03
        ],
        textColorRGB=[
            0.8,
            0.8,
            0.8
        ],
        textSize=1.2,
        lifeTime=0
    )

    p.addUserDebugText(
        "OBJETO",
        object_pos + np.array([
            0,
            0,
            0.07
        ]),
        textColorRGB=[
            1,
            0.8,
            0
        ],
        textSize=1.2,
        lifeTime=0
    )

    # --------------------------------------------------------
    # CÁMARA
    # --------------------------------------------------------

    p.resetDebugVisualizerCamera(
        cameraDistance=1.9,
        cameraYaw=180,
        cameraPitch=-8,
        cameraTargetPosition=[
            object_pos[0],
            object_pos[1],
            object_pos[2]
        ]
    )

    # --------------------------------------------------------
    # GRIPPER
    # --------------------------------------------------------

    set_gripper(
        baxter_id,
        True
    )

    gripper_open = True

    # --------------------------------------------------------
    # CONTROL MANUAL
    # --------------------------------------------------------

    target = hand_start.copy()

    fixed_y = hand_start[1]

    x_min = (
        hand_start[0]
        - LIMIT_X
    )

    x_max = (
        hand_start[0]
        + LIMIT_X
    )

    z_min = (
        hand_start[2]
        - LIMIT_Z
    )

    z_max = (
        hand_start[2]
        + LIMIT_Z
    )

    previous_button = 0
    last_serial = time.time()

    print("\n" + "=" * 65)
    print("CONTROLES")
    print("=" * 65)
    print("VRx -> X")
    print("VRy -> Z")
    print("SW  -> agarrar / soltar")
    print("=" * 65)

    # --------------------------------------------------------
    # LOOP
    # --------------------------------------------------------

    try:

        while True:

            joystick_x = 0.0
            joystick_z = 0.0
            button = 0

            while ser.in_waiting:

                raw = (
                    ser.readline()
                    .decode(
                        "utf-8",
                        errors="ignore"
                    )
                    .strip()
                    .upper()
                )

                parsed = parse_joystick(
                    raw
                )

                if parsed is None:
                    continue

                (
                    joystick_x,
                    joystick_z,
                    button
                ) = parsed

                last_serial = time.time()

            # Watchdog
            if (
                time.time()
                - last_serial
                > 0.5
            ):
                joystick_x = 0.0
                joystick_z = 0.0

            # Movimiento X
            target[0] += (
                joystick_x
                * MOVE_SPEED_X
                * DT
            )

            # Movimiento Z
            target[2] += (
                joystick_z
                * MOVE_SPEED_Z
                * DT
            )

            # Y fija
            target[1] = fixed_y

            # Límites
            target[0] = np.clip(
                target[0],
                x_min,
                x_max
            )

            target[2] = np.clip(
                target[2],
                z_min,
                z_max
            )

            # ------------------------------------------------
            # BOTÓN
            # ------------------------------------------------

            if (
                previous_button == 0
                and button == 1
            ):

                if gripper_open:

                    print(
                        "\n[PICK] Iniciando..."
                    )

                    target = automatic_pick(
                        baxter_id,
                        ee_id,
                        object_id,
                        target
                    )

                    gripper_open = False

                else:

                    print(
                        "\n[RELEASE] Soltando..."
                    )

                    set_gripper(
                        baxter_id,
                        True
                    )

                    for _ in range(25):
                        p.stepSimulation()

                    release_object()

                    gripper_open = True

            previous_button = button

            # ------------------------------------------------
            # IK
            # ------------------------------------------------

            solution = ik_solution(
                baxter_id,
                ee_id,
                target
            )

            apply_ik(
                baxter_id,
                solution
            )

            p.stepSimulation()

            time.sleep(DT)

    except KeyboardInterrupt:
        print("\nPrograma detenido.")

    finally:

        try:
            ser.close()
        except Exception:
            pass

        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()

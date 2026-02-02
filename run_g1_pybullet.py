import pybullet as p
import pybullet_data
import time
import random
import numpy as np
import cv2
import os

# ================== ПАРАМЕТРЫ ==================
ROOM_HALF = 2.0
WALL_THICK = 0.05
WALL_HEIGHT = 1.0

CUBE_SIZE = 0.3
CUBE_HALF = CUBE_SIZE / 2
CUBE_MARGIN = 0.4

STOP_DISTANCE = 0.5

# ================== НАСТРОЙКИ ПОВЕДЕНИЯ ==================
CENTER_TOLERANCE = 25      # немного увеличен для быстрого центрирования
FORWARD_SPEED = 1.8        # УВЕЛИЧЕНО: базовая скорость движения вперед (было 1.2)
TURN_SPEED_SEARCH = 3.5    # УВЕЛИЧЕНО: скорость поворота при поиске (было 2.5)
TURN_SPEED_CENTER = 3.0    # УВЕЛИЧЕНО: скорость поворота при центрировании
TURN_GAIN = 0.012          # УВЕЛИЧЕНО: коэффициент реакции на ошибку (было 0.008)

# ================== PYBULLET INIT ==================
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

root = os.path.dirname(os.path.abspath(__file__))
p.setAdditionalSearchPath(root)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")

# ================== СТЕНЫ ==================
wall_col_ns = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[ROOM_HALF, WALL_THICK, WALL_HEIGHT / 2]
)

wall_vis_ns = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[ROOM_HALF, WALL_THICK, WALL_HEIGHT / 2],
    rgbaColor=[0.7, 0.7, 0.7, 1]
)

wall_col_ew = p.createCollisionShape(
    p.GEOM_BOX, halfExtents=[WALL_THICK, ROOM_HALF, WALL_HEIGHT / 2]
)

wall_vis_ew = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[WALL_THICK, ROOM_HALF, WALL_HEIGHT / 2],
    rgbaColor=[0.7, 0.7, 0.7, 1]
)

p.createMultiBody(0, wall_col_ns, wall_vis_ns, [0, ROOM_HALF, WALL_HEIGHT / 2])
p.createMultiBody(0, wall_col_ns, wall_vis_ns, [0, -ROOM_HALF, WALL_HEIGHT / 2])
p.createMultiBody(0, wall_col_ew, wall_vis_ew, [ROOM_HALF, 0, WALL_HEIGHT / 2])
p.createMultiBody(0, wall_col_ew, wall_vis_ew, [-ROOM_HALF, 0, WALL_HEIGHT / 2])

# ================== ROBOT ==================
yaw0 = random.uniform(0, 2 * np.pi)

urdf_path = os.path.join("unitree_g1", "urdf", "urdf", "g1_description.urdf")

robot_id = p.loadURDF(
    urdf_path,
    basePosition=[0, 0, 0.85],
    baseOrientation=p.getQuaternionFromEuler([0, 0, yaw0]),
    useFixedBase=False,
    flags=p.URDF_USE_INERTIA_FROM_FILE
)

# Оптимизированные параметры для устойчивости и движения
p.changeDynamics(robot_id, -1, 
                 linearDamping=0.4,       # Уменьшили для более быстрого движения
                 angularDamping=0.3,      # Уменьшили для быстрого поворота
                 lateralFriction=0.85,    # Уменьшили для лучшего скольжения
                 spinningFriction=0.05,
                 rollingFriction=0.05,
                 mass=40.0)               # Уменьшили массу для большей маневренности

# Даём роботу время "осесть" на полу
print("🤖 Робот загружается, подождите...")
for _ in range(60):  # Уменьшили время оседания
    p.stepSimulation()
    time.sleep(1/240)

# ================== КУБ ==================
def sample_cube():
    return (
        random.uniform(-ROOM_HALF + CUBE_MARGIN, ROOM_HALF - CUBE_MARGIN),
        random.uniform(-ROOM_HALF + CUBE_MARGIN, ROOM_HALF - CUBE_MARGIN)
    )

cube_x, cube_y = sample_cube()

cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[CUBE_HALF] * 3)
cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[CUBE_HALF] * 3, rgbaColor=[0, 1, 0, 1])

cube_id = p.createMultiBody(1, cube_col, cube_vis, [cube_x, cube_y, CUBE_HALF])
p.changeDynamics(cube_id, -1, lateralFriction=0.8)

print(f"🎯 Куб создан на позиции: ({cube_x:.2f}, {cube_y:.2f})")

# ================== КАМЕРА ==================
near, far = 0.1, 6.0
projection = p.computeProjectionMatrixFOV(75, 1.0, near, far)

LOWER_GREEN = np.array([35, 80, 80])
UPPER_GREEN = np.array([85, 255, 255])

# ================== FSM ==================
SEARCH, MOVE, STOP = 0, 1, 2
state = SEARCH

print("🚀 Симуляция запущена. Нажмите 'q' для выхода.")

# ================== LOOP ==================
frame_count = 0
last_print_time = time.time()

while p.isConnected():
    p.stepSimulation()
    frame_count += 1

    # ---- ПОЛУЧАЕМ ПОЗИЦИЮ РОБОТА ----
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    _, _, yaw = p.getEulerFromQuaternion(orn)

    # ---- КАМЕРА ----
    cam_x, cam_y = pos[0], pos[1]
    cam_z = pos[2] + 0.9
    pitch = -55 * np.pi / 180

    target = [
        cam_x + np.cos(yaw) * np.cos(pitch),
        cam_y + np.sin(yaw) * np.cos(pitch),
        cam_z + np.sin(pitch)
    ]

    view = p.computeViewMatrix([cam_x, cam_y, cam_z], target, [0, 0, 1])

    w, h, rgb, depth, _ = p.getCameraImage(320, 320, view, projection)

    img = np.reshape(rgb, (h, w, 4))[:, :, :3].astype(np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    depth = np.reshape(depth, (h, w))

    # ---- VISION ----
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)

    # Улучшение маски
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cx, distance = None, None
    if contours:
        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area > 50:  # ЕЩЁ УМЕНЬШИЛИ для более раннего обнаружения
            x, y, w2, h2 = cv2.boundingRect(c)
            cx = x + w2 // 2
            cy = y + h2 // 2

            # Быстрое измерение глубины
            d = np.clip(depth[cy, cx], 1e-3, 0.999)
            distance = (far * near) / (far - (far - near) * d)
            
            cv2.rectangle(img, (x, y), (x + w2, y + h2), (0, 0, 255), 2)
            cv2.putText(img, f"Area: {int(area)}", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    CENTER_X = img.shape[1] // 2

    # ---- ОТЛАДОЧНАЯ ИНФОРМАЦИЯ НА ЭКРАНЕ ----
    state_names = ["SEARCH", "MOVE", "STOP"]
    state_colors = [(0, 255, 255), (0, 255, 0), (0, 0, 255)]
    
    cv2.putText(img, f"State: {state_names[state]}", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_colors[state], 2)
    cv2.putText(img, f"Yaw: {np.degrees(yaw):.1f}°", (10, 60),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(img, f"Speed: {FORWARD_SPEED:.1f} m/s", (10, 90),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    if distance:
        cv2.putText(img, f"Dist: {distance:.2f}m", (10, 120),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    if cx is not None:
        # Линия от центра к кубу
        cv2.line(img, (CENTER_X, h//2), (cx, h//2), (0, 255, 255), 2)
        # Показываем ошибку
        error_text = f"Error: {cx - CENTER_X}"
        cv2.putText(img, error_text, (10, 150),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Центральная линия
    cv2.line(img, (CENTER_X, 0), (CENTER_X, h), (255, 255, 255), 1)

    # ---- FSM ----
    if state == SEARCH:
        # БЫСТРОЕ вращение на месте
        p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, TURN_SPEED_SEARCH])
        
        if cx is not None and distance is not None:
            print(f"\n🎯 Куб обнаружен на дистанции: {distance:.2f}m")
            state = MOVE
            # Очень короткая пауза
            for _ in range(3):
                p.stepSimulation()
                time.sleep(1/240)

    elif state == MOVE:
        if cx is None or distance is None:
            print("⚠️ Куб потерян из виду. Возврат в SEARCH")
            state = SEARCH
            p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
        else:
            error = cx - CENTER_X
            # Более резкий поворот к цели
            turn = float(np.clip(-error * TURN_GAIN, -2.0, 2.0))  # Увеличили лимит
            
            # =========== ЦЕНТРИРОВАНИЕ vs ДВИЖЕНИЕ ВПЕРЁД ===========
            if abs(error) > CENTER_TOLERANCE:
                # ❗ ФАЗА ЦЕНТРИРОВАНИЯ: ТОЛЬКО ПОВОРОТ
                # БЫСТРЫЙ поворот без движения вперед
                p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, turn])
                
                # Показываем зону центрирования
                cv2.rectangle(img, (CENTER_X - CENTER_TOLERANCE, 0), 
                            (CENTER_X + CENTER_TOLERANCE, h), (0, 100, 255), 1)
            else:
                # ❗ ФАЗА ДВИЖЕНИЯ ВПЕРЁД: Кубик в центре
                # ДИНАМИЧЕСКАЯ скорость: быстрее на дальних дистанциях
                if distance > 3.0:
                    current_speed = FORWARD_SPEED * 1.2  # Ускорение на дальних дистанциях
                elif distance > 1.5:
                    current_speed = FORWARD_SPEED
                else:
                    current_speed = FORWARD_SPEED * min(1.0, distance / 1.2)  # Замедление вблизи
                
                local_vx = current_speed
                
                # Преобразуем в глобальные координаты
                vx = local_vx * np.cos(yaw)
                vy = local_vx * np.sin(yaw)
                
                # Плавный поворот при движении
                smooth_turn = turn * 0.4
                p.resetBaseVelocity(robot_id, [vx, vy, 0], [0, 0, smooth_turn])
                
                # Показываем текущую скорость
                cv2.putText(img, f"Current: {current_speed:.1f} m/s", (10, 210),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 100), 2)
                
                # Показываем, что в целевой зоне
                cv2.rectangle(img, (CENTER_X - CENTER_TOLERANCE, 0), 
                            (CENTER_X + CENTER_TOLERANCE, h), (0, 255, 0), 1)
            
            # =========== ВИЗУАЛИЗАЦИЯ ===========
            # Рисуем стрелку направления
            arrow_length = 50
            center = (CENTER_X, img.shape[0] - 30)
            end_x = int(center[0] + arrow_length * np.cos(yaw))
            end_y = int(center[1] + arrow_length * np.sin(yaw))
            cv2.arrowedLine(img, center, (end_x, end_y), (0, 255, 0), 3)
            
            # Показываем режим (центрирование или движение)
            mode_text = "CENTERING" if abs(error) > CENTER_TOLERANCE else "FORWARD"
            mode_color = (0, 255, 255) if abs(error) > CENTER_TOLERANCE else (0, 255, 0)
            cv2.putText(img, f"Mode: {mode_text}", (10, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
            
            if distance <= STOP_DISTANCE:
                print(f"🛑 Достигнута дистанция {distance:.2f}m. STOP")
                state = STOP
                # Быстрая остановка
                for _ in range(5):
                    p.stepSimulation()
                    time.sleep(1/240)

    elif state == STOP:
        p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
        
        # Показываем успех
        cv2.putText(img, "TARGET REACHED!", (CENTER_X - 100, h//2),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # Быстрый возврат к поиску (через 1 секунду)
        if frame_count % 240 == 0:  # 1 секунда при 240 FPS
            print("🔁 Возврат к поиску...")
            cube_x, cube_y = sample_cube()
            p.resetBasePositionAndOrientation(cube_id, [cube_x, cube_y, CUBE_HALF], [0, 0, 0, 1])
            print(f"🎯 Новый куб на позиции: ({cube_x:.2f}, {cube_y:.2f})")
            
            # Быстрый поворот в случайном направлении
            state = SEARCH

    # ---- ОТОБРАЖЕНИЕ И УПРАВЛЕНИЕ ----
    cv2.imshow("camera", img)
    
    # Периодический вывод информации в консоль
    current_time = time.time()
    if current_time - last_print_time > 0.5:  # Каждые 0.5 секунды
        current_lin_vel, current_ang_vel = p.getBaseVelocity(robot_id)
        actual_speed = np.sqrt(current_lin_vel[0]**2 + current_lin_vel[1]**2)
        print(f"\r🤖 State: {state_names[state]:8s} | Pos: ({pos[0]:5.2f}, {pos[1]:5.2f}) | Speed: {actual_speed:.1f} m/s | Dist: {distance if distance else 0:.2f}m", end="")
        last_print_time = current_time
    
    # Горячие клавиши для управления скоростью
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        print("\n👋 Выход по команде пользователя")
        break
    elif key == ord('+') or key == ord('='):
        FORWARD_SPEED = min(3.0, FORWARD_SPEED + 0.2)  # Увеличили шаг
        print(f"\n⚡ Скорость увеличена до: {FORWARD_SPEED:.1f} m/s")
    elif key == ord('-') or key == ord('_'):
        FORWARD_SPEED = max(0.3, FORWARD_SPEED - 0.2)  # Увеличили шаг
        print(f"\n🐌 Скорость уменьшена до: {FORWARD_SPEED:.1f} m/s")
    elif key == ord('w'):
        # Экстремальное ускорение
        FORWARD_SPEED = 2.5
        TURN_SPEED_SEARCH = 4.5
        TURN_GAIN = 0.015
        print(f"\n🔥 ТУРБО РЕЖИМ! Speed: {FORWARD_SPEED:.1f} m/s")
    elif key == ord('s'):
        # Нормальный режим
        FORWARD_SPEED = 1.8
        TURN_SPEED_SEARCH = 3.5
        TURN_GAIN = 0.012
        print(f"\n⚡ Нормальный режим: {FORWARD_SPEED:.1f} m/s")
    elif key == ord('r'):
        # Рестарт симуляции
        print("\n🔄 Рестарт симуляции...")
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        # Перезагружаем робота
        yaw0 = random.uniform(0, 2 * np.pi)
        robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0.85],
            baseOrientation=p.getQuaternionFromEuler([0, 0, yaw0]),
            useFixedBase=False,
            flags=p.URDF_USE_INERTIA_FROM_FILE
        )
        p.changeDynamics(robot_id, -1, linearDamping=0.4, angularDamping=0.3, mass=40.0)
        # Новый куб
        cube_x, cube_y = sample_cube()
        cube_id = p.createMultiBody(1, cube_col, cube_vis, [cube_x, cube_y, CUBE_HALF])
        state = SEARCH
        print(f"🔄 Новая симуляция | Куб: ({cube_x:.2f}, {cube_y:.2f})")

    time.sleep(1 / 240)

cv2.destroyAllWindows()
p.disconnect()
print("\n✅ Симуляция завершена")
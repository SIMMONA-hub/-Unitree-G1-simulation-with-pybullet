import pybullet as p
import pybullet_data
import time
import random
import numpy as np
import cv2
  
ROOM_HALF = 2.0
WALL_THICK = 0.05
WALL_HEIGHT = 1.0

CUBE_SIZE = 0.3
CUBE_HALF = CUBE_SIZE / 2
CUBE_MARGIN = 0.4

ROBOT_RADIUS = 0.18
ROBOT_HEIGHT = 0.4
ROBOT_Z = 0.2
ROBOT_MARGIN = ROBOT_RADIUS + 0.25

 
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

 
wall_col_ns = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[ROOM_HALF, WALL_THICK, WALL_HEIGHT / 2]
)
wall_vis_ns = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[ROOM_HALF, WALL_THICK, WALL_HEIGHT / 2],
    rgbaColor=[0.7, 0.7, 0.7, 1]
)

wall_col_ew = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[WALL_THICK, ROOM_HALF, WALL_HEIGHT / 2]
)
wall_vis_ew = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[WALL_THICK, ROOM_HALF, WALL_HEIGHT / 2],
    rgbaColor=[0.7, 0.7, 0.7, 1]
)

# Север / Юг
p.createMultiBody(0, wall_col_ns, wall_vis_ns, [0,  ROOM_HALF, WALL_HEIGHT / 2])
p.createMultiBody(0, wall_col_ns, wall_vis_ns, [0, -ROOM_HALF, WALL_HEIGHT / 2])
# Восток / Запад
p.createMultiBody(0, wall_col_ew, wall_vis_ew, [ ROOM_HALF, 0, WALL_HEIGHT / 2])
p.createMultiBody(0, wall_col_ew, wall_vis_ew, [-ROOM_HALF, 0, WALL_HEIGHT / 2])

#ROBOT BASE
robot_x = 0.0
robot_y = 0.0
yaw0 = random.uniform(0, 2 * np.pi)

robot_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=ROBOT_RADIUS, height=ROBOT_HEIGHT)
robot_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=ROBOT_RADIUS, length=ROBOT_HEIGHT, rgbaColor=[0.2, 0.2, 1, 1])

robot_id = p.createMultiBody(
    baseMass=10,
    baseCollisionShapeIndex=robot_col,
    baseVisualShapeIndex=robot_vis,
    basePosition=[robot_x, robot_y, ROBOT_Z],
    baseOrientation=p.getQuaternionFromEuler([0, 0, yaw0])
)

p.changeDynamics(robot_id, -1, lateralFriction=0.8, spinningFriction=0.001, rollingFriction=0.001)
p.changeDynamics(robot_id, -1, linearDamping=0.0, angularDamping=0.0)

#кубик
def sample_cube_pos():
    x = random.uniform(-ROOM_HALF + CUBE_MARGIN, ROOM_HALF - CUBE_MARGIN)
    y = random.uniform(-ROOM_HALF + CUBE_MARGIN, ROOM_HALF - CUBE_MARGIN)
    return x, y

cube_x, cube_y = sample_cube_pos()
while np.hypot(cube_x - robot_x, cube_y - robot_y) < 1.0:
    cube_x, cube_y = sample_cube_pos()

cube_z = CUBE_HALF

cube_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[CUBE_HALF] * 3)
cube_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[CUBE_HALF] * 3, rgbaColor=[0, 1, 0, 1])

cube_id = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=cube_col,
    baseVisualShapeIndex=cube_vis,
    basePosition=[cube_x, cube_y, cube_z]
)



#камера 
near = 0.1
far = 6.0

projection_matrix = p.computeProjectionMatrixFOV(
    fov=75,
    aspect=1.0,
    nearVal=near,
    farVal=far
)

 
cv2.namedWindow("Robot Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Robot Camera", 420, 420)

LOWER_GREEN = np.array([35, 80, 80])
UPPER_GREEN = np.array([85, 255, 255])

#фсм
SEARCH = 0
ALIGN = 1
MOVE = 2
STOP = 3
state = SEARCH
state_names = ["SEARCH", "ALIGN", "MOVE", "STOP"]

#управление
 
yaw_speed = 1.8          
forward_speed = 1.2       
forward_align_speed = 0.25   
TURN_GAIN = 0.01          
TURN_MAX = 2.5           

 
CENTER_TOL = 10           
STOP_DISTANCE = 0.5      
SLOW_DISTANCE = 0.9      
MIN_CONTOUR_AREA = 60    

#VISION SEARCH
PITCH_LEVELS = [
    -20 * np.pi / 180,
    -35 * np.pi / 180,
    -55 * np.pi / 180
]
pitch_index = 0
pitch = PITCH_LEVELS[pitch_index]

#TRACKING PARAMETERS
lost_counter = 0
LOST_LIMIT = 15

 
pos0, orn0 = p.getBasePositionAndOrientation(robot_id)
_, _, prev_yaw = p.getEulerFromQuaternion(orn0)
yaw_accum = 0.0
 
def normalize_angle(angle):
     
    return (angle + np.pi) % (2 * np.pi) - np.pi

def get_robot_pose():
    
    pos, orn = p.getBasePositionAndOrientation(robot_id)
    _, _, yaw = p.getEulerFromQuaternion(orn)
    return pos, yaw

# ROBOT CONTROL API  
 


# MAIN LOOP  
try:
    while p.isConnected():
        p.stepSimulation()
        
         
        pos, yaw = get_robot_pose()
        
       
        dy = normalize_angle(yaw - prev_yaw)
        yaw_accum += abs(dy)
        prev_yaw = yaw
        
       
        cam_x, cam_y = pos[0], pos[1]
        cam_z = pos[2] + 0.35   # примерно уровень "глаз"

        
       
        target_x = cam_x + np.cos(yaw) * np.cos(pitch)
        target_y = cam_y + np.sin(yaw) * np.cos(pitch)
        target_z = cam_z + np.sin(pitch)
        
        
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[cam_x, cam_y, cam_z],
            cameraTargetPosition=[target_x, target_y, target_z],
            cameraUpVector=[0, 0, 1]
        )
        
        
        width, height, rgb, depth, _ = p.getCameraImage(
            width=320,
            height=320,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix
        )
        
         
        img = np.reshape(rgb, (height, width, 4))[:, :, :3].astype(np.uint8)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        depth = np.reshape(depth, (height, width))
        
         
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        
       
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        CENTER_X = width // 2
        CENTER_Y = height // 2
        
        
        cube_found = False
        cx = cy = None
        distance = None
        c = None
        contour_area = 0
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            contour_area = cv2.contourArea(c)
            
            if contour_area > MIN_CONTOUR_AREA:
                x, y, w, h = cv2.boundingRect(c)
                cx = x + w // 2
                cy = y + h // 2
                cube_found = True
                
                 
                y1 = max(cy - 3, 0)
                y2 = min(cy + 4, height)
                x1 = max(cx - 3, 0)
                x2 = min(cx + 4, width)
                depth_patch = depth[y1:y2, x1:x2]
                depth_val = float(np.clip(np.median(depth_patch), 0.0, 1.0))
                
                distance = (far * near) / (far - (far - near) * depth_val)
                 
                cv2.rectangle(img_bgr, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(img_bgr, (cx, cy), 5, (255, 0, 0), -1)
                cv2.putText(
                    img_bgr,
                    f"{distance:.2f} m",
                    (x, max(15, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )
        
     
        if cube_found:
            lost_counter = 0
        else:
            lost_counter += 1
        
        #фсм логика
        
        if state == SEARCH:
            if cube_found:
                yaw_accum = 0.0
                state = ALIGN
                print(f"Переход в ALIGN: куб найден на расстоянии {distance:.2f} м")
            else:
                 
                p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, yaw_speed])
                
                # Смена pitch после полного оборота
                if yaw_accum >= 2 * np.pi:
                    yaw_accum = 0.0
                    pitch_index = min(pitch_index + 1, len(PITCH_LEVELS) - 1)
                    pitch = PITCH_LEVELS[pitch_index]
                    print(f"Смена pitch на {pitch*180/np.pi:.0f}°")
        
        elif state == ALIGN:
            if lost_counter > LOST_LIMIT:
                state = SEARCH
                pitch_index = 0
                pitch = PITCH_LEVELS[pitch_index]
                print("Переход в SEARCH: потерян куб")
            else:
                if cube_found:
                    error = cx - CENTER_X
                    turn = float(np.clip(-error * TURN_GAIN, -TURN_MAX, TURN_MAX))
                    
                     
                    vx = forward_align_speed * np.cos(yaw)
                    vy = forward_align_speed * np.sin(yaw)
                    
                    p.resetBaseVelocity(robot_id, [vx, vy, 0], [0, 0, turn])
                    
                    if abs(error) < CENTER_TOL:
                        state = MOVE
                        print("Переход в MOVE: куб отцентрирован")
        
        elif state == MOVE:
            if lost_counter > LOST_LIMIT:
                state = SEARCH
                pitch_index = 0
                pitch = PITCH_LEVELS[pitch_index]
                print("Переход в SEARCH: потерян куб в движении")
            else:
                if cube_found:
                    error_x = cx - CENTER_X
                    
                    # Dead zone для стабильного движения
                    if abs(error_x) < CENTER_TOL:
                        turn = 0.0
                    else:
                        turn = float(np.clip(-error_x * TURN_GAIN, -TURN_MAX, TURN_MAX))
                    
                    
                    pitch = -35 * np.pi / 180
                    
                    # Проверка дистанции для остановки
                    if distance is not None and distance <= STOP_DISTANCE:
                        state = STOP
                        print(f"Переход в STOP: достигнута дистанция {distance:.2f} м")
                    else:
                         
                        speed = forward_speed
                        if distance is not None and distance < SLOW_DISTANCE:
                             
                            slowdown_factor = 0.3 + 0.7 * (distance - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE)
                            speed *= max(slowdown_factor, 0.3)
                        
                        vx = speed * np.cos(yaw)
                        vy = speed * np.sin(yaw)
                        
                        p.resetBaseVelocity(robot_id, [vx, vy, 0], [0, 0, turn])
        
        elif state == STOP:
            p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
            
             
            if cube_found and distance is not None and distance > STOP_DISTANCE + 0.1:
                state = ALIGN
                print(f"Переход в ALIGN: куб отошёл на {distance:.2f} м")
            
         
            if lost_counter > LOST_LIMIT:
                state = SEARCH
                pitch_index = 0
                pitch = PITCH_LEVELS[pitch_index]
                print("Переход в SEARCH: потерян куб в STOP")
             
            if cube_found and c is not None:
                x, y, w, h = cv2.boundingRect(c)
                cv2.putText(
                    img_bgr,
                    "STOP",
                    (x, max(35, y - 30)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )
        
      
        dist_str = f"{distance:.2f}m" if distance is not None else "N/A"
        
     
        cv2.putText(
            img_bgr,
            f"State: {state_names[state]}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )
        
        cv2.putText(
            img_bgr,
            f"Pitch: {pitch*180/np.pi:.0f}° | Lost: {lost_counter}/{LOST_LIMIT}",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )
        
        cv2.putText(
            img_bgr,
            f"Distance: {dist_str} | Area: {contour_area:.0f}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )
        
        
        cv2.line(img_bgr, (CENTER_X, 0), (CENTER_X, height), (255, 255, 255), 1)
        cv2.line(img_bgr, (0, CENTER_Y), (width, CENTER_Y), (255, 255, 255), 1)
        
      
        cv2.imshow("Robot Camera", img_bgr)
        
       
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
         
        
        time.sleep(1. / 240.)

except KeyboardInterrupt:
    print("\Программа прервана пользователем")

finally:
    cv2.destroyAllWindows()
    p.disconnect()
    print("Симуляция завершена")
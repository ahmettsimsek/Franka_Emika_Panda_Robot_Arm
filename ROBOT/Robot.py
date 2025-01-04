import pybullet as p
import numpy as np
import mediapipe as mp
from colorama import Fore, init
import pybullet_data, time, cv2
init(autoreset=True)


# Main Dosyası

# PyBullet kurulum
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # PyBullet veri dosyaları için arama yolunu ayarla
p.setGravity(0, 0, -30)  # Yerçekimini etkinleştir
ur3_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)
print(f"{Fore.GREEN} Simülasyona Bağlandı.")

# Hareket etmeyen yüzey (kare taban) ekleme
table_half_extents = [4, 4, 0.02]  # Yarı boyutlar (1x1 metre taban, 2 cm kalınlık)
table_start_position = [0, 0, -0.02]  # Zemin seviyesinde yerleştirme
table_collision_shape = p.createCollisionShape(
    shapeType=p.GEOM_BOX, halfExtents=table_half_extents

)# Çarpışma şekli oluştur
table_visual_shape = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=table_half_extents,
    rgbaColor=[0.6, 0.3, 0.1, 1],  # Kahverengi zemin
)# Görsel şekil oluştur
table_id = p.createMultiBody(
    baseMass=0,  # Hareket etmeyen yüzey için kütle sıfır
    baseCollisionShapeIndex=table_collision_shape,
    baseVisualShapeIndex=table_visual_shape,
    basePosition=table_start_position,
)

# Kutu nesnesi ekleme
box_half_extents = [0.03, 0.03, 0.02]  # Yarı uzunluklar
box_start_position = [0.9, 0, 0.3]  # Kutunun başlangıç pozisyonu
box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_half_extents)
box_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=box_half_extents, rgbaColor=[1, 0, 0, 1])  # Kırmızı renk
box_object_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=box_id, baseVisualShapeIndex=box_visual_id, basePosition=box_start_position)

# Başka bir kutu nesnesi ekleme
box = [0.1, 0.1, 0.1]  # Yarı uzunluklar
start = [0.9, 0, 0.1]  # Kutunun başlangıç pozisyonu
ids = p.createCollisionShape(p.GEOM_BOX, halfExtents=box)
visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=box, rgbaColor=[1, 0, 1, 1])  # Kırmızı renk
obejct_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=ids, baseVisualShapeIndex=visual_id, basePosition=start)


# İki nokta arasındaki açıyı hesaplayan fonksiyon
def calculate_angle(a, b, c):
    a, b, c = np.array(a), np.array(b), np.array(c)
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)
    return 360 - angle if angle > 180.0 else angle

# MediaPipe kurulum
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands

# Kameradan görüntü yakalama
cap = cv2.VideoCapture(0)

# Poz ve el algılama için MediaPipe modellerini başlat
with mp_pose.Pose(min_detection_confidence=0.8, min_tracking_confidence=0.8) as pose:
    with mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5) as hands:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print(f"{Fore.RED} Hata: Kamera Erişim Sağlanamadı.")
                break

           
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            result = pose.process(image)   # Poz verilerini işleme
            hands_result = hands.process(image)  # El verilerini işleme
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Poz ve el verileri mevcutsa işlemleri gerçekleştir
            if result.pose_landmarks and hands_result.multi_hand_landmarks:
                landmarks = result.pose_landmarks.landmark

                 # Sağ omuz, dirsek ve bilek koordinatlarını al
                right_shoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
                right_elbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
                right_wrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
                right_hip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
                right_index = [landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].y]

                # Baş parmak ve işaret parmağı uçlarının koordinatlarını al
                thumb_tip = [hands_result.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.THUMB_TIP].x,
                             hands_result.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.THUMB_TIP].y]
                index_tip = [hands_result.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x,
                             hands_result.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y]

                 # Açıları hesapla
                shoulder_angle = calculate_angle(right_hip, right_shoulder, right_elbow)
                elbow_angle = calculate_angle(right_shoulder, right_elbow, right_wrist)
                wrist_angle = calculate_angle(right_elbow, right_wrist, index_tip)
                thumb_index_angle = calculate_angle(right_wrist, thumb_tip, index_tip)  

               # Açıları görüntü üzerine yazdır
                cv2.putText(image, str(int(shoulder_angle)), tuple(np.multiply(right_shoulder, [640, 480]).astype(int)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(image, str(int(elbow_angle)), tuple(np.multiply(right_elbow, [640, 480]).astype(int)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(image, str(int(wrist_angle)), tuple(np.multiply(right_wrist, [640, 480]).astype(int)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(image, f"{int(thumb_index_angle)}°", tuple(np.multiply(thumb_tip, [640, 480]).astype(int)), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

                # MediaPipe bağlantılarını çiz
                mp_drawing.draw_landmarks(image, result.pose_landmarks, mp_pose.POSE_CONNECTIONS)
                mp_drawing.draw_landmarks(image, hands_result.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)

                 # Simülasyonda eklemler için açıları ayarla
                shoulder_joint = (180 - shoulder_angle) * np.pi / 180 
                elbow_joint = elbow_angle * np.pi / 180
                wrist_joint = wrist_angle * np.pi / 180
                thump_joint = thumb_index_angle *np.pi / 180

                if thump_joint <= 1.0:
                    thump_joint = 0
                else:
                    pass
                
                print(f"Omuz Açısı: {shoulder_joint}")
       

                p.setJointMotorControl2(bodyIndex=ur3_id, jointIndex=5, controlMode=p.POSITION_CONTROL, targetPosition=wrist_joint)
                p.setJointMotorControl2(bodyIndex=ur3_id, jointIndex=3, controlMode=p.POSITION_CONTROL, targetPosition=elbow_joint) # Çözülecek sadece ilave dirsek eklenecek 
                p.setJointMotorControl2(bodyIndex=ur3_id, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=shoulder_joint)
                p.setJointMotorControl2(bodyIndex=ur3_id, jointIndex=9, controlMode=p.POSITION_CONTROL, targetPosition=thump_joint)
                


                
                p.stepSimulation()
                time.sleep(1./240.)

            cv2.imshow('El Pozu Tespiti', image)
            if cv2.waitKey(5) & 0xFF == 27:
                break

cap.release()
cv2.destroyAllWindows()
p.disconnect()

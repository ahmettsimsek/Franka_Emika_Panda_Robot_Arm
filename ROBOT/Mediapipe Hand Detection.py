import numpy as np  #Matematiksel hesaplamalar için kullanılan kütüphane.
import mediapipe as mp #el ve vücut tespiti gibi işlevleri barındıran kütüphane.
from colorama import Fore, init #Terminalde renkli metinler yazmak için kullanılır.
import time, cv2  #OpenCV kütüphanesi, görüntü işleme için kullanılır. time: Zamanlayıcılar ve bekleme süreleri için kullanılır.
init(autoreset=True)

"""
Python Mediapipe kullanırken yardımı dokunan bir link: https://lvimuth.medium.com/hand-detection-in-python-using-opencv-and-mediapipe-30c7b54f5ff4

----->    https://medium.com/@lukasz.kurant/high-performance-hand-landmark-detection-in-react-native-using-vision-camera-and-skia-frame-9ddec89362bc
----->    https://medium.com/@flip.flo.dev/real-time-hand-tracking-in-python-e2bcdd0feace
"""
"""
https://github.com/google-ai-edge/mediapipe/blob/master/docs/solutions/holistic.md  -----> mediapipe github linki
"""

# ---------------------------------
# Açıklama: Bu script dosyası Python Yazılım dili ile Mediapipe Modülünü kullanarak el omuz dirsek bilek açılarını hesaplayan bir script geliştirir
# ---------------------------------


def calculate_angle(a, b, c): #fonksiyon, 3 nokta alarak, bu noktalar arasındaki açıyı hesaplar.
    a, b, c = np.array(a), np.array(b), np.array(c)
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)
    return 360 - angle if angle > 180.0 else angle


# MediaPipe kurulum ve başlatılması
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands


cap = cv2.VideoCapture(0)  #bilgisayarın kamerasını başlatır. 0, ilk kamerayı temsil eder 



with mp_pose.Pose(min_detection_confidence=0.8, min_tracking_confidence=0.8) as pose:
    with mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5) as hands:
        while cap.isOpened():       
            ret, frame = cap.read()  #mp_pose.Pose ve mp_hands.Hands, vücut duruşu ve el takibini başlatır.
            if not ret:              #min_detection_confidence ve min_tracking_confidence, algılama ve takip güven seviyelerini ayarlar.
                print(f"{Fore.RED} Hata: Kamera Erişim Sağlanamadı.")
                break

           
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False     #Görüntü işleme yapılır. Kameradan alınan görüntü (frame), önce RGB formatına çevrilir.
            result = pose.process(image)      #pose.process() ve hands.process() fonksiyonları ile vücut ve el tespiti yapılır.
            hands_result = hands.process(image)  #İşlem tamamlandıktan sonra, görüntü tekrar BGR formatına dönüştürülür.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

          
            if result.pose_landmarks and hands_result.multi_hand_landmarks:   ##### (1-)	İnsan Eklem ve El Tespiti
                landmarks = result.pose_landmarks.landmark

                 #Eğer vücut ve eller tespit edildiyse, bu kısımdan vücut ve el noktalarının koordinatları alınır.
                 # Bu noktalar, omuz, dirsek, bilek gibi vücut eklemleri ve parmak uçlarıdır.
                right_shoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y] ####(2-)Koordinatların Alınması
                right_elbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
                right_wrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
                right_hip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
                right_index = [landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].x, landmarks[mp_pose.PoseLandmark.RIGHT_INDEX.value].y]

                
                thumb_tip = [hands_result.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.THUMB_TIP].x,
                             hands_result.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.THUMB_TIP].y]
                index_tip = [hands_result.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x,
                             hands_result.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y]

                
                #Burada, elde edilen vücut eklemleri arasında hesaplanan açıları ekrana yazdırılır. cv2.putText fonksiyonu ile bu açıların değerleri ekrana yerleştirilir.
                shoulder_angle = calculate_angle(right_hip, right_shoulder, right_elbow)    ####(3-)	Açı Hesaplama 
                elbow_angle = calculate_angle(right_shoulder, right_elbow, right_wrist)
                wrist_angle = calculate_angle(right_elbow, right_wrist, index_tip)
                thumb_index_angle = calculate_angle(right_wrist, thumb_tip, index_tip)  

                #####(4-) Açı Hesaplama
                cv2.putText(image, str(int(shoulder_angle)), tuple(np.multiply(right_shoulder, [640, 480]).astype(int)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(image, str(int(elbow_angle)), tuple(np.multiply(right_elbow, [640, 480]).astype(int)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(image, str(int(wrist_angle)), tuple(np.multiply(right_wrist, [640, 480]).astype(int)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(image, f"{int(thumb_index_angle)}°", tuple(np.multiply(thumb_tip, [640, 480]).astype(int)), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)


                #mp_drawing.draw_landmarks fonksiyonu ile tespit edilen vücut ve el noktaları çizilir, böylece ekranda işaretlenir.
                mp_drawing.draw_landmarks(image, result.pose_landmarks, mp_pose.POSE_CONNECTIONS)  #####(5-)MediaPipe Görselleştirme
                mp_drawing.draw_landmarks(image, hands_result.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)

                
                shoulder_joint = (180 - shoulder_angle) * np.pi / 180   #####(6-) Açıların Radyana Dönüştürülmesi 
                elbow_joint = elbow_angle * np.pi / 180
                wrist_joint = wrist_angle * np.pi / 180
                thump_joint = thumb_index_angle *np.pi / 180

                if thump_joint <= 1.0:   ####(7-) 	Başparmak Kontrolü
                    thump_joint = 0
                else:
                    pass
                
       

                time.sleep(1./240.)

            #Her işlenen kare, ekranda gösterilir. cv2.waitKey(5) ile kısa bir süre beklenir. 27 tuşu (ESC) basıldığında program sonlanır.
            cv2.imshow('El Pozu Tespiti', image)  
            if cv2.waitKey(5) & 0xFF == 27:
                break

cap.release()   #Kameradan alınan kaynak serbest bırakılır ve tüm OpenCV pencereleri kapatılır.
cv2.destroyAllWindows() 

#Özetleyecek olursam Bu kod, bir webcam görüntüsü üzerinden vücut pozisyonu ve el hareketlerini tespit eder, çeşitli eklem açılarını hesaplar ve ekranda gösterir.


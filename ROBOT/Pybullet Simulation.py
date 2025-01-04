import pybullet as p  #pybullet: Robot simülasyonu yapmak için kullanılan kütüphane.
from colorama import Fore, init
import pybullet_data  #pybullet_data: PyBullet için varsayılan veri yolunu almanızı sağlayan modül.
import time  
init(autoreset=True)

"""
Python Pybulelt için yardımı dokunan bir github reposu https://github.com/rparak/PyBullet_Industrial_Robotics_Gym

https://github.com/hsp-iit/pybullet-robot-envs/blob/master/pybullet_robot_envs/envs/panda_envs/panda_env.py

"""


# ---------------------------------
# Tarih: 
# Açıklama: Bu script dosyası Python Yazılım dili ile Pybullet Modülünü kullanarak belirtilen robot kolun istenilen açıda durmasını sağlar
# ---------------------------------


""" PyBullet kurulum """
#p.connect(p.GUI): PyBullet ile GUI (grafiksel kullanıcı arayüzü) üzerinden bağlantı kurar. Bu, robot simülasyonunun görsel olarak görüntülenmesini sağlar.
#p.setAdditionalSearchPath(pybullet_data.getDataPath()): PyBullet’in yerleşik veri yolunu ekler (örneğin, robot modelleri, URDF dosyaları).
#p.setGravity(0, 0, -9.8): Dünya üzerinde yerçekimini ayarlar (bu örnekte -9.8 m/s², y eksenine zıt yönde).
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())    ####(1) PyBullet Kurulumu  
p.setGravity(0, 0, -9.8)  



#p.loadURDF("franka_panda/panda.urdf", ...): franka_panda modelini yükler. URDF (Unified Robot Description Format) dosyası, robotun 3D modelini tanımlar.
#basePosition=[0, 0, 0]: Robotun başlangıç konumunu belirler (x, y, z koordinatları).
#useFixedBase=True: Robotun tabanının sabit olduğunu belirtir (yani masa üzerinde hareket etmeyecek).
ur3_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)   ####(2-) Robotun Tanımlanması
print(f"{Fore.GREEN} Simülasyona Bağlandı.")



#Bu bölümde bir masa oluşturuluyor. Masa için görsel şekil (table_visual_shape) ve çarpışma şekli (table_collision_shape) tanımlanıyor.
# Masa sabit bir objedir, bu yüzden baseMass=0 olarak ayarlanır. rgbaColor ile masa rengini ayarlayabilirsiniz
table_half_extents = [4, 4, 0.02]  
table_start_position = [0, 0, -0.02]  
table_collision_shape = p.createCollisionShape(
    shapeType=p.GEOM_BOX, halfExtents=table_half_extents
)
table_visual_shape = p.createVisualShape(    ######(3-) Çevresel Nesnelerin Eklenmesi masa ve kutu
    shapeType=p.GEOM_BOX,
    halfExtents=table_half_extents,
    rgbaColor=[0.6, 0.3, 0.1, 1],  
)
table_id = p.createMultiBody(
    baseMass=0,  
    baseCollisionShapeIndex=table_collision_shape,
    baseVisualShapeIndex=table_visual_shape,
    basePosition=table_start_position,
)



#Bu kısımda, simülasyona bir kutu ekleniyor. Kutu için çarpışma ve görsel şekil tanımlanıyor.
#  Kutu kırmızı renkte olacak şekilde ayarlanıyor ve masa üzerine yerleştiriliyor.
box_half_extents = [0.03, 0.03, 0.02]  
box_start_position = [0.9, 0, 0.3]  
box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=box_half_extents)
box_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=box_half_extents, rgbaColor=[1, 0, 0, 1])  # Kırmızı renk
box_object_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=box_id, baseVisualShapeIndex=box_visual_id, basePosition=box_start_position)



# Robot kol açı ayarları
wrist_joint = 0.4  # bilek ekleminin
elbow_joint = 0.4  #dirsek eklemi         ####(4-) Robot Eklem Açıları
shoulder_joint = 0.2   #omuz eklemi 
thump_joint = 0.2   #baş parmak eklemi



#Bu fonksiyon, belirli bir eklemin limitlerini (minimum ve maksimum açıları) almak için kullanılır. 
# getJointInfo ve getJointLimits PyBullet fonksiyonları, eklemin bilgilerini döndürür.
def get_joint_limits(joint_index):
    joint_info = p.getJointInfo(ur3_id, joint_index)
    joint_limits = p.getJointLimits(ur3_id, joint_index)     ######(5-) Eklemlerin Limitlerinin Alınması
    print(f"Joint {joint_index} limits: {joint_limits}")
    return joint_limits



# p.setJointMotorControl2 fonksiyonu ile robot kolunun her eklemine hedef açı atanır. 
# Örneğin, jointIndex=5 ile bilek eklemi kontrol edilir ve targetPosition=wrist_joint ile hedef açıya yönlendirilir. 

""""controlMode=p.POSITION_CONTROL: Bu modda, eklemler hedef pozisyona hareket eder"""

#force=500: Eklem motoruna uygulanan kuvvet miktarını belirtir (yüksek değerler, daha hızlı hareket için faydalıdır).

""""p.stepSimulation(): Simülasyon bir adım ileri gider. Bu, her döngüde simülasyonun ilerlemesini sağlar."""

#get_joint_limits(...): Belirli aralıklarla, her eklemin limitlerini almak için çağrılır (her 5 saniyede bir).

start_time = time.time() ####(6-)6.	Ana Döngü: Simülasyon
while True:
    p.setJointMotorControl2(bodyIndex=ur3_id, jointIndex=5, controlMode=p.POSITION_CONTROL, targetPosition=wrist_joint, force=500)
    p.setJointMotorControl2(bodyIndex=ur3_id, jointIndex=3, controlMode=p.POSITION_CONTROL, targetPosition=elbow_joint, force=500)
    p.setJointMotorControl2(bodyIndex=ur3_id, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=shoulder_joint, force=500)   
    p.setJointMotorControl2(bodyIndex=ur3_id, jointIndex=9, controlMode=p.POSITION_CONTROL, targetPosition=thump_joint, force=500)
    
    p.stepSimulation()

    if time.time() - start_time > 60:
        get_joint_limits(5)  # Wrist joint
        get_joint_limits(3)  # Elbow joint
        get_joint_limits(1)  # Shoulder joint
        get_joint_limits(9)  # Thump joint
        start_time = time.time()

    time.sleep(0.01) #time.sleep(0.01): Simülasyon döngüsünü yavaşlatmak için kullanılır, böylece simülasyon gerçek zamanlı olarak izlenebilir


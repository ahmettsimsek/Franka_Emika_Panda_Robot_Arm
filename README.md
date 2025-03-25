# Franka_Emika_Panda_Robot_Arm
# 🤖 Franka Emika Panda Robot Arm – Human Movement Imitation

This project demonstrates the imitation of human arm movements using the **Franka Emika Panda** robot arm in a simulated environment, leveraging **MediaPipe** for human pose detection and **PyBullet** for real-time robot control.

---

## 📌 Human Movement Detection

Using the **MediaPipe** library, joint landmarks from the human body were detected in real time through webcam input.  
Key features:

- Accurate coordinate extraction of joint points (shoulder, elbow, wrist).
- Real-time angle calculation between joints using vector-based mathematics.
- Smooth and continuous tracking of human movements.

---

## 🛠️ Simulation Environment and Interaction

A realistic simulation environment was created using **PyBullet**, including objects such as tables and boxes.  
The robot arm is able to:

- Interact with its environment in a physics-based simulation.
- Replicate human movements accurately and responsively.
- Adapt to obstacles and maintain fluid motion.

---

## 🧠 Robot Arm Imitation Capability

The **Franka Emika Panda** robot arm inside the PyBullet simulation mimics human arm motion using the angle values detected via MediaPipe.

- Joints involved: **Shoulder**, **Elbow**, and **Wrist**.
- The robot follows human arm movement in real time.
- Synchronization between human input and robot motion is successfully achieved.

---



## Detection of Human Movements: Using the MediaPipe library, 
joint points were successfully detected and the angle values ​​between these points were calculated in real time.
was calculated. The coordinates of the joint points are obtained accurately during the image processing process. 
Angles were determined using the vector-based angle calculation method. 

## Simulation Environment and Interaction: Robot with objects such as tables and boxes defined in the PyBullet environment 
The arm has successfully interacted. This shows that the robot arm can work in harmony with the environment. 
shows.Real-time simulation has been able to quickly imitate human movements. 


## Imitation Ability of Robot Arm: Franka Panda robot arm in PyBullet simulation environment, 
It was moved successfully according to the angle values ​​taken from MediaPipe. robot arm 
joints (shoulder, elbow, wrist) human movements

## 📦 Installation

```
pip install -r requirements.txt
```

```
pip install pybullet
```

```
tall colorama
```

```
pip install mediapipe
```

And

https://visualstudio.microsoft.com/visual-cpp-build-tools/


--------------------------------------------------------------------------------------------------------------

![Image](https://github.com/user-attachments/assets/af0aade0-7760-4829-9958-6b9d88e0e328)
![Image](https://github.com/user-attachments/assets/14da9f36-f5e2-42e4-b53f-e61e4f5cc400)
![Image](https://github.com/user-attachments/assets/3ff04522-9bfc-4d15-a00a-4dada104e8aa)
![Image](https://github.com/user-attachments/assets/d5a766ac-e7bc-4bef-b395-b45fa3724014)
![Image](https://github.com/user-attachments/assets/2d71a3df-3191-43cd-90bd-d7ae6aacbfb9)

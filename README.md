# 🚁 UAV Formation Control — ArUco Marker-Based Leader–Follower Tracking  
**TIH iHub Drishti | IIT Jodhpur | July 2025**  
---

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)  
![Project](https://img.shields.io/badge/Project-UAV%20Formation-orange) 
![ROS2](https://img.shields.io/badge/ROS%202-Foxy-blue) 
![OpenCV](https://img.shields.io/badge/OpenCV-4.7-green) 
![Python](https://img.shields.io/badge/Python-3.10-yellow)

---

## 🛰️ Overview
This project implements a **vision-based leader–follower UAV formation control system** using **ArUco markers** for relative pose estimation and a **vector-field control law** for stable tracking.  
It is designed for **GPS-denied environments**, enabling autonomous coordination between drones based solely on onboard camera and computer vision algorithms.

> 🧾 **Full Internship Report (PDF)** is available in `docs/` — see **Internship_Report** at the end of this README.
---

## 🧰 System Architecture

### ⚙️ Hardware Stack
| Component | Specification |
|-----------|---------------|
| **UAV Platform** | DJI Mavic 3 Enterprise (Leader + Follower) |
| **Onboard Compute** | NVIDIA Jetson Xavier NX (8GB) |
| **Vision Sensor** | 20MP RGB Camera (5472×3648 @30fps) with 3-axis Gimbal |
| **Fiducial Marker** | 15 cm ArUco (DICT_4X4_50) |
| **Communication** | MAVLink protocol via ROS 2 (900 MHz link) |

### 🧩 Software Stack
- **Middleware:** ROS 2 Foxy + MAVROS  
- **Computer Vision:** OpenCV 4.7 (ArUco detection, PnP solving)  
- **Control System:** Vector-field controller (C++/Python)  
- **Data Processing:** NumPy, Matplotlib, Pandas  

---

## ⚙️ Technical Workflow

### 1️⃣ Marker Detection & Pose Estimation

#### Step 1: ArUco Marker Detection
- Uses **OpenCV's ArUco module** (`cv2.aruco`) to detect fiducial markers in the RGB image.
- Each detected marker provides image-space corner coordinates `(u, v)` in pixels.

#### Step 2: Pose Estimation via Perspective-n-Point (PnP)
Given:
- **Known marker size** `s` (e.g., 0.15 m)
- **Camera intrinsic matrix**:
```
K = [[f_x, 0,   c_x],
     [0,   f_y, c_y],
     [0,   0,   1  ]]
```
- **3D coordinates** of marker corners in marker frame `P_marker`  
- **2D pixel coordinates** `p_i` from detection

We solve the PnP problem:
```
p_i ≈ K · [R | t] · P_marker
```
Where:
- `R` — rotation matrix (marker → camera)
- `t` — translation vector (marker origin in camera frame)

OpenCV implementation:
```python
retval, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, K, distCoeffs)
R, _ = cv2.Rodrigues(rvec)
```

---

### 2️⃣ Control Law

#### Step 1: Positional Error
```python
# Positional error
e_t = t_d - t_m
v_trans = k_p * e_t
```
Where:
- `t_d` = desired relative position  
- `t_m` = measured relative position  
- `k_p` = proportional gain (translation)

#### Step 2: Rotational Error
```python
# Rotational error
R_e = R_d.T @ R_m
r_err, _ = cv2.Rodrigues(R_e)
v_rot = k_o * r_err
```
Where:
- `R_d` = desired orientation  
- `R_m` = measured orientation  
- `k_o` = proportional gain (rotation)

---

### **Performance Metrics**

| Metric                     | Value       | Conditions                  |
| -------------------------- | ----------- | --------------------------- |
| **Positional Accuracy**    | < 20 cm     | Steady-state                |
| **Operating Range**        | 1.3–3.1 m   | Outdoor environment         |
| **Processing Latency**     | 38 ms/frame | Jetson Xavier NX            |
| **Detection Rate**         | 80%         | Varying lighting/background |
| **Convergence Time**       | < 2 sec     | From initial offset         |
| **Max Tracking Speed**     | 1.8 m/s     | Figure-eight trajectory     |
| **Position Hold Duration** | 12–15 min   | Full payload                |

---


---
## 🔍 Future Improvements
- Onboard real-time deployment on Jetson Xavier NX.  
- Integrating SLAM/VIO for temporary marker occlusion recovery.  
- Exploring AprilTags for higher detection range.  
- Multi-drone scalability testing.

---

### 📹 Field Testing Video
[📥 Download & Watch Test Video](https://drive.google.com/file/d/1JZ3NOyC9hoaikbANnyhYPBDoCg2FOD1a/view?usp=drive_link))

---
###Marker Detection.
![Multi-Marker Detection](figures/multi_marker_detection.png)  
<img width="500" height="500" alt="image_1" src="https://github.com/user-attachments/assets/ed1670d2-8169-4fe0-9314-d03efbb2c596" />
<img width="500" height="500" alt="Image_2" src="https://github.com/user-attachments/assets/013d22b7-7ab4-42cb-adda-2ef27ce7d4f5" />
<img width="500" height="500" alt="Indoor_testing" src="https://github.com/user-attachments/assets/0cbbbec5-309d-445e-8c08-329569818fab" />

---

---

## 🧾 Final Report
[📥 Download Final_Report (PDF)](https://drive.google.com/file/d/1RP-vB7r8DxE9ECV-YNQd_9_YhrM64mn-/view?usp=drive_link)

---


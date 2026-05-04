# LiDAR-Based Bicycle Rear-Collision Prediction and Warning System-Introduction

## Problem Addressed
Bicycle riders are highly vulnerable on the road, especially when facing nearby front obstacles or fast-approaching vehicles from behind.  
This project is designed to improve riding safety by detecting these risks and providing early warnings in real time.

---

## Project Description
This project is a real-time bicycle safety system built to reduce collision risk and improve rider awareness.  
It uses **RP-LiDAR A1** to detect front obstacles and **TF02-Pro** to monitor rear approaching objects.  
Warnings are delivered through a **buzzer**, **LED**, and an **ST7735 TFT display** displays timely visual feedback.

The idea came from my **personal experience in a serious bicycle accident**, which led me to explore whether an embedded system could help riders notice danger earlier and react sooner.

---

## Hardware

### Main Controller
- **Arduino Mega 2560** microcontroller

### Sensors
- **RP-LiDAR A1** LiDAR
- **TF02-Pro** LiDAR

(LiDAR refer to as Light Detection and Ranging)

### Output Devices
- **ST7735 TFT display**
- **Buzzer**
- **LED**

The Arduino Mega 2560 serves as the system controller, handling sensor input, warning logic, and output control.

---

## Software

### Development Platform
- **Arduino IDE**

### Programming Language
- **C++** [view source code](./source_code.cpp)

The program is written in C++ for sensor communication, data processing, warning control, and display output.

---

## Warning Functions

### Front Warning Function
The front warning function uses RP-LiDAR A1 to scan the 180° area in front of the bicycle.  
A predefined warning zone is used to judge whether an object is dangerously close.

When objects are repeatedly detected inside this zone within a short time window, the system treats the situation as a potential hazard and activates the buzzer.  
This design helps reduce false alarms while maintaining real-time response.

### Rear Warning Function
The rear warning function uses TF02-Pro to continuously measure the distance of objects behind the bicycle.  
By comparing consecutive measurements, the system estimates relative speed and calculates the time to collision (TTC).

If the predicted TTC is less than or equal to **2.5 seconds**, the system activates the LED warning.  
This means the rear warning is based on prediction, not only on distance.

### Warning Mechanism
The system uses two different warning strategies:
- **Buzzer:** front obstacle warning
- **LED:** rear collision prediction warning

This design allows the system to respond differently to front and rear risks in real riding situations.

---

## Repository Structure

```text
README
demo_photos
media/

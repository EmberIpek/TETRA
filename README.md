# 10/12/2025
# Ember Ipek – TETRA team

## 10/12/2025:
Implemented facial recognition with OpenCV Haar Cascades using Logitech C270 camera connected to RPi as proof of concept. RPi kept disconnecting while trying to install OpenCV. Board gets very hot and runs out of memory. To fix, increase swap size before building:


```bash
sudo nano /etc/dphys-swapfile
# Change CONF_SWAPSIZE=2048
sudo dphys-swapfile setup
```


### Running headless: 
Open xLauncher multiple windows, access control disabled.  
Make sure Xming is running before starting SSH session.  


On powershell:  
```powershell
PS C:\Users\ember> $env:DISPLAY="localhost:0.0"
PS C:\Users\ember> ssh -Y rpi@raspberrrypi.local
```


On Raspberry Pi:  
```bash
rpi@raspberrrypi:~ $ echo $DISPLAY
```


Very low FPS on VNC, reduce image resolution for better FPS:  
```python
LogiC270.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
LogiC270.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
```


Next steps: Canny edge detection thresholds are questionable, play with different values. Apply gaussian blur to reduce noise for edge detection. Implement feature detection using SIFT. Yolo instead of cv2 for face recognition. Use QR codes for preliminary “curb detection”.

---

## **11/2/2025:** 
Testing motor PWM and implementing obstacle avoidance using HC-SR04 ultrasonic sensor. Motor is surprisingly strong for size and voltage, recommend exercising caution during operation to avoid bloody fingers. Implement motor speed up when cat detected (stand-in for curb detection). Can control multiple motors with one driver using relay or MOSFET. Will use MOSFET for now. Weigh pros/cons.
Obstacle avoidance implemented using two motors and proximity sensor modulates PWM signal. Timing issues cause motor 0 to either stall when time.sleep() is too low (checkdist does not receive echo signal?), or spin faster than motor 1 when time.sleep() is too high. 

Next steps: Resolve echo timeout issue. Convert messy whiteboard calculations and diagrams to digital. Use buttons as stand in for orientation detection before using IMU. Eventually test 120-degree RPi cam for object recognition. Test on TETRA motor with 14V external battery, voltage regulator, and driver. **IMPORTANT: USE MULTIMETER TO VERIFY ALL VOLTAGES AND CURRENTS WITHIN SAFE OPERATIONAL THRESHOLDS BEFORE CONNECTING TO RPI.**

Possible issues: Even with orientation detection, must be able to determine direction to drive. Obstacle avoidance vs curb detection conflict resolution.

---

## **11/6/2025:**
Reduced echo timing delays by returning maxValue after a set timeout period to allow execution to continue. Implemented button control for 3 motors with latching logic. PWM logic is functional with motors responding to button control signals. 12V motor schematic created, voltage regulator used for 12V output from 14.8V battery and verified with multimeter. Motor driver used with IRLB8721 n-channel MOSFET to control PWM logic. Wiring schematic:

[![CircuitLab Schematic d4trm5m7588w](https://www.circuitlab.com/circuit/d4trm5m7588w/screenshot/540x405/)](https://www.circuitlab.com/circuit/d4trm5m7588w/tetra-motor-control/)

Video:

[![Watch the video](https://img.youtube.com/vi/uYNMLLLu1mc/default.jpg)](https://youtu.be/uYNMLLLu1mc)

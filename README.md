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

---

## **11/29/2025:**
Chassis prototype ready, 12V motors acquired. Reworked PWM code for use with one driver and multiple motors with MOSFETs as switches. Wired according to diagram and code tested with no results. Testing battery voltage output on one end of wire with multimeter gives 15V while other end of same wire is 2V. Possible causes: faulty wires, witchcraft. Replacing wires resolved issue. Having trouble with MOSFET, switching to relay module for motor control; will consult further with electrical team to determine solution. 12V motors successfully controlled using relay module and buttons, flipping algorithm defined and will be implemented. Motor 1 is always on, always rotates clockwise; other motors always rotate counterclockwise and determined by the face TETRA is driving on.

MPU implemented, to turn on I2C and verify:

```bash
sudo raspi-config
# Interface Options -> I2C
sudo i2cdetect -y 1
```

The slave address of the MPU-60X0 is b110100X which is 7 bits long. The LSB bit of the 7 bit address is determined by the logic level on pin AD0. AD0 pulled to ground for address 0x68.
MPU6050 registers: [https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf]

Register 0x6B: power management, clear all bits to wake up MPU.

Gyroscope output registers read 2 bytes at a time. 0x43: Gyroscope X, 0x45: Gyroscope Y, 0x47: Gyroscope Z.

Next steps: Interpret gyroscope values and range of values each face corresponds to, e.g. driving on face A vs after flipping onto face B. Sleep before any more debugging.

---

## **2/7/26:**
Set up new Raspberry Pi 5, configured static IP: 192.168.0.2 for ethernet adapter, run 

```bash
sudo nano /boot/cmdline.txt
```

add “ip=192.168.0.4::192.168.0.1:255.255.255.0:rpi:eth0:off” to end of file. RPi and Ethernet IP must be on same subnet as wifi.

Set up wifi for Pi 5. To set up wifi, run

```bash
nmcli dev wifi list
sudo nmcli dev wifi connect "SSID" password "PASSWORD"
```

Physical pull up resistor used for gyroscope SCL line. Added GPIO cleanup and reduced motor speed to begin field testing. Cannot connect to wifi when powered with 5V regulator from battery, attempting to use capacitors to compensate for current spikes. Wifi connection obtained and TETRA car is officially untethered and ready for testing. 5V Wire on regulator smoked and melted due to swapped 15V and GND inputs, electronics were spared. Will place 2-5A fuse from battery to regulator for future electrical adventures. Need to process electrical trauma for the remainder of night.

###**2/7/26 LESSONS LEARNED**

The wires from the battery had disconnected and I just reconnected it without thinking. I didn't use red and black like I usually do because I ran out of those colors and thought “I'll just remember or check” but used orange for negative, which I mistook for red while in a hurry. I'm usually adamant about ALWAYS using red and black for power and ground, but now I truly understand it's not just a convenience thing. I have more respect for electricity now, and now that I'm looking at my hand and see a small burn on there... I'm scared. I EVEN HAD A LITTLE FLAG THAT I MADE WITH TAPE AND WIRE THAT SAID "CAUTION 15V RAIL" AND I IGNORED IT. Double checking with multimeter is NOT optional.




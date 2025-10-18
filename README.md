10/12/2025
Ember Ipek – TETRA team
--
10/12/2025: Implemented facial recognition with OpenCV using Logitech C270 camera connected to RPi as proof of concept. RPi kept disconnecting while trying to install OpenCV. Board gets very hot and runs out of memory. To fix, increase swap size before building:


```bash
sudo nano /etc/dphys-swapfile
# Change CONF_SWAPSIZE=2048
sudo dphys-swapfile setup
```


##Running headless: 
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


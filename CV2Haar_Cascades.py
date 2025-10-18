# TETRA team 10/13/2025
# Video facial recognition using Haar Cascades. Proof of concept.

import cv2
import time

t1=time.time()
#cv2.waitKey(0)
face_cascade = cv2.CascadeClassifier('/home/rpi/Projects/TETRA/haarcascade_frontalface_default.xml')

FPS=1.0

LogiC270 = cv2.VideoCapture(0)
# shrink video size -> better fps
LogiC270.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
LogiC270.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

try:
    while True:
        t2=time.time()
        
        success, C270img = LogiC270.read()
        img_gray = cv2.cvtColor(C270img, cv2.COLOR_BGR2GRAY)
        img_edges = cv2.Canny(C270img, 50, 200)
        
        face_found = face_cascade.detectMultiScale(img_gray, minSize=(20,20))
        for(x, y, w, h) in face_found:
            cv2.rectangle(C270img, (x, y), (x+w, y+h), (0,255,0), 5)
            cv2.rectangle(img_edges, (x, y), (x+w, y+h), (0,255,0), 5)
        
        FPSstr=f"{FPS:3.1f} fps"
        cv2.putText(C270img,FPSstr,(10,30),cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2)
        cv2.imshow("CAPTURE",C270img)
        cv2.imshow("EDGES", img_edges)
        
        print(f"Frame Rate = {FPS:4.1f} fps",22*'\b',end='',flush=True)
        cv2.waitKey(1)
        t3=time.time()
        FPS=1.0/(t3-t2)
except KeyboardInterrupt:
    print("\n\nCapture is done!!!")

cv2.destroyAllWindows()


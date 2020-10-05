import cv2 as cv
import numpy as np
import RPi.GPIO as IO 

width = 320
height = 240

x,y = -1,-1

def nothing(x):
    pass

def forward():
    print("FORWARd")
    IO.output(19,1)
    IO.output(26,0)
    IO.output(16,1)
    IO.output(20,0)
    
def reverse():
    print("BACK")
    IO.output(19,0)
    IO.output(26,1)
    IO.output(16,0)
    IO.output(20,1)
    
def left():
    print("left")
    IO.output(19,0)
    IO.output(26,1)
    IO.output(16,1)
    IO.output(20,0)
    
def right():
    print("right")
    IO.output(19,1)
    IO.output(26,0)
    IO.output(16,0)
    IO.output(20,1)
    
def stop():
    print("STOP")
    IO.output(19,0)
    IO.output(26,0)
    IO.output(16,0)
    IO.output(20,0)

IO.setwarnings(False) 
IO.setmode(IO.BCM) 
IO.setup(26, IO.OUT)
IO.setup(19, IO.OUT)
IO.setup(20, IO.OUT)
IO.setup(16, IO.OUT)

#cv.namedWindow('Trackbar', cv.WINDOW_AUTOSIZE)
#cv.createTrackbar('h', 'Trackbar', 0, 255, nothing)
#cv.createTrackbar('ls', 'Trackbar', 0, 255, nothing)
#cv.createTrackbar('hs', 'Trackbar', 255, 255, nothing)
#cv.createTrackbar('lv', 'Trackbar', 0, 255, nothing)
#cv.createTrackbar('hv', 'Trackbar', 255, 255, nothing)

cap = cv.VideoCapture(0)
cap.set(3, width)
cap.set(4, height)

while (1):
    #v = cv.getTrackbarPos('h', 'Trackbar')
    #a = cv.getTrackbarPos('ls', 'Trackbar')
   # s = cv.getTrackbarPos('hs', 'Trackbar')
  #  d = cv.getTrackbarPos('lv', 'Trackbar')
 #   f = cv.getTrackbarPos('hv', 'Trackbar')

    l = np.array([177 - 10, 80, 80])
    h = np.array([177 + 10, 255, 255])
    #l = np.array([v - 10, a, d])
    #h = np.array([v + 10, s, f])
    kernal = np.ones((2, 2), np.uint8)

    ret, frame = cap.read()
    
    img = frame.copy()
    cv.line(img, (int(width/3), 0), (int(width/3), height), (255, 0, 0), 2)
    cv.line(img, (int(width*2/3), 0), (int(width*2/3), height), (255, 0, 0), 2)
    cv.line(img, (0, int(height/3)), (width, int(height/3)), (255, 0, 0), 2)
    cv.line(img, (0, int(height*2/3)), (width, int(height*2/3)), (255, 0, 0), 2)

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, l, h)
    blur = cv.blur(mask,(3,3))
    erode = cv.erode(mask, kernal, iterations=5)
    dilate = cv.dilate(erode, kernal, iterations=5)

    con, heireracy = cv.findContours(dilate, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

    for c in con:
        a, b, w, h = cv.boundingRect(c)
        if (w * h) > 200:
            if(w*h)>25000:
                reverse()
            try:
                x = int(a + (w / 2))
                y = int(b + (h / 2))
                cv.circle(img, (x, y), 5, (255, 0, 0), 1)
            except:
                pass
    
    print(x,',',y,',',w*h)
    
    if(x>0 and y>0 and x<int(width/3) and y<int(height/3) and len(con)>0):
        left()
    elif(x>0 and y>=int(height/3) and x<int(width/3) and y<int(height*2/3) and len(con)>0):
        left()
    elif(x>0 and y>=int(height*2/3) and x<int(width/3) and y<=height and len(con)>0):
        left()
            
    elif(x>=int(width/3) and y>0 and x<int(width*2/3) and y<int(height/3) and len(con)>0):
        forward()
    elif(x>=int(width/3) and y>int(height/3) and x<int(width*2/3) and y<int(height*2/3) and len(con)>0):
        stop()
    elif(x>=int(width/3) and y>int(height*2/3) and x<int(width*2/3) and y<height and len(con)>0):
        reverse()
            
    elif(x>=int(width*2/3) and y>0 and x<=width and y<int(height/3) and len(con)>0 and len(con)>0):
        right()
    elif(x>=int(width*2/3) and y>int(height/3) and x<=width and y<int(height*2/3) and len(con)>0):
        right()
    elif(x>=int(width*2/3) and y>int(height*2/3) and x<=width and y<height and len(con)>0):
        right()
    else:
        stop()
    
    
    try:
        #cv.imshow('erode', erode)
        #cv.imshow('dilate',dilate)
        cv.imshow('mask',mask)
        cv.imshow('img', img)
    except:
        pass

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cv.destroyAllWindows()
cap.release()

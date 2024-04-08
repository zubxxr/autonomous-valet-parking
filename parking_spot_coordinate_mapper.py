import cv2
import pandas as pd
import numpy as np
from ultralytics import YOLO
import time
import torch

model=YOLO('weights/rosmaster_yolov8.pt')

def RGB(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE :  
        colorsBGR = [x, y]
        print(colorsBGR)
        

cv2.namedWindow('RGB')
cv2.setMouseCallback('RGB', RGB)

my_file = open("coco.txt", "r")
data = my_file.read()
class_list = data.split("\n")

# = tl, tr, br bl
area1=[(310,188),(438,192),(434,297),(290,311)]

area2=[(442,190),(571,193),(575,315),(434,296)]

area3=[(571,191),(700,194),(715,315),(577, 315)]

area4=[(703,195),(817,196),(845,311),(714,315)]

def display_priority_queue():
    cap=cv2.VideoCapture("videos/2024-03-05-134820.webm")

    while True:    
        ret,frame = cap.read()
        if not ret:
            break
        frame=cv2.resize(frame,(1020,500))

        results=model.predict(frame)
        a = results[0].boxes.data.cpu().numpy() 
        px=pd.DataFrame(a).astype("float")

        list1=[]
        list2=[]
        list3=[]
        list4=[]

        for index,row in px.iterrows():
            x1=int(row[0])
            y1=int(row[1])
            x2=int(row[2])
            y2=int(row[3])
            d=int(row[5])
            c=class_list[d]
            if 'Rosmasters' in c:
                cx=int(x1+x2)//2
                cy=int(y1+y2)//2

                results1=cv2.pointPolygonTest(np.array(area1,np.int32),((cx,cy)),False)
                if results1>=0:
                    list1.append(c)

                results2=cv2.pointPolygonTest(np.array(area2,np.int32),((cx,cy)),False)
                if results2>=0:
                    list2.append(c)

                results3=cv2.pointPolygonTest(np.array(area3,np.int32),((cx,cy)),False)
                if results3>=0:
                    list3.append(c)  

                results4=cv2.pointPolygonTest(np.array(area4,np.int32),((cx,cy)),False)
                if results4>=0:
                    list4.append(c)  

                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.circle(frame,(cx,cy),3,(0,0,255),-1)
                cv2.putText(frame,str(c),(x1,y1),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,255),1)

        a1=(len(list1))
        a2=(len(list2))       
        a3=(len(list3))    
        a4=(len(list4))

        o=(a1+a2+a3+a4)
        space=(4-o)

        # Create a list where any a's that are not 1 will be inside the list. It will be based on numerical order.

        # if a = 1, parking spot is occupied; else, parking spot is free
        if a1==1:
            cv2.polylines(frame,[np.array(area1,np.int32)],True,(0,0,255),2)

            #set text red for occupied
            cv2.putText(frame,str('1'),(50,441),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),1)
        else:
            cv2.polylines(frame,[np.array(area1,np.int32)],True,(0,255,0),2)
            
            #set text white for empty
            cv2.putText(frame,str('1'),(50,441),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,255),1)
        if a2==1:
            cv2.polylines(frame,[np.array(area2,np.int32)],True,(0,0,255),2)
            cv2.putText(frame,str('2'),(106,440),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),1)
        else:
            cv2.polylines(frame,[np.array(area2,np.int32)],True,(0,255,0),2)
            cv2.putText(frame,str('2'),(106,440),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,255),1)
        if a3==1:
            cv2.polylines(frame,[np.array(area3,np.int32)],True,(0,0,255),2)
            cv2.putText(frame,str('3'),(175,436),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),1)
        else:
            cv2.polylines(frame,[np.array(area3,np.int32)],True,(0,255,0),2)
            cv2.putText(frame,str('3'),(175,436),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,255),1)
        if a4==1:
            cv2.polylines(frame,[np.array(area4,np.int32)],True,(0,0,255),2)
            cv2.putText(frame,str('4'),(250,436),cv2.FONT_HERSHEY_COMPLEX,0.5,(0,0,255),1)
        else:
            cv2.polylines(frame,[np.array(area4,np.int32)],True,(0,255,0),2)
            cv2.putText(frame,str('4'),(250,436),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,255,255),1)

        # Store the values of a1, a2, a3, and a4 in a dictionary
        priority_dict = {'a1': a1, 'a2': a2, 'a3': a3, 'a4': a4}

        # Sort the dictionary based on the numerical order of the keys
        sorted_priority = sorted(priority_dict.items(), key=lambda x: int(x[0][1:]))

        # Iterate over the sorted dictionary and add the elements to the priority queue
        priority_queue = []
        for key, value in sorted_priority:
            if value != 1:
                priority_queue.append(key)

        print("Priority Queue based on numerical order:")
        for item in priority_queue:
            print(item)

        # Empty Parking Spot Counter
        cv2.putText(frame,str(space),(23,30),cv2.FONT_HERSHEY_PLAIN,3,(255,255,255),2)

        cv2.imshow("RGB", frame)

        if cv2.waitKey(0) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    display_priority_queue()

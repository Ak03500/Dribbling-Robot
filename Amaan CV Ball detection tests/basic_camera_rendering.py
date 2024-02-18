import cv2
from matplotlib import pyplot as plt

#connect to camera
cap = cv2.VideoCapture(0)
#get frame
ret,frame = cap.read()

print(ret)

print(frame)

plt.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
plt.show()

#real-time video
cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()

    #show image
    cv2.imshow('Webcam', frame)

    #button press to stop video
    if (cv2.waitKey(1) & 0xFF == ord('q')):
        break

#release webcam
cap.release()

cv2.destroyAllWindows()
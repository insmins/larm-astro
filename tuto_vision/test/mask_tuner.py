import cv2
import numpy as np

def souris(event, x, y, flags, param):
    global lo, hi, color, hsv_px

    if event == cv2.EVENT_MOUSEMOVE:
        # Conversion des trois couleurs RGB sous la souris en HSV
        px = frame[y,x]
        px_array = np.uint8([[px]])
        hsv_px = cv2.cvtColor(px_array,cv2.COLOR_BGR2HSV)

    if event==cv2.EVENT_MBUTTONDBLCLK:
        color=image[y, x][0]

    if event==cv2.EVENT_LBUTTONDOWN:
        if color>5:
            color-=1

    if event==cv2.EVENT_RBUTTONDOWN:
        if color<250:
            color+=1

    # lo[1]=color-10
    # hi[1]=color+10

color=100

lo=np.array([58-20, 50, 20])
hi=np.array([58+20, 255,255])

color_info=(0, 0, 255)

cap=cv2.VideoCapture(0)
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', souris)
hsv_px = [0,0,0]

# Creating morphological kernel
erodeKernel = np.ones((3, 3), np.uint8)
dilateKernel = np.ones((3, 3), np.uint8)

while True:
    ret, frame=cap.read()
    base_img = frame.copy()
    image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = image

    # Seuillage par colorimétrie
    mask=cv2.inRange(mask, lo, hi)
    beforeErosion = mask

    # Réduction du bruit
    mask=cv2.erode(mask, erodeKernel, iterations=4)
    mask=cv2.dilate(mask, dilateKernel, iterations=4)

    # Application du masque
    image2=cv2.bitwise_and(frame, frame, mask= mask)


    cv2.putText(frame, "Couleur: {:d}".format(color), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

    # Affichage des composantes HSV sous la souris sur l'image
    pixel_hsv = " ".join(str(values) for values in hsv_px)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260),
               font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    
    # Segmentation
    ## Min enclosing circle method
    elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(elements) > 0:
        c=max(elements, key=cv2.contourArea)
        ((x, y), rayon)=cv2.minEnclosingCircle(c)
        if rayon>30:
            cv2.circle(frame, (int(x), int(y)), int(rayon), color_info, 2)
            cv2.circle(frame, (int(x), int(y)), 5, color_info, 10)
            cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), color_info, 2)
            cv2.putText(frame, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)




    cv2.imshow('Camera', frame)
    cv2.imshow('Before erosion', beforeErosion)
    # cv2.imshow('image2', image2)
    cv2.imshow('Mask', mask)

    if cv2.waitKey(1)&0xFF==ord('q'):
        break
    cv2.imwrite('mask.jpg', mask)
    cv2.imwrite('rgb.jpg', base_img)
cap.release()
cv2.destroyAllWindows()
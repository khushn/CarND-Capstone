import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2

testDirectory='carla_images/red/'
kernel=np.ones((15,15),np.float32)/255
"""
ower_filter1=np.array([220,200,0])
upper_filter1=np.array([255,255,255])
lower_filter2=np.array([0,0,0])
upper_filter2=np.array([0,255,255])
"""
def nothing(x):
    pass

def ModImg(img):
	#resize 800x600 => 400x300
	small=cv2.resize(img,(400,300), 0, 0, cv2.INTER_LINEAR)	
	#hsv red+orange filter
	hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
	mask1 = cv2.inRange(hsv,lower_filter1, upper_filter1)
	mask2 = cv2.inRange(hsv,lower_filter2, upper_filter2)
	mask = 	cv2.add(mask1,mask2)
	img2 = cv2.bitwise_and(small,small,mask = mask)
	return mask,img2 ,small

#create the sliding window
wnd = 'HSV Filter'
cv2.namedWindow(wnd)
im='image_nbr'
bl='kernel 2n+1'
hh='Hue High'
hl='Hue Low'
sh='Saturation High'
sl='Saturation Low'
vh='Value High'
vl='Value Low'

cv2.createTrackbar(im, wnd,0,len(os.listdir(testDirectory)),nothing)
cv2.createTrackbar(bl, wnd,0,7,nothing)
cv2.createTrackbar(hl, wnd,20,179,nothing)
cv2.createTrackbar(hh, wnd,35,179,nothing)
cv2.createTrackbar(sl, wnd,65,255,nothing)
cv2.createTrackbar(sh, wnd,130,255,nothing)
cv2.createTrackbar(vl, wnd,250,255,nothing)
cv2.createTrackbar(vh, wnd,255,255,nothing)

while(1):  
    #read trackbar positions for each trackbar
    imv=cv2.getTrackbarPos(im, wnd)  
    blv=cv2.getTrackbarPos(bl, wnd)  
    hul=cv2.getTrackbarPos(hl, wnd)
    huh=cv2.getTrackbarPos(hh, wnd)
    sal=cv2.getTrackbarPos(sl, wnd)
    sah=cv2.getTrackbarPos(sh, wnd)
    val=cv2.getTrackbarPos(vl, wnd)
    vah=cv2.getTrackbarPos(vh, wnd)
    #
    imgName= testDirectory + os.listdir(testDirectory)[imv]
    img= cv2.imread(imgName)
    #img=cv2.fGaussianBlur(img,(15,15),0)
    # resize image
    small_img=cv2.resize(img,(800,600), 0, 0, cv2.INTER_LINEAR)
    crop_img=small_img[170:260,:]
    #bluring the imaqge
    blur_img=cv2.medianBlur(crop_img,2*blv+1)
    #convert from a BGR stream to an HSV stream
    hsv=cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)    
    #make array for final values
    HSVLOW=np.array([hul,sal,val])
    HSVHIGH=np.array([huh,sah,vah])
    #create a mask for that range
    mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
    res = cv2.bitwise_and(blur_img,blur_img, mask =mask)
    #res=cv2.medianBlur(res,2*blv+1)
    cv2.imshow(wnd, res)
    cv2.imshow("original",small_img)
    print(mask.sum(axis=None))    
    k = cv2.waitKey(20000) #&amp;&amp; 0xFF    
    if k == ord('q'):
        break
    #print(imv)

cv2.destroyAllWindows()



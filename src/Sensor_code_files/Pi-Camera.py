import cv2
#print(cv2.__version__)

cv2.namedWindow("preview")
vc = cv2.VideoCapture(-1)

if vc.isOpened():
    rval, frame = vc.read()
else:
    rval = False

while rval:
    cv2.imShow("preview",frame)
    rval, frame = vc.read()
    key = cv2.waitKey(20)
    if key == 27: #exit on esc
        break

cv2.destroyWindow("preview")
vc.release
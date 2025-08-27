import numpy as np
import cv2

def cv_display(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((600, 800, 4))
    i3 = i2[:,:,:3]
    cv2.imshow("", i3)
    cv2.waitKey(1)

    return i3/255.0

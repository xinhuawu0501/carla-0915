import numpy as np
import cv2
import carla

def process_rgb_img(image, img_x=800, img_y=600):
    try:
        i = np.array(image.raw_data)
        i2 = i.reshape((img_y, img_x, 4))
        i3 = i2[:,:,:3]
        return i3
    except Exception as e:
        print(e)
        return np.zeros((img_y, img_x, 3), dtype=np.uint8)

def process_semantic_img(image):
    try:
        image.convert(carla.ColorConverter.CityScapesPalette)
        img_array = np.frombuffer(image.raw_data, dtype=np.uint8)
        img_array = img_array.reshape((image.height, image.width, 4))[:, :, :3]

        return img_array
    except Exception as e:
        print(e)
        return np.zeros((600, 800, 3), dtype=np.uint8) #output black image



def cv_display(img_arr):
    cv2.imshow("", img_arr)
    cv2.waitKey(1)



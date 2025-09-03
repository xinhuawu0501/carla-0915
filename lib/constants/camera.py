import numpy as np

IMG_Y = 600
IMG_X = 800

CITYSCAPES_PALETTE = {
    0: np.array([0, 0, 0]),         # Unlabeled
    1: np.array([70, 70, 70]),      # Buildings
    4: np.array([220, 20, 60]),     # Pedestrians
    10: np.array([0, 0, 142]),      # Vehicles
    12: np.array([128, 64, 128]),   # Road
    13: np.array([244, 35, 232]),   # Sidewalk
    14: np.array([220, 220, 0]),    # Traffic Signs
}

CITYSCAPES_LABEL = {
    0: 'unlabeled',         # Unlabeled
    1: 'buildings',      # Buildings
    4: 'pedestrian',     # Pedestrians
    10: 'vehicle',      # Vehicles
    12: 'road',   # Road
    13: 'sidewalk',   # Sidewalk
    14: 'traffic sign',    # Traffic Signs
}
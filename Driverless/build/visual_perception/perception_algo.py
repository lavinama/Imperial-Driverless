from final_YOLO import YOLO_N
from Classifier.cone_classifier import Classifier
from Classifier.utils import crop_cones, get_stereoConePixelCoordinate, check_MatchingCones
from Stereo.cone_locations import Stereo
import torchvision.transforms.functional

import numpy as np
import torch

class PerceptionAlgo():

    def __init__(self, B=2, N=128, M=128, fov=45) -> None:
        """
        Parameters
        ----------
        B : float, optional
            Distance between cameras (in metres). The default is ?.
        N,M : int, optional
            Number of pixels in image (in number of pixels). The default are ? x ?.
        fov : float, optional
            Field of view (in radians). The default is ?.
        """
        self.yolo = YOLO_N()
        self.classifier = Classifier()
        self.stereo = Stereo(B, N, M, fov)
        

    def detect_cones(self, img_left: np.ndarray, img_right: np.ndarray):
        """
        Detect the cones and return their coordinates
        ----------
        Steps:
        1. Detect cones
        2. Crop images of cones?
        3. Calculate array of coordinates of cones in left and right cones?
        4. Calculate coordinates of cone
        ----------
        Parameters
        ----------
        img_left : numpy array
            left camera image
        img_right : numpy array
            right camera image

        Returns
        -------
        [X, Y, Z, class]: np.array
            Coordinates of cone and their class
            The classes are:
                0: left
                1: right
                2: orange (small)
                3: orange (large)
        """
        detections_left, imageInfo_left = self.yolo.detectFrame(img_left)
        detections_right, imageInfo_right = self.yolo.detectFrame(img_right)
        pixelCoord_left, pixelCoord_right = get_stereoConePixelCoordinate(detections_left, detections_right, imageInfo_left, imageInfo_right)
        stereo_pixelCoord, left_args = check_MatchingCones(pixelCoord_left, pixelCoord_right)

        detections_left = detections_left[left_args]

        img_left = torchvision.transforms.functional.to_pil_image(img_left)

        cropped_cones = crop_cones(img_left, detections_left, imageInfo_left)
        cone_classes = np.zeros(detections_left.shape[0])
        for (i, cone) in enumerate(cropped_cones):
            cone_classes[i] = self.classifier.classify_cone_bins(cone)
        cones_3dCoord = self.stereo.get_coordinates(stereo_pixelCoord)
        return np.hstack((cones_3dCoord, np.expand_dims(cone_classes, axis=1)))
        
        
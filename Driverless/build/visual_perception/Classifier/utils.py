import numpy as np
import torch
from PIL import Image
from sklearn.neighbors import KDTree

def crop_cones(original_img: Image.Image, detections: torch.Tensor, imageInfo: np.ndarray):
    ratio, pad_h, pad_w = imageInfo
    for i in range(len(detections)):
        x0 = detections[i, 0].to('cpu').item() / ratio - pad_w
        y0 = detections[i, 1].to('cpu').item() / ratio - pad_h
        x1 = detections[i, 2].to('cpu').item() / ratio - pad_w
        y1 = detections[i, 3].to('cpu').item() / ratio - pad_h 
        img = original_img.crop((x0, y0, x1, y1))
        yield img

def get_conePixelCoordinate(detections: torch.Tensor, imageInfo: np.ndarray):
    ratio, _, _ = imageInfo
    coordinates = np.zeros((detections.shape[0], 4))
    detections /= ratio
    coordinates[:,0] = (detections[:, 2] + detections[:, 0]) /2
    coordinates[:,1] = (detections[:, 3] + detections[:, 1]) /2
    coordinates[:,2] = detections[:, 2] - detections[:, 0]
    coordinates[:,3] = detections[:, 3] - detections[:, 1]
    return coordinates

def check_MatchingCones(left_cones, right_cones):
    """
        Match the detected cones from the right and left camera
        ----------

        Parameters
        ----------
        coordinates : numpy array
            pixel coordinates for cones in the left (first two columns) and
            right (second two columns) camera images

        Returns
        -------
        stereo_pixelCoord: numpy array
            picel coordinates of the same cone inin the left (first two columns) and
            right (second two columns) camera images
        args_left: numpy array
            the indeces of matched cones in left
        args_right: numpy array
            the indeces of matched cones in the right
        """
    kdt = KDTree(left_cones, leaf_size=30, metric='euclidean')
    distances, nn = kdt.query(right_cones, k=1, return_distance=True)

    filter_distance = np.squeeze(distances<15)
    nn = nn[filter_distance,0]
    distances = distances[filter_distance]

    stereo_pixelCoord = np.zeros((nn.shape[0], 4))
    print(nn.shape)
    print(filter_distance.shape)
    stereo_pixelCoord[:,:2] = left_cones[nn,:2]
    stereo_pixelCoord[:,2:] = right_cones[filter_distance,:2]

    return stereo_pixelCoord, nn
    

def get_stereoConePixelCoordinate(
        detections1: torch.Tensor,
        detections2: torch.Tensor,
        imageInfo1: np.ndarray,
        imageInfo2: np.ndarray
    ):
    # detections1, detections2 = check_MatchingCones(detections1, detections2)
    coordinates_left = get_conePixelCoordinate(detections1, imageInfo1)
    print("right: \n", coordinates_left)
    coordinates_rigth = get_conePixelCoordinate(detections2, imageInfo2)
    print("\n\nright: \n", coordinates_rigth)
    return (coordinates_left, coordinates_rigth)
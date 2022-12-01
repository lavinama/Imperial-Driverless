# from CVC_YOLOv3.detect import Detector
# from CVC_YOLOv3.models import Darknet
import numpy as np
from final_YOLO import YOLO_N
from PIL import Image
import torchvision.transforms.functional
from Classifier.cone_classifier import Classifier
from Classifier.utils import crop_cones, get_stereoConePixelCoordinate, check_MatchingCones
import torch
from perception_algo import PerceptionAlgo

# print(torch.__version__)

cone_classes = ['left', 'right', 'orange small', 'orange large']
# img_loc = "dataset/sample_image.jpg"
img_loc = "dataset/1_cam1.png"
img = Image.open(img_loc)
img = torchvision.transforms.functional.to_tensor(img)

"""
# yolo = Detector(
#     target_path="CVC_YOLOv3/dataset/YOLO_Dataset/"+img,
#     output_path="CVC_YOLOv3/outputs/visualization/",
#     weights_path="CVC_YOLOv3/yolo_weights/pretrained_yolo.weights",
#     model_cfg="CVC_YOLOv3/model_cfg/yolo_baseline.cfg",
#     conf_thres=0.8,
#     nms_thres=0.25,
#     xy_loss=2,
#     wh_loss=1.6,
#     no_object_loss=25,
#     object_loss=0.1,
#     vanilla_anchor=False
# )
"""

print(type(img))
# print(img)
# print(img[0])

yolo = YOLO_N()
classifier = Classifier()

ret, img_info = yolo.detectFrame(img)
# print(ret)
img = torchvision.transforms.functional.to_pil_image(img)
cropped_cones = crop_cones(img, ret, img_info)
for cone in cropped_cones:
    cone.show()
    cone_class = classifier.classify_cone_bins(cone)
    print(cone_classes[cone_class])
    input("press enter")
    print(cone_class)


# img_loc = "1_cam1.png"
# img = Image.open(img_loc)
# img2 = torchvision.transforms.functional.to_tensor(img)
# img_loc = "1_cam2.png"
# img = Image.open(img_loc)
# img1 = torchvision.transforms.functional.to_tensor(img)

# # img1 = img1.numpy()
# # img2 = img2.numpy()

# algo = PerceptionAlgo(B=0.2, N=785, M=785, fov=90)
# algo.detect_cones(img1, img2)



# import argparse
# import os
from os.path import isfile, join
import random
# import tempfile
# import time
# import copy
# import multiprocessing
# import subprocess
# import shutil
# import cv2

import torch
import torch.nn as nn
# from torch.utils.data import DataLoader
# from tensorboardX import SummaryWriter

from PIL import Image #, ImageDraw

import torchvision
import torchvision.transforms.functional
from YOLO.models import Darknet
from YOLO.utils.datasets import ImageLabelDataset
from YOLO.utils.nms import nms
from YOLO.utils.utils import xywh2xyxy, calculate_padding

import warnings
# from tqdm import tqdm

from typing import List, Tuple, Union
import numpy as np

warnings.filterwarnings("ignore")

DetectionResults = torch.Tensor

class YOLO_N():

    def __init__(
        self,
        weights_path="YOLO/yolo_weights/pretrained_yolo.weights", #"CVC_YOLOv3/yolo_weights/pretrained_yolo.weights",
        model_cfg="YOLO/model_cfg/yolo_baseline.cfg", #"CVC_YOLOv3/model_cfg/yolo_baseline.cfg",
        conf_thres=0.8,
        nms_thres=0.25,
        vanilla_anchor=False
    ) -> None:

        # probably only use CPU since we analyse one image at a time

        # if torch.cuda.is_available():
        #     self.device = torch.device('cuda:0')
        #     self.setup_cuda()
        #     print('Using GPU')
        # else:
        #     self.device = torch.device('cpu')
        #     print('Using CPU')

        self.device = torch.device('cpu')
        # print('Using CPU')
        
        random.seed(0)
        torch.manual_seed(0)
        
        # self.model = Darknet(config_path=model_cfg,xy_loss=xy_loss,wh_loss=wh_loss,no_object_loss=no_object_loss,object_loss=object_loss,vanilla_anchor=vanilla_anchor)
        self.model = Darknet(config_path=model_cfg, vanilla_anchor=vanilla_anchor)

        self.model.load_weights(weights_path, self.model.get_start_weight_dim())

        self.model.to(self.device, non_blocking=True)
        self.model.eval()
        
        self.nms_thres = nms_thres
        self.conf_thres = conf_thres

        self.pad_h, self.pad_w, self.ratio = None, None, None

    def preprocess(self, img: Image.Image) -> torch.Tensor:

        img = img.convert('RGB')
        w, h = img.size
        new_width, new_height = self.model.img_size()
        self.pad_h, self.pad_w, self.ratio = calculate_padding(h, w, new_height, new_width)
        img = torchvision.transforms.functional.pad(img, padding=(self.pad_w, self.pad_h, self.pad_w, self.pad_h), fill=(127, 127, 127), padding_mode="constant")
        img = torchvision.transforms.functional.resize(img, (new_height, new_width))

        bw = self.model.get_bw()
        if bw:
            img = torchvision.transforms.functional.to_grayscale(img, num_output_channels=1)

        img = torchvision.transforms.functional.to_tensor(img)
        return img.unsqueeze(0)

    def single_img_detect(self, img: Image.Image) -> DetectionResults:
        with torch.no_grad():
            img_tensor = img.to(self.device, non_blocking=True)
            output = self.model(img_tensor)

            detections: torch.Tensor = output[0]
            detections = detections[detections[:, 4] > self.conf_thres]

            box_corner = torch.zeros((detections.shape[0], 4), device=detections.device)
            xy: torch.Tensor = detections[:, 0:2]
            wh = detections[:, 2:4] / 2
            box_corner[:, 0:2] = xy - wh
            box_corner[:, 2:4] = xy + wh
            probabilities = detections[:, 4]
            nms_indices = nms(box_corner, probabilities, self.nms_thres)
            main_box_corner = box_corner[nms_indices]
            if nms_indices.shape[0] == 0:  
                raise Exception("I don't even know what happened")
            
            return main_box_corner

    def getImageInfo(self) -> np.ndarray:
        return np.array([self.ratio, self.pad_h, self.pad_w])

    def detectFrame(self, frame: Union[np.ndarray, torch.Tensor]) -> Tuple[DetectionResults, np.ndarray]:
        frame = torchvision.transforms.ToPILImage()(frame)
        frame = self.preprocess(frame)
        box_corners = self.single_img_detect(frame)
        return (box_corners, self.getImageInfo())

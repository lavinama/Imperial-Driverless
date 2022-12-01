# Visual_Perception
Perception algorithm from Camera

### Exploring the code

The directory structure is as follows:

```
Visual_Perception
    ├──  Stereo
    │  ├──  __init__.py
    │  ├──  cone_locations.py
    │  └──  test.py
    ├──  YOLO
    │  ├──  __pycache__
    │  │  ├──  __init__.cpython-37.pyc
    │  │ └──  models.cpython-37.pyc
    │  ├──  model_cfg
    │  │  ├──  yolo_baseline.cfg
    │  │ └──  yolo_baseline_tiny.cfg
    │  ├──  utils
    │  │  ├──  __init__.py
    │  │  ├──  datasets.py
    │  │  ├──  nms.py
    │  │  ├──  parse_config.py
    │  │  └──  utils.py
    │  ├──  yolo_weights # Have to mannually add this folder to VSCODE
    │  │   └──  pretrained_yolo.weights
    │  ├──  __init__.py
    │  ├──  models.py
    │  └──  requirements.txt
    ├── dataset # Have to mannually add this folder to VSCODE (for testing purposes)
    │  └──  sample_image.jpg
    ├──  README.md
    ├──  YOLO_utility.py
    ├── final_YOLO.py
    └── ros_node.py
 ```

# TODO:
- [ ] Write a ROS node that listens to camera topic and pushes the data to the perception stack
- [ ] Move it to camera/camera repo
- [x] Insert the stereo algorithm into this repository
- [x] Change the algorithm so that the image it takes in is a numpy array not a '.jpg' image (this is what gets passed in ROS)

# REMEMBER TO:
- [ ] Check dimensions of the camera frames
- [ ] Remove for-loop by passing the cropped cone images as a batch
- [ ] Remove portions of the preprocessing code of the yolo by considering all images as same size
- [ ] Check for speed of algorithm (make sure it can run at acceptable speed)

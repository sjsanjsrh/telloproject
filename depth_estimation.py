#https://github.com/VictorHoffmann1/Real-Time-Depth-Estimation/blob/main/Real_Time_Depth_Estimation.ipynb

import cv2
import torch
import time
import numpy as np
import os

# Let's load a MiDaS model for depth estimation task.
model_type = "DPT_Hybrid" #best of both worlds regarding accuracy and inference speed

midas = torch.hub.load("intel-isl/MiDaS", model_type)

#Move to model to GPU if it is available.
device_name = "cuda" if torch.cuda.is_available() else "cpu"
device = torch.device(device_name)
midas.to(device)
print('MiDaS successfully loaded.')
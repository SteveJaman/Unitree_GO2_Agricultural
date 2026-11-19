"""YOLO11 (ultralytics) for the RGB-D detector: same interface as YoloV5Model.

model_type: yolo11, model_name: yolo11s (n/s/m/l/x). The weights are looked for, in order:
weights_path (fine-tuned), <torch hub dir>/<model_name>.pt, <working directory>/<model_name>.pt;
otherwise ultralytics downloads <model_name>.pt into the working directory (needs internet
once; plant_detection.sh runs from /ws/.torch/hub, so it stays there).
Class names: the model's own (COCO for the pretrained weights, in the same order as YOLOv5's).
"""

import logging
import os
from typing import Any, Dict

import numpy as np
from PIL import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

from .base_model import BaseModel


class Yolo11Model(BaseModel):
    def __init__(self, config: Dict[str, Any]):
        self.model_name = config.get("model_name") or "yolo11s"
        if self.model_name.endswith(".pt"):
            self.model_name = self.model_name[:-3]
        self.weights_path = config.get("weights_path") or None
        self.device = config.get("device") or "cpu"
        self.confidence_threshold = float(config.get("confidence_threshold", 0.25))
        self.iou_threshold = float(config.get("iou_threshold", 0.45))
        self.filter_classes = config.get("filter_classes") or None
        self.model = None
        self.names = {}
        self.logger = logging.getLogger("Yolo11Model")

    def _weights(self):
        if self.weights_path:
            return self.weights_path
        folders = [os.getcwd()]
        try:
            import torch

            folders.insert(0, torch.hub.get_dir())
        except Exception:  # noqa: BLE001  (torch hub dir is only a place to look)
            pass
        for folder in folders:
            candidate = os.path.join(folder, f"{self.model_name}.pt")
            if os.path.isfile(candidate):
                return candidate
        return f"{self.model_name}.pt"  # ultralytics downloads it into the working directory

    def load(self):
        from ultralytics import YOLO  # slow import, and only this model type needs it

        weights = self._weights()
        self.model = YOLO(weights)
        names = self.model.names
        self.names = names if isinstance(names, dict) else dict(enumerate(names))
        if self.filter_classes:
            known = [c for c in self.filter_classes if c in self.names.values()]
            unknown = [c for c in self.filter_classes if c not in self.names.values()]
            if unknown:
                self.logger.warning(f"filter_classes not in the model, ignored: {unknown}")
            self.filter_classes = known or None
        self.logger.info(f"YOLO11 '{weights}' loaded on {self.device} ({len(self.names)} classes)")

    def predict(self, image):
        """image: PIL.Image (RGB, from the node) or a BGR np.ndarray (OpenCV)."""
        if isinstance(image, Image.Image):
            frame = np.asarray(image.convert("RGB"))[:, :, ::-1]  # ultralytics wants BGR arrays
        elif isinstance(image, np.ndarray):
            frame = image
        else:
            raise ValueError("Unsupported image type. Expected np.ndarray or PIL.Image.")
        return self.model.predict(
            frame,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.device,
            verbose=False,
        )[0]

    def process_outputs(self, outputs, image_size, header):
        detections = Detection2DArray()
        detections.header = header
        if outputs is None or outputs.boxes is None or len(outputs.boxes) == 0:
            return detections
        img_w, img_h = image_size
        for xyxy, score, cls in zip(
            outputs.boxes.xyxy.tolist(), outputs.boxes.conf.tolist(), outputs.boxes.cls.tolist()
        ):
            if score < self.confidence_threshold:
                continue
            class_name = self.names.get(int(cls), str(int(cls)))
            if self.filter_classes and class_name not in self.filter_classes:
                continue
            xmin, ymin = max(0.0, xyxy[0]), max(0.0, xyxy[1])
            xmax, ymax = min(img_w - 1.0, xyxy[2]), min(img_h - 1.0, xyxy[3])

            detection = Detection2D()
            detection.header = header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis = ObjectHypothesis()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = float(score)
            detection.results.append(hypothesis)

            bbox = BoundingBox2D()
            bbox.center = Pose2D()
            bbox.center.position = Point2D()
            bbox.center.position.x = float((xmin + xmax) / 2.0)
            bbox.center.position.y = float((ymin + ymax) / 2.0)
            bbox.center.theta = 0.0
            bbox.size_x = float(xmax - xmin)
            bbox.size_y = float(ymax - ymin)
            detection.bbox = bbox
            detections.detections.append(detection)
        return detections

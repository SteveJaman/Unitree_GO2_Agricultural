from typing import Dict, Any
from .base_model import BaseModel


class ModelFactory:
    @staticmethod
    def create_model(config: Dict[str, Any]) -> BaseModel:
        model_type = config.get("model_type", "").lower()

        # Imported per type: the Hugging Face models need transformers and timm,
        # which a YOLOv5-only installation does not have.
        if model_type == "detr":
            from .hf_model import DetrModel

            return DetrModel(config)
        elif model_type == "yolos":
            from .hf_model import YolosModel

            return YolosModel(config)
        elif model_type in ("yolo11", "ultralytics"):
            from .yolo11_model import Yolo11Model

            return Yolo11Model(config)
        elif model_type == "yolov5":
            from .yolov5_model import YoloV5Model

            return YoloV5Model(config)
        else:
            raise ValueError(f"Unsupported model type: {model_type}")

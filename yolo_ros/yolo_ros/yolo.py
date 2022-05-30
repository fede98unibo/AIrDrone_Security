import os
import warnings
from typing import List, Tuple

# comment out below line to enable tensorflow outputs
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
warnings.filterwarnings(action="ignore", category=DeprecationWarning)

import tensorflow as tf
from tensorflow.python.saved_model import tag_constants

import cv2
import numpy as np


MODELS_PATH = "./src/yolo_ros/networks"


class Yolo:
    def __init__(
        self,
        name: str,
        input_size: int,
        confidence_threshold: float,
        nms_threshold: float,
    ) -> None:
        super().__init__()
        self.conf_th = confidence_threshold
        self.nms_th = nms_threshold
        self.name = name
        self.input_size = (input_size, input_size)

        with open(f"{MODELS_PATH}/coco.names") as f:
            self.labels = f.read().strip().split("\n")

        # save model to a variable otherwise the garbage collector will destroy the network
        self._model = tf.saved_model.load(
            f"{MODELS_PATH}/{name}", tags=[tag_constants.SERVING]
        )
        self._infer = self._model.signatures["serving_default"]

    def get_predictions(self, image: np.ndarray) -> np.ndarray:
        image_data = cv2.resize(image, self.input_size)
        image_data = image_data / 255.0
        image_data = image_data[np.newaxis, ...].astype(np.float32)

        batch_data = tf.constant(image_data)

        pred_bbox = self._infer(batch_data)

        for _, value in pred_bbox.items():
            results = value
        return results

    def process_results(
        self, image: np.ndarray, results: np.ndarray
    ) -> Tuple[int, List[str], np.ndarray, np.ndarray]:
        original_h, original_w, _ = image.shape
        boxes = results[:, :, 0:4]
        pred_conf = results[:, :, 4:]

        boxes, scores, classes, valid = tf.image.combined_non_max_suppression(
            boxes=tf.reshape(boxes, (tf.shape(boxes)[0], -1, 1, 4)),
            scores=tf.reshape(
                pred_conf, (tf.shape(pred_conf)[0], -1, tf.shape(pred_conf)[-1])
            ),
            max_output_size_per_class=50,
            max_total_size=50,
            iou_threshold=self.nms_th,
            score_threshold=self.conf_th,
        )

        detected = int(valid[0])

        bboxes = boxes.numpy()[0][:detected]
        for box in bboxes:
            y_min = box[0] * original_h
            x_min = box[1] * original_w
            y_max = box[2] * original_h
            x_max = box[3] * original_w
            w = x_max - x_min
            h = y_max - y_min
            c_x = x_min + w / 2
            c_y = y_min + h / 2
            box[:] = np.array((c_x, c_y, w, h))

        classes = [self.labels[int(c)] for c in classes.numpy()[0][:detected]]

        scores = scores.numpy()[0][:detected]

        return detected, classes, scores, bboxes

    def __call__(
        self, image: np.ndarray
    ) -> Tuple[int, List[str], np.ndarray, np.ndarray]:
        """Execute the yolo network on the image
        Args:
            image: numpy array of the BGR image
        Returns:
            'detected': integer, the number of object detected
            'classes': list of classes of each detected object
                len(classes) = detected
            'scores': np.ndarray with the confidence of each box 
                scores.shape=(detected,)
            'bboxes': np.ndarray with the coordinates of the box 
                in the format (center_x, center_y, width, height)
                bboxes.shape=(detected, 4)

        """
        results = self.get_predictions(image)
        return self.process_results(image, results)

#!/usr/bin/env python
# coding: utf-8

import cv2
import numpy as np
from tensorflow import keras


class FallDetector:
    """Class that identifies falls through the fall_check method."""

    def __init__(self, model_path):
        # Loading in model file
        self.model = keras.models.load_model(model_path)

    def fall_check(self, frame) -> int:
        # Resizing the image to match the target size used during training
        resized_image = cv2.resize(frame, (224, 224))

        # Converting image to a numpy array
        img_array = np.array(resized_image)

        # Domain expansion
        img_array = np.expand_dims(img_array, axis=0)

        # Rescale the image
        img_array = img_array / 255.0

        # Model making predictions
        predictions = self.model.predict(img_array)

        fall_probability = predictions[0][1]

        return fall_probability

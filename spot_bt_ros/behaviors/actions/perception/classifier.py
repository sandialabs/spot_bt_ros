"""ML Classifier related behaviors."""
from __future__ import annotations

import cv2

import py_trees

# import torch

from spot_bt_ros.data import Blackboards


class ClassifyObjectsInImage(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, path_to_model: str, first_load: bool = False):
        super().__init__(name)
        self.blackboard = Blackboards()
        self.classifications = []
        self.first_load = first_load
        self.images = None
        self.path_to_model = path_to_model
        self.model = None

    def setup(self, **kwargs):
        """Setup model inference."""
        self.logger.debug(f"\t{self.name} [ClassifyObjectsInImage::setup()]")
        try:
            if self.first_load:
                # self.model = torch.load(self.path_to_model)
                pass
        except FileNotFoundError as exp:
            self.logger.error(
                f"{exp.args} Could not find file in path {self.path_to_model}"
            )

    def initialise(self):
        """Initialize robot object and client for behavior on first tick."""
        self.logger.debug(f"\t{self.name} [ClassifyObjectsInImage::initialise()]")
        self.blackboard.perception = self.attach_blackboard_client("Perception")
        self.blackboard.perception.register_key(
            key="rgb_images", access=py_trees.common.Access.READ
        )
        self.images = self.blackboard.perception.rgb_images
        # TODO: Consider removing this since the model should be loaded at setup.
        if not self.first_load:
            self.blackboard.perception.register_key(
                key="model", access=py_trees.common.Access.WRITE
            )
            self.model = self.blackboard.perception.model

    def update(self) -> py_trees.common.Status:
        """Run the ClassifyObjectsInImage behavior when ticked."""
        self.logger.debug(f"\t\t{self.name} [ClassifyObjectsInImage::update()]")
        try:
            for image in self.images:
                # TODO: preprocess image data

                # input preprocessed image into ML model
                output = self.model(image)
                self.classifications.append(output)

        except:  # pylint: disable=bare-except
            return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: str):
        """Terminate behavior and save information."""
        self.blackboard.perception.model = self.model
        self.logger.debug(
            f"\t{self.name} [ClassifyObjectsInImage::terminate().terminate()]"
            f"[{self.status}->{new_status}]"
        )

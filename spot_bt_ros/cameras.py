from __future__ import annotations

from bosdyn.api import image_pb2

import cv2

from google.protobuf import wrappers_pb2

import numpy as np


DEPTH_CAMERA_OPTIONS = [
    "frontleft_depth",
    "frontright_depth",
    "left_depth",
    "right_depth",
    "back_depth",
]


DEPTH_CAMERA_OPTIONS_WITH_ARM = [
    "frontleft_depth",
    "frontright_depth",
    "left_depth",
    "right_depth",
    "back_depth",
    "hand_depth",
]


PIXEL_FORMAT_ENUM_ENCODING_INFORMATION = {
    1: (np.uint8, 1, ".jpg"),
    3: (np.uint8, 3, ".jpg"),
    4: (np.uint8, 4, ".jpg"),
    5: (np.uint16, 1, ".png"),
    6: (np.uint8, 2, ".jpg"),
}


PIXEL_FORMAT_STRING_ENCODING_INFORMATION = {
    "PIXEL_FORMAT_GREYSCALE_U8": (np.uint8, 1, ".jpg"),
    "PIXEL_FORMAT_RGB_U8": (np.uint8, 3, ".jpg"),
    "PIXEL_FORMAT_RGBA_U8": (np.uint8, 4, ".jpg"),
    "PIXEL_FORMAT_DEPTH_U16": (np.uint16, 1, ".png"),
    "PIXEL_FORMAT_GREYSCALE_U16": (np.uint8, 2, ".jpg"),
}


IMAGE_CAMERA_OPTIONS = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "left_fisheye_image",
    "right_fisheye_image",
    "back_fisheye_image",
]


IMAGE_CAMERA_OPTIONS_WITH_ARM = [
    "frontleft_fisheye_image",
    "frontright_fisheye_image",
    "left_fisheye_image",
    "right_fisheye_image",
    "back_fisheye_image",
    "hand_color_image",
]


IMAGE_ROTATION_ANGLES = {
    "back_fisheye_image": 0,
    "frontleft_fisheye_image": -78,
    "frontright_fisheye_image": -102,
    "left_fisheye_image": 0,
    "right_fisheye_image": 180,
}


ROS_IMAGE_CAMERA_OPTIONS = [
    "back",
    "frontleft",
    "frontright",
    "left",
    "right",
]


ROS_IMAGE_CAMERA_OPTIONS_WITH_ARM = [
    "back",
    "frontleft",
    "frontright",
    "left",
    "right",
    "hand",
]


ROS_IMAGE_CAMERA_TOPICS = {
    "back": "camera/back/image",
    "frontleft": "camera/frontleft/image",
    "frontright": "camera/frontright/image",
    "left": "camera/left/image",
    "right": "camera/right/image",
}


ROS_IMAGE_CAMERA_TOPICS_WITH_ARM = {
    "back": "camera/back/image",
    "frontleft": "camera/frontleft/image",
    "frontright": "camera/frontright/image",
    "left": "camera/left/image",
    "right": "camera/right/image",
    "hand": "camera/hand/image",
}


ROS_DEPTH_CAMERA_OPTIONS = [
    "back",
    "frontleft",
    "frontright",
    "left",
    "right",
]


ROS_DEPTH_CAMERA_OPTIONS_WITH_ARM = [
    "back",
    "frontleft",
    "frontright",
    "left",
    "right",
    "hand",
]


ROS_DEPTH_CAMERA_TOPICS = {
    "back": "depth/back/image",
    "frontleft": "depth/frontleft/image",
    "frontright": "depth/frontright/image",
    "left": "depth/left/image",
    "right": "depth/right/image",
}


ROS_DEPTH_CAMERA_TOPICS_WITH_ARM = {
    "back": "depth/back/image",
    "frontleft": "depth/frontleft/image",
    "frontright": "depth/frontright/image",
    "left": "depth/left/image",
    "right": "depth/right/image",
    "hand": "depth/hand/image",
}


def find_center_pixel(polygon) -> tuple[int, int]:
    """Find polygon's approx. center by taking the axis-aligned bounding box's center."""
    min_x = np.inf
    min_y = np.inf
    max_x = -np.inf
    max_y = -np.inf
    for vert in polygon.vertexes:
        if vert.x < min_x:
            min_x = vert.x
        if vert.y < min_y:
            min_y = vert.y
        if vert.x > max_x:
            max_x = vert.x
        if vert.y > max_y:
            max_y = vert.y
    x = np.fabs(max_x - min_x) / 2.0 + min_x
    y = np.fabs(max_y - min_y) / 2.0 + min_y
    return x, y


def get_bounding_box_image(response):
    dtype = np.uint8
    img = np.fromstring(response.image_response.shot.image.data, dtype=dtype)
    if response.image_response.shot.image.format == image_pb2.Image.FORMAT_RAW:
        img = img.reshape(response.image_response.shot.image.rows,
                          response.image_response.shot.image.cols)
    else:
        img = cv2.imdecode(img, -1)

    # Convert to BGR so we can draw colors
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Draw bounding boxes in the image for all the detections.
    for obj in response.object_in_image:
        conf_msg = wrappers_pb2.FloatValue()
        obj.additional_properties.Unpack(conf_msg)
        confidence = conf_msg.value

        polygon = []
        min_x = float('inf')
        min_y = float('inf')
        for v in obj.image_properties.coordinates.vertexes:
            polygon.append([v.x, v.y])
            min_x = min(min_x, v.x)
            min_y = min(min_y, v.y)

        polygon = np.array(polygon, np.int32)
        polygon = polygon.reshape((-1, 1, 2))
        cv2.polylines(img, [polygon], True, (0, 255, 0), 2)

        caption = "{} {:.3f}".format(obj.name, confidence)
        cv2.putText(img, caption, (int(min_x), int(min_y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return img


def get_encoding_for_pixel_format_string(
    pixel_format: str,
) -> tuple[type, int, str]:
    """Return encoding information for specific pixel format string."""
    return PIXEL_FORMAT_STRING_ENCODING_INFORMATION[pixel_format]


def get_encoding_for_pixel_format_enum(pixel_format: int) -> tuple[type, int, str]:
    """Return encoding information for specific pixel format enum."""
    return PIXEL_FORMAT_ENUM_ENCODING_INFORMATION[pixel_format]


def pixel_format_type_strings() -> list[str]:
    """Return names of possible image pixel formats."""
    return image_pb2.Image.PixelFormat.keys()[1:]


def pixel_format_string_to_enum(enum_string: str) -> int:
    """Convert pixel format string to respective enum."""
    return dict(image_pb2.Image.PixelFormat.items()).get(enum_string)

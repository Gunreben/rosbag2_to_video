# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pathlib
import sys
from typing import Any
from typing import Callable
from typing import Tuple
from typing import TYPE_CHECKING

import cv2
from cv_bridge import CvBridge

from rclpy.serialization import deserialize_message
import rclpy.time
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

if TYPE_CHECKING:
    from argparse import ArgumentParser
    import numpy as np


def cv2_video_writer_fourcc(codec_str: str) -> int:
    """Validate the user provided video codec string and return it as fourcc."""
    if len(codec_str) != 4:
        raise ValueError(f'codecs should be specified using fourcc, got "{codec_str}"')
    try:
        return cv2.VideoWriter.fourcc(*codec_str)
    except Exception as e:
        raise ValueError(f'"{codec_str}" is not a valid fourcc codec: {e}')


class CommandInputError(ValueError):
    """Error raised when provided command line arguments are not valid."""

    def __init__(self, msg: str):
        super().__init__(msg)


def get_stamp_from_image_msg(image_msg) -> float:
    """Convert timestamp in msg from nanoseconds to seconds."""
    stamp = rclpy.time.Time.from_msg(image_msg.header.stamp).nanoseconds
    return stamp / 1e9


def get_topic_type(topic_name: str, topics_and_types) -> str:
    """Get the topic type from the topic name and the topic information in the bag."""
    try:
        topic_type = next(x for x in topics_and_types if x.name == topic_name).type
    except StopIteration:
        raise ValueError(
            f'Topic {topic_name} was not recorded in the bagfile')
    if topic_type not in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
        raise ValueError(
            'topic type should be sensor_msgs/msg/Image or '
            f'sensor_msgs/msg/CompressedImage, got {topic_type}')
    return topic_type


class SequentialImageBagReader:
    """Reader of images from a bagfile source sequentially."""

    def __init__(self, bag_reader: rosbag2_py.SequentialReader, topic_name: str):
        """
        Create image bagfile reader.

        :param bag_reader: rosbag2_py sequential reader instance.
        :param topic_name: topic from where to read the images in the bagfile.
        """
        self._bag_reader: rosbag2_py.SequentialReader = bag_reader
        self._cvbridge: CvBridge = CvBridge()
        self._topic_name: str = topic_name
        self._topic_type: str = get_topic_type(topic_name, bag_reader.get_all_topics_and_types())
        self._msg_type = get_message(self._topic_type)
        self._msg_to_cv2: Callable[[Any], 'np.ndarray']
        if self._topic_type == 'sensor_msgs/msg/Image':
            self._msg_to_cv2 = lambda msg: self._cvbridge.imgmsg_to_cv2(msg, 'bgr8')
        elif self._topic_type == 'sensor_msgs/msg/CompressedImage':
            self._msg_to_cv2 = lambda msg: self._cvbridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
        self._bag_reader.set_filter(storage_filter)

    def get_next(self) -> Tuple['np.ndarray', float]:
        """Return next image and its timestamp."""
        _, data, _ = self._bag_reader.read_next()
        image_msg = deserialize_message(data, self._msg_type)
        return self._msg_to_cv2(image_msg), get_stamp_from_image_msg(image_msg)

    def has_next(self):
        """Return true if there is at least one more message to read."""
        return self._bag_reader.has_next()


class SequentialVideoWriter:
    """Create videos from images provided sequentially."""

    def __init__(
        self,
        cv_video_writer: cv2.VideoWriter,
        first_cv_image: 'np.ndarray',
        start_stamp: float,
        fps: float,
    ):
        """
        Create a video writer.

        :param cv_video_writer: A cv2.VideoWriter instance, already opened.
        :param first_cv_image: First image to write in the video.
        :param start_stamp: Timestamp of the first image.
        :param fps: Fps used to record the video.
        """
        self._fps: float = fps
        self._cv_video_writer: cv2.VideoWriter = cv_video_writer
        self._cv_video_writer.write(first_cv_image)
        self._images_processed: int = 1
        self._frame_count: int = 1
        self._images_skipped: int = 0
        self._start_stamp: float = start_stamp
        self._last_image: 'np.ndarray' = first_cv_image
        self._last_image_written_once: bool = True

    def add_frame(self, cv_image: 'np.ndarray', stamp: float):
        """Add a frame to the video, given an image and its timestamp."""
        self._images_processed += 1
        t_from_start = stamp - self._start_stamp

        # accept jitter up to 0.5/fps
        if t_from_start < (float(self._frame_count) - 0.5) / self._fps:
            self._last_image = cv_image
            if not self._last_image_written_once:
                self._images_skipped += 1
                print(
                    'video fps is too low compared to image publish rate, skipping one message',
                    file=sys.stderr)
            self._last_image_written_once = False
            return
        current_image_frame_index = int(round(t_from_start * self._fps))
        repeat_last_image = current_image_frame_index - self._frame_count
        for _ in range(repeat_last_image):
            self._cv_video_writer.write(self._last_image)
        self._cv_video_writer.write(cv_image)
        self._last_image = cv_image
        if not self._last_image_written_once and repeat_last_image == 0:
            self._images_skipped += 1
            print(
                'video fps is too low compared to image publish rate, skipping one message',
                file=sys.stderr)
        self._last_image_written_once = True
        self._frame_count += repeat_last_image + 1

    def close(self):
        """Close the video writer."""
        if not self._last_image_written_once:
            self._cv_video_writer.write(self._last_image)
        self._cv_video_writer.release()

    @property
    def images_processed(self) -> int:
        """
        Get the number of images that were provided.

        Images that were skipped and not written are also counted.
        """
        return self._images_processed

    @property
    def frames_written(self) -> int:
        """Get the number of frames that were written to the video."""
        return self._frame_count

    @property
    def images_skipped(self) -> int:
        """Get the number of images that were skipped and not written to the video."""
        return self._images_skipped


def create_sequential_image_bag_reader(
    bag_path: str, storage_id: str, topic_name: str
) -> SequentialImageBagReader:
    """
    Create a SequentialImageBagReader instance.

    :param bag_path: Path to the bagfile (folder with metadata or file).
    :param storage_id: Storage id of the bagfile.
    :param topic_name: Name of the image topic.
    """
    if pathlib.Path(bag_path).is_dir():
        # load storage id from metadata.yaml
        storage_options = rosbag2_py.StorageOptions(uri=bag_path)
    else:
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    # TODO(jacobperron): Shouldn't we be able to infer serialization format from metadaya.yaml?
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    bag_reader = rosbag2_py.SequentialReader()
    bag_reader.open(storage_options, converter_options)

    if not bag_reader.has_next():
        raise CommandInputError('empty bag file')
    return SequentialImageBagReader(bag_reader, topic_name)


def create_sequential_video_writer(
    output_path: str,
    codec: int,
    fps: float,
    first_cv_image: 'np.ndarray',
    start_stamp: float
):
    """
    Create a SequentialVideoWriter instance.

    :param output_path: Path of the video to be created.
    :param codec: fourcc of the codec to be used.
    :param fps: Video frame per second to be used.
    :param first_cv_image: First image to write to the video.
        Video width and height are got from here.
    :param start_stamp: Timestamp of the first image.
    """
    height, width, _ = first_cv_image.shape
    cv_video_writer = cv2.VideoWriter()
    success = cv_video_writer.open(
        output_path,
        cv2.CAP_FFMPEG,
        codec,
        fps,
        (width, height))
    if not success:
        raise CommandInputError(f'Failed to open file {output_path}')
    return SequentialVideoWriter(cv_video_writer, first_cv_image, start_stamp, fps)


def convert_bag_to_video(
    bag_path: str, topic_name: str, output_path: str, codec: int, fps: float,
    storage_id: str = '' # Add storage_id as optional, derive if not provided
):
    """
    Convert image sequence from a rosbag topic into a video file.

    :param bag_path: Path to the bagfile (folder with metadata or file).
    :param topic_name: Name of the image topic.
    :param output_path: Path to the output video file.
    :param codec: OpenCV FourCC integer code for the video codec.
    :param fps: Frames per second for the output video.
    :param storage_id: Rosbag2 storage id (e.g., 'sqlite3'). If empty, attempts to derive.
    :raises ValueError: If inputs are invalid, topic not found/supported, or codec issues.
    :raises Exception: For underlying issues during bag reading or video writing.
    """
    output_path = pathlib.Path(output_path)
    if not output_path.parent.exists():
         output_path.parent.mkdir(parents=True)
    elif output_path.exists():
         print(f'Output file {output_path} already exists, overwriting', file=sys.stderr)
         output_path.unlink()

    # --- Setup Bag Reader --- (Adapted from create_sequential_image_bag_reader)
    bag_file_path = pathlib.Path(bag_path)
    actual_storage_id = storage_id
    if bag_file_path.is_dir():
        metadata_file = bag_file_path / 'metadata.yaml'
        if not metadata_file.is_file():
             raise ValueError(f"Bag directory does not contain metadata.yaml: {bag_path}")
        # Attempt to load storage id from metadata.yaml if not provided
        if not actual_storage_id:
            # Requires PyYAML, let's keep it simple for now and require storage_id if path is dir
            # Or just assume sqlite3 if it's a dir and storage_id not given?
            # For now, let's keep the logic as it was: derive from metadata if possible
            # storage_options will handle this using default discovery if storage_id is empty
             pass
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=actual_storage_id)
    elif bag_file_path.is_file():
        if not actual_storage_id:
            # If it's a file, user *must* provide storage_id (or default works?)
            # Let's default to sqlite3 if not provided and it's a file path
            actual_storage_id = 'sqlite3'
            print(f"Assuming storage_id='{actual_storage_id}' for bag file: {bag_path}", file=sys.stderr)
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=actual_storage_id)
    else:
        raise ValueError(f"Bag path is not a valid file or directory: {bag_path}")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    bag_reader = rosbag2_py.SequentialReader()
    try:
        bag_reader.open(storage_options, converter_options)
    except Exception as e:
        raise RuntimeError(f"Failed to open bag: {e}") # More specific error

    try:
        # This implicitly checks topic existence and type via get_topic_type
        image_reader = SequentialImageBagReader(bag_reader, topic_name)
    except ValueError as e:
        # Clean up reader if topic validation fails
        del bag_reader
        raise e # Re-raise the ValueError (topic not found/invalid type)

    if not image_reader.has_next():
        del bag_reader # Clean up reader
        raise ValueError(f'Topic "{topic_name}" seems to have no messages in the bag')

    # --- Setup Video Writer --- (Adapted from create_sequential_video_writer)
    try:
        first_image, start_stamp = image_reader.get_next()
    except Exception as e:
        del bag_reader
        raise RuntimeError(f"Error reading first image from topic '{topic_name}': {e}")

    width = first_image.shape[1]
    height = first_image.shape[0]
    cv_video_writer = cv2.VideoWriter(
        str(output_path), codec, fps, (width, height)) # Use integer codec directly

    if not cv_video_writer.isOpened():
        del bag_reader
        # Try to provide a more helpful error message about the codec
        raise ValueError(
            f'Failed to open video writer for "{output_path}". '
            f'Codec fourcc={codec} may not be supported for the '
            f'container format (based on extension {output_path.suffix}) or system setup.'
        )

    video_writer = SequentialVideoWriter(cv_video_writer, first_image, start_stamp, fps)

    # --- Process Messages --- 
    print(
        f'Writing video for topic "{topic_name}" to {output_path} at {fps} FPS '
        f'({width}x{height})...')
    try:
        while image_reader.has_next():
            try:
                image, stamp = image_reader.get_next()
                video_writer.add_frame(image, stamp)
            except Exception as e:
                 # Log error for specific message but try to continue?
                 # Or fail fast? Let's fail fast for now.
                 raise RuntimeError(f"Error processing message for topic '{topic_name}': {e}")

    finally:
        # Ensure resources are released even if errors occur during processing
        video_writer.close()
        del bag_reader # Ensure SequentialReader is destroyed before Python GC potentially closes files

    print(
        f'Finished writing {video_writer.frames_written} frames '
        f'({video_writer.images_processed} images processed, {video_writer.images_skipped} skipped).'
    )
    # Optionally return some stats or just finish

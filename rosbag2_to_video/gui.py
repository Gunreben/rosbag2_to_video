import sys
import os
import pathlib
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QLineEdit,
    QFileDialog, QDoubleSpinBox, QListWidgetItem, QMessageBox, QStatusBar,
    QComboBox
)
from PySide6.QtCore import Qt, QThread, Signal

# Import refactored functions from api.py
from .api import convert_bag_to_video, cv2_video_writer_fourcc, get_topic_type
import rosbag2_py # For reading bag metadata

# Import qt-material
import qt_material

# Define supported image topic types
IMAGE_TOPIC_TYPES = ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage')

# Define common codecs
# List common FourCC codes (as strings)
# See: https://www.fourcc.org/codecs.php
# Availability depends on system OpenCV/FFmpeg build
AVAILABLE_CODECS = {
    'avc1': 'H.264 / AVC',
    'mp4v': 'MPEG-4',
    'mjpg': 'Motion JPEG',
    'xvid': 'Xvid',
    'h263': 'H.263',
    # Add more common ones if needed
    'iyuv': 'Intel Indeo YUV',
    'raw ': 'Uncompressed RGB',
    'png ': 'PNG' # Note: space padding for 4 chars
}

class BagInfoThread(QThread):
    """Loads bag info in a separate thread to avoid freezing the GUI."""
    result_ready = Signal(object, list) # Sends bag_path, list of (topic_name, topic_type)
    error_occurred = Signal(str)

    def __init__(self, bag_path):
        super().__init__()
        self.bag_path = bag_path

    def run(self):
        try:
            # Attempt to determine storage_id, default if needed
            storage_id = ''
            if os.path.isfile(self.bag_path):
                # Heuristic: assume sqlite3 for single files if not specified
                # A more robust solution might involve peeking at the file
                storage_id = 'sqlite3' 
            storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id=storage_id)
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr', output_serialization_format='cdr')

            reader = rosbag2_py.SequentialReader()
            # Wrap open in try/except as it can fail
            try:
                reader.open(storage_options, converter_options)
            except Exception as e:
                 # Check if it failed because storage_id was wrong (e.g., file is actually dir)
                 if 'Sqlite' in str(e) and os.path.isdir(self.bag_path):
                     # Retry with empty storage ID if it looks like a directory
                     storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='')
                     reader.open(storage_options, converter_options)
                 else:
                     raise # Re-raise original exception if retry condition not met

            topic_metadata = reader.get_all_topics_and_types()
            del reader # Release reader resources
            self.result_ready.emit(self.bag_path, topic_metadata)
        except Exception as e:
            self.error_occurred.emit(f"Error reading bag metadata: {e}")


class ExportThread(QThread):
    """Runs the video export process in a separate thread."""
    progress_update = Signal(str)
    finished_signal = Signal(str) # Final status message
    error_occurred = Signal(str, str) # topic_name, error_message

    def __init__(self, bag_path, output_dir, topics_to_export, fps, codec_str):
        super().__init__()
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.topics_to_export = topics_to_export
        self.fps = fps
        self.codec_str = codec_str
        # Use the imported conversion function
        self.convert_func = convert_bag_to_video

    def run(self):
        if not self.topics_to_export:
            self.finished_signal.emit("No topics selected for export.")
            return

        num_topics = len(self.topics_to_export)
        success_count = 0
        errors = []

        self.progress_update.emit(f"Starting export of {num_topics} topic(s)...")

        # Get FourCC code using the imported function
        try:
            codec_fourcc = cv2_video_writer_fourcc(self.codec_str)
        except ValueError as e:
            self.finished_signal.emit(f"Error setting codec '{self.codec_str}': {e}. Export cancelled.")
            return

        for i, topic_name in enumerate(self.topics_to_export):
            self.progress_update.emit(f"[{i+1}/{num_topics}] Exporting {topic_name}...")
            # Sanitize topic name for filename
            sanitized_name = topic_name.strip('/').replace('/', '_')
            # Ensure output has .mp4 extension (or let api handle? API currently doesn't force)
            output_filename = os.path.join(self.output_dir, f"{sanitized_name}.mp4")

            try:
                # Call the actual conversion function from api.py
                # storage_id is left empty ('') - convert_bag_to_video will derive it
                self.convert_func(
                    bag_path=self.bag_path,
                    topic_name=topic_name,
                    output_path=output_filename,
                    codec=codec_fourcc, # Pass the integer FourCC code
                    fps=self.fps,
                    storage_id='' # Let the API function handle discovery
                )
                success_count += 1
                self.progress_update.emit(f"[{i+1}/{num_topics}] Finished exporting {topic_name}")
            except (ValueError, RuntimeError, Exception) as e: # Catch potential errors from conversion
                error_msg = f"Error exporting {topic_name}: {e}"
                self.error_occurred.emit(topic_name, error_msg)
                errors.append(error_msg)
                self.progress_update.emit(f"[{i+1}/{num_topics}] Error exporting {topic_name}")

        final_message = f"Export finished. {success_count}/{num_topics} topics exported successfully."
        if errors:
            final_message += f"\n\nEncountered errors:\n" + "\n".join(errors)
        self.finished_signal.emit(final_message)


class BagToVideoApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('ROS Bag to Video Exporter')
        self.setGeometry(100, 100, 650, 450)

        self.bag_path = None
        self.output_dir = "."
        self.bag_info_thread = None
        self.export_thread = None

        self.main_layout = QVBoxLayout(self)

        # Bag Selection - Remove Load Topics Button
        bag_layout = QHBoxLayout()
        self.bag_label = QLabel("Bag File/Folder: Not Selected")
        self.bag_label.setWordWrap(True)
        self.browse_bag_button = QPushButton("Browse Bag")
        bag_layout.addWidget(self.bag_label, 1) # Give label more space
        bag_layout.addWidget(self.browse_bag_button)
        self.main_layout.addLayout(bag_layout)

        # Topic List
        self.topic_list_widget = QListWidget()
        self.topic_list_widget.setSelectionMode(QListWidget.NoSelection)
        self.main_layout.addWidget(QLabel("Image Topics (Select to Export):"))
        self.main_layout.addWidget(self.topic_list_widget)

        # Output Settings
        output_layout = QHBoxLayout()
        self.output_dir_label = QLabel(f"Output Dir: {self.output_dir}")
        self.output_dir_label.setWordWrap(True)
        self.browse_output_button = QPushButton("Browse Output Dir")
        output_layout.addWidget(self.output_dir_label, 1)
        output_layout.addWidget(self.browse_output_button)
        self.codec_label = QLabel("Codec:")
        self.codec_combobox = QComboBox()
        for fourcc, description in AVAILABLE_CODECS.items():
            self.codec_combobox.addItem(f"{fourcc} ({description})", fourcc)
        self.codec_combobox.setCurrentIndex(0)
        self.codec_combobox.setToolTip("Select video codec (FourCC). Availability depends on system.")
        self.fps_label = QLabel("FPS:")
        self.fps_spinbox = QDoubleSpinBox()
        self.fps_spinbox.setRange(1.0, 120.0)
        self.fps_spinbox.setValue(30.0)
        self.fps_spinbox.setSingleStep(1.0)
        output_layout.addStretch(1)
        output_layout.addWidget(self.codec_label)
        output_layout.addWidget(self.codec_combobox)
        output_layout.addWidget(self.fps_label)
        output_layout.addWidget(self.fps_spinbox)
        self.main_layout.addLayout(output_layout)

        # Export Button
        self.export_button = QPushButton("Export Selected Topics")
        self.export_button.setEnabled(False)
        self.main_layout.addWidget(self.export_button)

        # Status Bar
        self.status_bar = QStatusBar()
        self.main_layout.addWidget(self.status_bar)
        self.status_bar.showMessage("Ready. Select a bag file.")

        # --- Connections - Remove Load Topics ---
        self.browse_bag_button.clicked.connect(self.browse_bag)
        self.browse_output_button.clicked.connect(self.browse_output_dir)
        self.export_button.clicked.connect(self.start_export)
        self.topic_list_widget.itemChanged.connect(self.update_export_button_state)

    def browse_bag(self):
        dialog = QFileDialog(self)
        dialog.setFileMode(QFileDialog.ExistingFile)
        dialog.setNameFilter("ROS Bag (metadata.yaml *.db3);;All Files (*)")
        dialog.setWindowTitle("Select ROS Bag (metadata.yaml or .db3 file)")

        if dialog.exec():
            selected_files = dialog.selectedFiles()
            if selected_files:
                selected_path = selected_files[0]
                bag_path_to_use = selected_path
                if os.path.basename(selected_path) == 'metadata.yaml' and os.path.isdir(os.path.dirname(selected_path)):
                    bag_path_to_use = os.path.dirname(selected_path)
                    print(f"Using bag directory: {bag_path_to_use}")
                
                # Check if path actually changed before reloading
                if bag_path_to_use != self.bag_path:
                    self.bag_path = bag_path_to_use
                    self.bag_label.setText(f"Bag: {self.bag_path}")
                    self.topic_list_widget.clear()
                    self.export_button.setEnabled(False)
                    # self.load_topics_button.setEnabled(True) # Removed
                    # self.status_bar.showMessage(f"Bag selected: {self.bag_path}. Click 'Load Topics'.") # Removed message
                    
                    # Call load_topics directly
                    self.load_topics() 
                else:
                    # Same bag selected again, maybe show a message?
                    self.status_bar.showMessage(f"Bag already loaded: {self.bag_path}")

    def load_topics(self):
        if not self.bag_path:
            # This case might be less likely now, but keep for safety
            self.show_error("No bag file selected.")
            return

        # Disable browse button while loading
        self.browse_bag_button.setEnabled(False) 
        self.status_bar.showMessage(f"Loading topics from {self.bag_path}...")
        # self.load_topics_button.setEnabled(False) # Removed
        self.export_button.setEnabled(False)
        self.topic_list_widget.clear()

        self.bag_info_thread = BagInfoThread(self.bag_path)
        self.bag_info_thread.result_ready.connect(self.on_topics_loaded)
        self.bag_info_thread.error_occurred.connect(self.on_topic_load_error)
        # Re-enable browse button when finished (success or error)
        self.bag_info_thread.finished.connect(lambda: self.browse_bag_button.setEnabled(True))
        # self.bag_info_thread.finished.connect(lambda: self.load_topics_button.setEnabled(True)) # Removed
        self.bag_info_thread.start()

    def on_topics_loaded(self, bag_path, topic_metadata):
        if bag_path != self.bag_path:
            return
        # self.browse_bag_button.setEnabled(True) # Moved to finished signal connection
        image_topics_found = False
        for topic_info in topic_metadata:
            if topic_info.type in IMAGE_TOPIC_TYPES:
                item = QListWidgetItem(f"{topic_info.name} ({topic_info.type})")
                item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
                item.setCheckState(Qt.Unchecked)
                self.topic_list_widget.addItem(item)
                image_topics_found = True
        if image_topics_found:
             self.status_bar.showMessage(f"Found {self.topic_list_widget.count()} image topics. Select topics to export.")
        else:
             self.status_bar.showMessage("No suitable image topics found in the bag.")
             self.export_button.setEnabled(False)
        self.update_export_button_state()

    def on_topic_load_error(self, error_message):
        # self.browse_bag_button.setEnabled(True) # Moved to finished signal connection
        self.show_error(f"Error loading topics: {error_message}")
        self.status_bar.showMessage("Error loading topics.")
        # self.load_topics_button.setEnabled(True) # Removed

    def browse_output_dir(self):
        directory = QFileDialog.getExistingDirectory(self, "Select Output Directory", self.output_dir)
        if directory:
            self.output_dir = directory
            self.output_dir_label.setText(f"Output Dir: {self.output_dir}")
            self.status_bar.showMessage(f"Output directory set to: {self.output_dir}")

    def update_export_button_state(self):
        checked_count = 0
        for i in range(self.topic_list_widget.count()):
            if self.topic_list_widget.item(i).checkState() == Qt.Checked:
                checked_count += 1
        self.export_button.setEnabled(checked_count > 0 and (self.export_thread is None or not self.export_thread.isRunning()))

    def start_export(self):
        if not self.bag_path or not self.output_dir:
            self.show_error("Please select a bag file and output directory.")
            return

        selected_topics = []
        for i in range(self.topic_list_widget.count()):
            item = self.topic_list_widget.item(i)
            if item.checkState() == Qt.Checked:
                topic_name = item.text().split(' (')[0]
                selected_topics.append(topic_name)

        if not selected_topics:
            self.show_error("No topics selected for export.")
            return

        fps = self.fps_spinbox.value()
        selected_codec_str = self.codec_combobox.currentData()

        # Disable controls during export
        self.export_button.setEnabled(False)
        self.browse_bag_button.setEnabled(False) # Disable browse bag during export
        # self.load_topics_button.setEnabled(False) # Removed
        self.browse_output_button.setEnabled(False)
        self.topic_list_widget.setEnabled(False)
        self.fps_spinbox.setEnabled(False)
        self.codec_combobox.setEnabled(False)

        self.status_bar.showMessage(f"Starting export...")

        self.export_thread = ExportThread(
            self.bag_path, self.output_dir, selected_topics, fps, selected_codec_str
        )
        self.export_thread.progress_update.connect(self.status_bar.showMessage)
        self.export_thread.error_occurred.connect(self.on_export_error)
        self.export_thread.finished_signal.connect(self.on_export_finished)
        self.export_thread.finished.connect(self.on_export_thread_completed)
        self.export_thread.start()

    def on_export_error(self, topic_name, error_message):
        print(f"ERROR: {error_message}")
        self.status_bar.showMessage(f"Error exporting {topic_name}. See final report.")

    def on_export_finished(self, final_message):
        if "Encountered errors" in final_message:
            QMessageBox.warning(self, "Export Complete with Errors", final_message)
        else:
            QMessageBox.information(self, "Export Complete", final_message)
        self.status_bar.showMessage("Export finished. Ready.")

    def on_export_thread_completed(self):
        self.export_thread = None
        # Re-enable controls
        self.browse_bag_button.setEnabled(True) # Re-enable browse bag
        # self.load_topics_button.setEnabled(self.bag_path is not None) # Removed
        self.browse_output_button.setEnabled(True)
        self.topic_list_widget.setEnabled(True)
        self.fps_spinbox.setEnabled(True)
        self.codec_combobox.setEnabled(True)
        self.update_export_button_state()

    def show_error(self, message):
        QMessageBox.warning(self, "Error", message)

    def closeEvent(self, event):
        # Clean up threads if they are running
        if self.bag_info_thread and self.bag_info_thread.isRunning():
            # We might want to wait or terminate, but let's just warn for now
            print("Warning: Closing while bag info thread is running.")
        if self.export_thread and self.export_thread.isRunning():
            reply = QMessageBox.question(self, 'Confirm Exit',
                                       "Export is in progress. Are you sure you want to exit?",
                                       QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.Yes:
                 # Terminating threads forcefully can be risky,
                 # consider adding a proper cancellation mechanism later.
                 # self.export_thread.terminate() # Use with caution
                 event.accept()
            else:
                 event.ignore()
        else:
            event.accept()


def main():
    app = QApplication(sys.argv)

    # Define extra colors for the theme
    extra = {
        # Ensure text is white
        'primaryTextColor': '#ffffff',
        'secondaryTextColor': '#ffffff',
        # You can add other customizations here if needed
        # 'density_scale': '-1', # Make controls slightly smaller
    }

    # Apply qt-material theme with extra customization
    qt_material.apply_stylesheet(app, theme='dark_blue.xml', extra=extra)

    window = BagToVideoApp()
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main() 
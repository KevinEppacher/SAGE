from flask import Flask, render_template, request
import os
import atexit
from werkzeug.utils import secure_filename
from gui.ros2backend import ROS2Backend

class WebGUI:
    """Handles Flask app routing and user interactions."""
    def __init__(self, ros_backend: ROS2Backend):
        self.ros_backend = ros_backend
        self.app = Flask(__name__)
        self.upload_folder = os.path.join(self.app.root_path, 'static', 'uploaded')
        os.makedirs(self.upload_folder, exist_ok=True)
        self.app.config['UPLOAD_FOLDER'] = self.upload_folder
        atexit.register(self.clean_upload_folder)
        self.setup_routes()

    def clean_upload_folder(self):
        self.ros_backend.get_logger().info(f"Cleaning upload folder: {self.upload_folder}")
        for filename in os.listdir(self.upload_folder):
            path = os.path.join(self.upload_folder, filename)
            if os.path.isfile(path):
                os.unlink(path)

    def setup_routes(self):
        @self.app.route("/", methods=["GET", "POST"])
        def index():
            uploaded_filename = None
            if request.method == "POST":
                if 'upload_image' in request.form:
                    file = request.files.get("image_file")
                    if file and file.filename != '':
                        filename = secure_filename(file.filename)
                        save_path = os.path.join(self.app.config['UPLOAD_FOLDER'], filename)
                        file.save(save_path)
                        self.ros_backend.get_logger().info(f"Image saved to: {save_path}")
                        self.ros_backend.publish_image(save_path)
                        return render_template("index.html", uploaded_image=filename)

                elif 'text_input' in request.form:
                    text = request.form.get("text_input")
                    self.ros_backend.publish_text(text)

            return render_template("index.html", uploaded_image=uploaded_filename)

    def run(self):
        self.app.run(host="0.0.0.0", port=5000, debug=True)


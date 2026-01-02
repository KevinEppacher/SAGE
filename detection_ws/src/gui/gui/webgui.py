from flask import Flask, render_template, request
import os
import atexit
from werkzeug.utils import secure_filename


class WebGUI:
    def __init__(self, ros_backend):
        self.ros_backend = ros_backend
        self.app = Flask(__name__)

        self.upload_folder = os.path.join(
            self.app.root_path, 'static', 'uploaded'
        )
        os.makedirs(self.upload_folder, exist_ok=True)
        self.app.config['UPLOAD_FOLDER'] = self.upload_folder

        atexit.register(self.clean_upload_folder)
        self.setup_routes()

    def clean_upload_folder(self):
        for f in os.listdir(self.upload_folder):
            path = os.path.join(self.upload_folder, f)
            if os.path.isfile(path):
                os.unlink(path)

    def setup_routes(self):
        @self.app.route("/", methods=["GET", "POST"])
        def index():
            if request.method == "POST":

                # --- User prompt -----------------------------------------
                if "user_prompt" in request.form:
                    text = request.form.get("user_prompt")
                    self.ros_backend.publish_user_prompt(text)

                # --- Evaluator prompt -----------------------------------
                elif "evaluator_prompt" in request.form:
                    text = request.form.get("evaluator_prompt")
                    self.ros_backend.publish_evaluator_prompt(text)

                # --- Zero-shot prompt -----------------------------------
                elif "zero_shot_prompt" in request.form:
                    text = request.form.get("zero_shot_prompt")
                    self.ros_backend.publish_zero_shot_prompt(text)

                # --- Image upload ---------------------------------------
                elif "upload_image" in request.form:
                    file = request.files.get("image_file")
                    if file and file.filename:
                        filename = secure_filename(file.filename)
                        path = os.path.join(self.upload_folder, filename)
                        file.save(path)
                        self.ros_backend.publish_image(path)

            return render_template("index.html")

    def run(self):
        self.app.run(host="0.0.0.0", port=5000, debug=True)

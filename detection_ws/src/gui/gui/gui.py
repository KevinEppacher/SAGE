from flask import Flask, render_template, request, redirect, url_for
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from werkzeug.utils import secure_filename
import shutil
import atexit
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time
import time
from sensor_msgs.msg import CameraInfo

rclpy.init()

def clean_upload_folder():
    folder = app.config['UPLOAD_FOLDER']
    if os.path.exists(folder):
        print(f"[CLEANUP] Lösche Inhalte in: {folder}")
        for filename in os.listdir(folder):
            file_path = os.path.join(folder, filename)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(f"Fehler beim Löschen von {file_path}: {e}")

class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_node')
        self.text_pub = self.create_publisher(String, '/user_text', 10)
        self.image_pub = self.create_publisher(Image, '/uploaded_image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/uploaded_image/camera_info', 10)
        self.bridge = CvBridge()


node = FlaskNode()
app = Flask(__name__)
UPLOAD_FOLDER = os.path.join(app.root_path, 'static', 'uploaded')
os.makedirs(UPLOAD_FOLDER, exist_ok=True)
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

clean_upload_folder()
atexit.register(clean_upload_folder)

@app.route("/", methods=["GET", "POST"])
def index():
    filename = None

    if request.method == "POST":
        # Bild-Upload
        if 'upload_image' in request.form:
            file = request.files.get("image_file")
            if file and file.filename != '':
                filename = secure_filename(file.filename)
                save_path = os.path.join(app.config['UPLOAD_FOLDER'], filename)
                file.save(save_path)
                print(f"[UPLOAD] Bild gespeichert unter: {save_path}")

                # Bild laden mit OpenCV
                cv_image = cv2.imread(save_path)  # BGR-Format

                if cv_image is not None:
                    ros_image_msg = node.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                    ros_image_msg.header.stamp = node.get_clock().now().to_msg()
                    ros_image_msg.header.frame_id = "uploaded_camera"
                    node.image_pub.publish(ros_image_msg)

                    info_msg = CameraInfo()
                    info_msg.header = ros_image_msg.header
                    info_msg.height = cv_image.shape[0]
                    info_msg.width = cv_image.shape[1]
                    info_msg.k = [1.0, 0.0, info_msg.width / 2.0,
                                0.0, 1.0, info_msg.height / 2.0,
                                0.0, 0.0, 1.0]
                    info_msg.p = [1.0, 0.0, info_msg.width / 2.0, 0.0,
                                0.0, 1.0, info_msg.height / 2.0, 0.0,
                                0.0, 0.0, 1.0, 0.0]
                    node.camera_info_pub.publish(info_msg)
                    print("[ROS2] Bild wurde als sensor_msgs/Image veröffentlicht.")
                else:
                    print("[ROS2] Fehler beim Laden des Bildes für ROS2-Publikation.")

                return render_template("index.html", uploaded_image=filename)  # ← wichtig

        # Text wurde gesendet
        elif 'text_input' in request.form:
            text = request.form.get("text_input")
            msg = String()
            msg.data = text
            node.text_pub.publish(msg)
            # optional auch Template zurückgeben

    return render_template("index.html", uploaded_image=None)


def main():
    app.run(host="0.0.0.0", port=5000, debug=True)

if __name__ == "__main__":
    main()

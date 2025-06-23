from flask import Flask, render_template, request, redirect, url_for
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from werkzeug.utils import secure_filename

rclpy.init()

class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_node')
        self.text_pub = self.create_publisher(String, '/user_text', 10)


node = FlaskNode()
app = Flask(__name__)

UPLOAD_FOLDER = os.path.join(app.root_path, 'static', 'uploaded')
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

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

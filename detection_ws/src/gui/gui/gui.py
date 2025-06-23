from flask import Flask, render_template, request
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()

class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_node')
        self.text_pub = self.create_publisher(String, '/user_text', 10)

node = FlaskNode()
app = Flask(__name__)

@app.route("/", methods=["GET", "POST"])
def index():
    if request.method == "POST":
        text = request.form.get("text_input")
        msg = String()
        msg.data = text
        node.get_logger().info(f"Received text: {text}")
        node.text_pub.publish(msg)
    return render_template("index.html")

def main():
    app.run(host="0.0.0.0", port=5000, debug=True)

if __name__ == "__main__":
    main()

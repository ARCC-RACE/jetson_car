import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Byte, Bool, Float64, String # for mode reading and file input

import numpy as np
import cv2, os, sys, yaml, importlib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# stuff for NN
import keras
from keras.models import load_model
from keras import backend as K
import tensorflow.compat.v1 as tf
tf.disable_v2_behavior()
from tensorflow.python.compiler.tensorrt import trt_convert as tftrt
import copy

#################################################################
# Code found here https://github.com/jeng1220/KerasToTensorRT
# For converting keras to TensorRT
class FrozenGraph(object):
  def __init__(self, model, shape):
    shape = (None, shape[0], shape[1], shape[2])
    x_name = 'image_tensor_x'
    with tf.keras.backend.get_session() as sess:
        x_tensor = tf.placeholder(tf.float32, shape, x_name)
        K.set_learning_phase(0)
        y_tensor = model(x_tensor)
        y_name = y_tensor.name[:-2]
        graph = sess.graph.as_graph_def()
        graph0 = tf.graph_util.convert_variables_to_constants(sess, graph, [y_name])
        graph1 = tf.graph_util.remove_training_nodes(graph0)

    self.x_name = [x_name]
    self.y_name = [y_name]
    self.frozen = graph1

class TfEngine(object):
  def __init__(self, graph):
    g = tf.Graph()
    with g.as_default():
      x_op, y_op = tf.import_graph_def(graph_def=graph.frozen, return_elements=graph.x_name + graph.y_name)
      self.x_tensor = x_op.outputs[0]
      self.y_tensor = y_op.outputs[0]

    config = tf.ConfigProto(gpu_options=
      tf.GPUOptions(per_process_gpu_memory_fraction=0.5,
      allow_growth=True))

    self.sess = tf.Session(graph=g, config=config)

  def infer(self, x):
    y = self.sess.run(self.y_tensor, feed_dict={self.x_tensor: x})
    return y

class TftrtEngine(TfEngine):
  def __init__(self, graph, batch_size, precision):
    tftrt_graph = tftrt.create_inference_graph(
      graph.frozen,
      outputs=graph.y_name,
      max_batch_size=batch_size,
      max_workspace_size_bytes=1 << 30,
      precision_mode=precision,
      minimum_segment_size=2)

    opt_graph = copy.deepcopy(graph)
    opt_graph.frozen = tftrt_graph
    super(TftrtEngine, self).__init__(opt_graph)
    self.batch_size = batch_size

#################################################################

def constrain(x, min, max):
    if x < min:
        return min
    elif x > max:
        return max
    else:
        return x

class ModelLoader(Node):
    def __init__(self):
        super().__init__('model_loader')
        self.get_logger().info("Model loader starting...")

        self.bridge = CvBridge()

        self.declare_parameter("absolute_max_speed", value=0.5)
        self.declare_parameter("ai_ackermann_drive_topic", value="/ai/cmd")
        self.declare_parameter("model_path", value="model") #include h5, and util functions for loading and sending data from net
        self.declare_parameter("triple_camera_mode", value=False)
        self.declare_parameter("center_camera_topic", value="/camera/color/image_raw")
        self.declare_parameter("center_camera_depth_topic", value="/camera/depth/image_rect_raw")

        self.update_timer = None

        self.create_subscription(Image, self.get_parameter("center_camera_topic").value, self.new_color_image_cb, 1)
        self.create_subscription(Image, self.get_parameter("center_camera_depth_topic").value, self.new_depth_image_cb, 1)
        self.create_subscription(String, self.get_parameter("model_path").value, self.new_model_cb, 10)

        self.ackermann_cmd_publisher = self.create_publisher(AckermannDriveStamped, self.get_parameter("ai_ackermann_drive_topic").value, 1)

        self.loaded_model = None
        self.tftrt_engine = None
        self.package_name = None
        self.utils = None # holds functions for preprocessing and postprocessing data for AI
        self.last_color_image = None
        self.last_depth_image = None
        self.recursion_factor = None
        # self.last_imu_data = None
        # self.last_imu_status = None

    def new_model_cb(self, path):
        model_path = os.path.join(path.data, "model.h5")
        config_path = os.path.join(path.data, "config.yaml")
        self.package_name = os.path.basename(path.data)

        if os.path.exists(model_path):

            if self.loaded_model is not None:
                self.loaded_model = None  # stop any more model processing if a previous model was in use
                sys.path[-1] = os.path.dirname(path.data)  # allows us to change import utils
            else:
                sys.path.append(os.path.dirname(path.data))  # allows us to import utils

            self.recursion_factor = None
            self.utils = None
            config_file = yaml.load(open(config_path))
            self.get_logger().info("Loading Model")
            self.loaded_model = load_model(model_path)
            self.get_logger().info("Model Loaded")

            if self.update_timer is not None:
                self.update_timer.cancel()

            update_rate = float(config_file["update_rate"])
            self.update_timer = self.create_timer(1.0 / update_rate, self.update_cb)
        else:
            self.get_logger().error("Model not found at " + model_path)

    def new_color_image_cb(self, image):
        try:
            self.last_color_image = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            self.get_logger().error(e)

    def new_depth_image_cb(self, image):
        try:
            self.last_depth_image = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError as e:
            self.get_logger().error(e)

    def update_cb(self):
        if not (self.loaded_model is None and self.last_depth_image is None and self.last_color_image is None):
            if self.utils is None or self.tftrt_engine is None:
                self.utils = importlib.import_module(".utils", self.package_name)
                frozen_graph = FrozenGraph(self.loaded_model, (self.utils.IMAGE_HEIGHT, self.utils.IMAGE_WIDTH, self.utils.IMAGE_CHANNELS))
                self.tftrt_engine = TftrtEngine(frozen_graph, 1, 'FP32')

            # preprocess_data will take in images and IMU and return array to make prediction (input to network) and a recursion factor
                # recursion factor starts as None. Make sure None case is handled
            # postprocess data will take in output of network and return steering, throttle
            x, self.recursion_factor = self.utils.preprocess_data(last_color_image=self.last_color_image, last_depth_image=self.last_depth_image, recursion_factor=self.recursion_factor)
            # command = self.loaded_model.predict(x, batch_size=1) # Keras version
            command = self.tftrt_engine.infer(x) #TensorRT version
            steer, throttle = self.utils.postprocess_data(command)

            ai_cmd = AckermannDriveStamped()
            # print(self.get_clock().now())
            # hw_cmd.header.stamp = self.get_clock().now()
            ai_cmd.drive.steering_angle = constrain(steer, -1.0, 1.0)
            ai_cmd.drive.speed = constrain(throttle, -1.0, 1.0)
            self.ackermann_cmd_publisher.publish(ai_cmd)

def main(args=None):
    rclpy.init(args=args)
    model_loader = ModelLoader()
    rclpy.spin(model_loader)
    model_loader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

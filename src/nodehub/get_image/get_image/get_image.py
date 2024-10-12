import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2 as cv
import uuid
import os
import numpy as np
import datetime
import time
from sensor_msgs.msg import Image


class ImageSubscriber(Node):

  def __init__(self):
    super().__init__('ImageSubscriber')
    # 创建sub节点，订阅image_raw话题
    self.subscription = self.create_subscription(
      Image,
      '/zed_camera_left_sensor/image_raw',
      self.listener_callback,
      1)
    self.declare_parameter("fps",2.0)
    self.declare_parameter("line_follow_image_folder","line_follow_image")
    self.declare_parameter("detection_image_folder","detection_image")
    # 创建CvBridge实例
    self.bridge = CvBridge()
    self.uuid = -1
    self.line_follow_image = np.zeros((960, 224, 3))
    self.detection_image = np.zeros((960, 544, 3))
    self.start_time = time.time()
    self.time_diff_threshold = 1 / self.get_parameter('fps').get_parameter_value().double_value

    self.line_follow_image_folder = self.get_parameter('line_follow_image_folder').get_parameter_value().string_value
    self.detection_image_folder = self.get_parameter('detection_image_folder').get_parameter_value().string_value
    # 检查用户存放标定数据的image_dataset文件夹是否存在，如果不存在则创建
    if not os.path.exists(self.line_follow_image_folder):
      os.makedirs(self.line_follow_image_folder)
    if not os.path.exists(self.detection_image_folder):
      os.makedirs(self.detection_image_folder)
    # 设置opecv展示窗属性

    self.subscription
  


  # sub回调函数
  def listener_callback(self, msg):
    end_time = time.time()
    time_diff = end_time - self.start_time
    if time_diff > self.time_diff_threshold:
      self.start_time = end_time
      self.detection_image = self.bridge.imgmsg_to_cv2(msg)
      self.line_follow_image = self.detection_image[160:384,:,:].copy()
      self.uuid = uuid.uuid1()
      # 保存上一次标定的结果
      cv.imwrite(os.path.join(self.line_follow_image_folder, str(self.uuid) + '.jpg'), self.line_follow_image)
      cv.imwrite(os.path.join(self.detection_image_folder, str(self.uuid) + '.jpg'), self.detection_image)
      print("Save"+ str(self.uuid) + ".jpg")
      # 载入新的图像


def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
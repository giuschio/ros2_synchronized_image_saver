import rclpy

from os.path import join as pathjoin
from pathlib import Path
from rclpy.node import Node

from message_filters import TimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from PIL import Image as PILImage

from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImgSaver:
    def __init__(self, outpath, rgb=True) -> None:
        self.outpath = outpath
        self.rgb = rgb
        self.counter = 0
        Path(self.outpath).mkdir(parents=True, exist_ok=True)
        self.cv_bridge = CvBridge()

    def __call__(self, img_msg) -> None:
        s1 = f'{self.counter:05d}'
        fname = pathjoin(self.outpath, "image_"+s1+".png")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.cv_bridge.imgmsg_to_cv2(img_msg)
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            imshape = cv2_img.shape
            if len(imshape) > 2:
                # if rgb
                # bgr -> rgb encoding
                cv2_img = cv2_img[:, :, [2, 1, 0, 3]]
            img_pil = PILImage.fromarray(cv2_img)
            img_pil.save(fname)

        self.counter += 1


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('synchronized_image_saver')
        self.ok = True
        self.declare_parameter('output_path', "")
        output_path = self.get_parameter('output_path').get_parameter_value().string_value
        if len(output_path) < 1:
            self.ok = False
            print("The output_path was not set and the node cannot start")

        self.camera_0_saver = ImgSaver(pathjoin(output_path, "camera_0"), rgb=True)
        self.camera_1_saver = ImgSaver(pathjoin(output_path, "camera_1"), rgb=True)

        self.camera_0_sub = Subscriber(self, Image, "image0")
        self.camera_1_sub = Subscriber(self, Image, "image1")

        tss = TimeSynchronizer([self.camera_0_sub, self.camera_1_sub], queue_size=10)
        tss.registerCallback(self.m_callback)

    def m_callback(self, image0_mgs, image1_msg):
        self.camera_0_saver(image0_mgs)
        self.camera_1_saver(image1_msg)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    if minimal_subscriber.ok:
        rclpy.spin(minimal_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()
    else:
        print("There was a problem with initialization and the node could not start.")


if __name__ == '__main__':
    main()

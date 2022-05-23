#!/usr/bin/env python3

# 必要なライブラリをインポート
import rospy
from opencv_apps.msg import FaceArrayStamped


class FaceSubscriber(object):
    def __init__(self):
        # ROSの設定
        rospy.Subscriber("/face_detection_node/faces", FaceArrayStamped, self.facesCB)
        rospy.spin()

    def facesCB(self, msg):
        # 顔情報が含まれているか確認
        if len(msg.faces) > 0:
            face_data = msg.faces[0].face
            print("顔の位置情報")
            print(" x: {}".format((320 / 2) - face_data.x))
            print(" y: {}".format((240 / 2) - face_data.y))
            print("-------------------------------")

if __name__ == '__main__':
    # ノードを宣言
    rospy.init_node("face_subscriber")
    fu = FaceSubscriber()

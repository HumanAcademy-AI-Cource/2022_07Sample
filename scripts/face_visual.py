#!/usr/bin/env python3

# 必要なライブラリをインポート
import rospy
from opencv_apps.msg import FaceArrayStamped
from visualization_msgs.msg import Marker

class FaceVisualization(object):
    def __init__(self):
        # ROSの設定
        self.pub = rospy.Publisher("face_block", Marker, queue_size = 10)
        rospy.Subscriber("/face_detection_node/faces", FaceArrayStamped, self.facesCB)
        rospy.spin()

    def facesCB(self, msg):
        # 顔情報が含まれているか確認
        if len(msg.faces) > 0:
            # 顔の位置情報を取り出す
            face_data = msg.faces[0].face

            # 顔の位置情報からRvizで表示するための設定を行う
            face_marker = Marker()
            face_marker.header.frame_id = "map"
            face_marker.header.stamp = rospy.Time.now()
            face_marker.lifetime = rospy.Duration()
            face_marker.type = 1

            face_marker.ns = "face_shapes"
            face_marker.id = 0
            face_marker.action = Marker.ADD
            
            # マーカーの位置情報を設定
            face_marker.pose.position.x = ((320 / 2) - face_data.x) * 0.02
            face_marker.pose.position.y = 0.0
            face_marker.pose.position.z = ((240 / 2) - face_data.y) * 0.02
            face_marker.pose.orientation.x=0.0
            face_marker.pose.orientation.y=0.0
            face_marker.pose.orientation.z=1.0
            face_marker.pose.orientation.w=0.0

            # マーカーの色情報を設定
            face_marker.color.r = 1.0
            face_marker.color.g = 0.647
            face_marker.color.b = 0.0
            face_marker.color.a = 1.0

            # マーカーの大きさを設定
            face_marker.scale.x = 1
            face_marker.scale.y = 1
            face_marker.scale.z = 1

            # マーカーをパブリッシュする
            self.pub.publish(face_marker)

if __name__ == '__main__':
    # ノードを宣言
    rospy.init_node("face_visualization")
    fv = FaceVisualization()

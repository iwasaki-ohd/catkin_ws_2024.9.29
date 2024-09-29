#!/usr/bin/env python

# 2024.4.1
# 2024.4.8
# 2024.4.11
# calculate_arc.pyの改良版
# rightShoulderとleftShoulderのキーポイントのみを表示する
# 角度をパブリッシュ7(unity用)
# キーポイントと人物IDをマッチングさせる
# 複数人の方向取得

import rospy
from sensor_msgs.msg import Image
from ros_posenet.msg import Poses
from cv_bridge import CvBridge
import cv2
import math
from std_msgs.msg import String  # 追加
from darknet_ros_msgs.msg import BoundingBoxes  # 追加

class PoseVisualizer:
    def __init__(self, line_color=(0, 255, 0), line_thickness=2):
        rospy.init_node('pose_visualizer')
        self.bridge = CvBridge()
        self.image = None  # イメージを初期化
        self.line_color = line_color  # 線の色を指定
        self.line_thickness = line_thickness  # 線の太さを指定
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pose_sub = rospy.Subscriber('/ros_posenet/result', Poses, self.pose_callback)
        self.bb_sub = rospy.Subscriber('/pc1/iou_tracker/bounding_boxes', BoundingBoxes, self.bb_callback)  # 追加
        self.angle_pub = rospy.Publisher('/shoulder_angle', String, queue_size=10)  # 追加
        self.bb_data = None  # 追加
        self.pub_data = ""  # 追加

    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def calculate_shoulder_angle(self, right_shoulder_pos, left_shoulder_pos):
        # 肩の角度計算
        angle_radian = math.atan2(left_shoulder_pos[1] - right_shoulder_pos[1], left_shoulder_pos[0] - right_shoulder_pos[0])
        angle_degree = math.degrees(angle_radian)
        return angle_degree

    def bb_callback(self, data):
        self.bb_data = data

    def pose_callback(self, data):
        self.pub_data = "" # 初期化
        if self.image is None or self.bb_data is None:
            return  # イメージまたはバウンディングボックスがまだ受信されていない場合は何もしない
        mached_keypoints_list_image = []  # 画像座標
        mached_keypoints_list_camera = []  # カメラ座標
        mached_keypoints_list_name = []  # マッチしたキーポイントの名前
        for pose in data.poses:
            for keypoint in pose.keypoints:
                part_name = keypoint.part

                # 画像座標
                x = int(keypoint.image_position.x)
                y = int(keypoint.image_position.y)
                # カメラ座標
                cam_x = keypoint.position.x
                cam_z = keypoint.position.z
                
                if part_name == "rightShoulder" or part_name == "leftShoulder":
                    # バウンディングボックスの領域内にキーポイントがあった場合、IDと紐づけ
                    for bb in self.bb_data.bounding_boxes:
                        if bb.xmin <= x <= bb.xmax and bb.ymin <= y <= bb.ymax:
                            part_name = str(bb.id) + '_' + part_name  # IDをキーポイント名に追加
                            rospy.loginfo(f"確認：{part_name}")
                            # マッチしたキーポイント情報をリストに追加
                            mached_keypoints_list_image.append((x, y))
                            mached_keypoints_list_camera.append((cam_x, cam_z))
                            mached_keypoints_list_name.append(part_name)
                            break  # 一度バウンディングボックスにマッチしたら、ループを抜ける

        number = 0
        check_list=[] # 重複回避用リスト。同じIDの角度が複数回パブリッシュされるのを防ぐ。
        for i in range(len(mached_keypoints_list_name) -1):
            num_1, char_1 = mached_keypoints_list_name[i].split('_')
            for j in range(i+1, len(mached_keypoints_list_name)):
                num_2, char_2 = mached_keypoints_list_name[j].split('_')
                # それぞれが右肩と左肩であるかつIDが同じ場合
                if (char_1 == "rightShoulder" and char_2 == "leftShoulder") or (char_1 == "leftShoulder" and char_2 == "rightShoulder"):
                    if num_1 == num_2:
                        # 角度計算
                        if char_1 == "rightShoulder" and char_2 == "leftShoulder":
                            shoulder_angle = int(self.calculate_shoulder_angle(mached_keypoints_list_camera[i], mached_keypoints_list_camera[j]))
                        elif char_1 == "leftShoulder" and char_2 == "rightShoulder":
                            shoulder_angle = int(self.calculate_shoulder_angle(mached_keypoints_list_camera[j], mached_keypoints_list_camera[i]))
                        if shoulder_angle < 0:
                            shoulder_angle = 360 + shoulder_angle
                        #angle_str = 'Shoulder Angle: {:.0f} degrees'.format(shoulder_angle)
                        number +=1
                        check_id = int(num_1)
                        if check_id not in check_list: # 重複回避用条件文
                            check_list.append(check_id)
                            angle_str = f"Shoulder Angle_{num_1}: {shoulder_angle} degrees"
                            #cv2.putText(self.image, angle_str, ((mached_keypoints_list_image[i][0] + mached_keypoints_list_image[j][0])//2, (mached_keypoints_list_image[i][1] + mached_keypoints_list_image[j][1])//2-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 1, cv2.LINE_AA)
                            cv2.putText(self.image, angle_str, (20, 20*number), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 1, cv2.LINE_AA)

                            # cv2で描画(円と線)
                            cv2.circle(self.image, mached_keypoints_list_image[i], 5, (0, 255, 0), -1)
                            cv2.circle(self.image, mached_keypoints_list_image[j], 5, (0, 255, 0), -1)
                            text_offset_y = -20  # テキストを上に移動するオフセット
                            cv2.putText(self.image, mached_keypoints_list_name[i], (mached_keypoints_list_image[i][0], mached_keypoints_list_image[i][1] + text_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                            cv2.putText(self.image, mached_keypoints_list_name[j], (mached_keypoints_list_image[j][0], mached_keypoints_list_image[j][1] + text_offset_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                            cv2.line(self.image, mached_keypoints_list_image[i], mached_keypoints_list_image[j], self.line_color, self.line_thickness)
                            # pub_dataに角度とIDを追加
                            self.pub_data += f"{num_1}_{shoulder_angle}, "
                            break
        
        cv2.imshow("Pose Visualization", self.image)
        cv2.waitKey(1)

        # パブリッシュ
        if self.pub_data !="":
            self.angle_pub.publish(self.pub_data)
            rospy.loginfo(f"publish: {self.pub_data}")
        rospy.loginfo(f"------------------------------------------------")

if __name__ == '__main__':
    try:
        # ユーザーが線の色と太さを指定できるようにする
        line_color = (255, 0, 0)  # 色
        line_thickness = 3  # 太さ
        visualizer = PoseVisualizer(line_color=line_color, line_thickness=line_thickness)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

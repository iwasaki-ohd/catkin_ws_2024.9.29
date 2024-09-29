#!/usr/bin/env python
# coding: UTF-8

# 2024.9.27
# key_maching.py + pc1_pub_TrackingData
# 世界座標の計算
# 世界座標から角度を計算
# パブリッシュ
# 距離もパブリッシュ

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String, Float32MultiArray
import numpy as np
from datetime import datetime
from ros_posenet.msg import Poses
import math
import copy

kaiten = -70

class PoseVisualizer:
    def __init__(self, line_color=(0, 255, 0), line_thickness=2):
        rospy.init_node("image_subscriber_pc1", anonymous=True)
        self.bridge = CvBridge()
        self.image = None  # イメージを初期化
        self.line_color = line_color  # 線の色を指定
        self.line_thickness = line_thickness  # 線の太さを指定
        # 世界座標計算用
        self.depth_image = None
        self.bounding_boxes_list = []
        self.around = 3 # 距離データの取得範囲
        self.A_cord = np.array([[1.0], [1.0], [1.0], [1.0]]) # 世界座標計算用配列
        # 外部パラメータ行列
        # 半自動で取得したパラメータを使用中(2024.9.23)
        self.initial_parameters = [
            [-1.83697020e-16,  0.00000000e+00, -1.00000000e+00, -1.0],
            [ 9.87688341e-01, -1.56434465e-01, -1.81435405e-16, 1.0],
            [-1.56434465e-01, -9.87688341e-01,  2.87365450e-17, -1.2],
            [ 0.        , 0.         , 0.         , 1.        ]
        ]
        self.extrinsic_parameters = [
            [ 0.81490274,  0.19735933,  0.54496129, -0.57892849],
            [-0.34787722,  0.91859486,  0.18752311, -0.01730745],
            [-0.46358921, -0.34239272,  0.81722229,  0.26584073],
            [ 0.        , 0.         , 0.         , 1.        ]
        ]
        """
        外部パラメータのストック
        <cam1>
        新
        床+壁+ロボ(28点)
        [
            [ 1.07086418e+00, -1.47182142e-02,  4.89235595e-02,  3.54098613e-01],
            [-6.19389769e-02, -4.88021228e-01,  1.02788817e+00, -1.80272089e+00],
            [ 3.00941578e-02, -9.63769763e-01, -4.44642576e-01,  1.92587304e+00],
            [ 2.77555756e-17,  2.49800181e-16, -6.24500451e-17,  1.00000000e+00]
        ]

        <cam2>
        新
        床+ロボ(17点)
        [
            [-2.42738287e-01,  4.05191340e-01, -9.35531480e-01,  2.86083890e+00],
            [ 1.00179605e+00,  6.38568567e-02, -2.50219012e-01,  3.16191196e+00],
            [-2.89507704e-02, -9.43108670e-01, -4.30104576e-01,  2.00400042e+00],
            [ 5.13478149e-16,  2.74086309e-16,  3.33066907e-16,  1.00000000e+00]
        ]
        """
        self.K_fx = 0
        self.K_fy = 0
        self.K_cx = 0
        self.K_cy = 0
        self.image_w = 0
        self.image_h = 0

        # 世界座標関係
        self.multiple_data_pub = rospy.Publisher("multiple_data", String, queue_size=10)# 複数のデータ(ID, 世界座標)を文字列でパブリッシュするパブリッシャ(Unityに送るデータ)
        self.all_data_pub = rospy.Publisher("all_data_pc1", String, queue_size=10) # 統合用データ(時刻, ID, 世界座標)
        self.world_point_pub = rospy.Publisher("world_point", Float32MultiArray, queue_size=10)

        self.camera_coordinate_pub = rospy.Publisher("camera_coordinate", Float32MultiArray, queue_size=10)  # 追加

        self.camera_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info",CameraInfo,self.camera_info_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)
        ###self.bounding_boxes_data_sub = rospy.Subscriber("/pc1/iou_tracker/bounding_boxes",BoundingBoxes,self.bounding_boxes_data_callback)
        # posenet関係
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pose_sub = rospy.Subscriber('/ros_posenet/result', Poses, self.pose_callback)
        self.bb_sub = rospy.Subscriber('/pc1/iou_tracker/bounding_boxes', BoundingBoxes, self.bb_callback)  # 追加
        self.angle_pub = rospy.Publisher('/shoulder_angle', String, queue_size=10)  # 追加
        self.bb_data = None 
        self.pub_data = ""  

    # カラー画像データの取得
    def image_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    # bboxデータの取得
    def bb_callback(self, data):
        self.bb_data = data
    
    # 肩の角度計算
    def calculate_shoulder_angle(self, right_shoulder_pos, left_shoulder_pos):
        angle_radian = math.atan2(left_shoulder_pos[1] - right_shoulder_pos[1], left_shoulder_pos[0] - right_shoulder_pos[0])
        angle_degree = math.degrees(angle_radian)
        return angle_degree

    # デプス画像を表示
    def depth_image_callback(self, data):
        CLmap = None
        self.depth_image = self.bridge.imgmsg_to_cv2(
            data, desired_encoding="passthrough"
        )

        if self.depth_image is not None:
            CLmap = cv2.applyColorMap(
                cv2.convertScaleAbs(self.depth_image, alpha=0.08), cv2.COLORMAP_JET
            )

        for person_box in self.bounding_boxes_list:
            if person_box.Class == "person":
                x_center = int((person_box.xmax + person_box.xmin) / 2)
                y_center = int((person_box.ymax + person_box.ymin) / 2)
                cv2.circle(
                    CLmap,
                    (x_center, y_center),
                    radius=self.around,
                    color=(255, 255, 255),
                    thickness=10,
                )

        cv2.imshow("Depth", CLmap)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("User initiated shutdown")


    # 世界座標の計算+方向の計算
    def pose_callback(self, data):
        # 時刻を取得
        timestamp = rospy.get_time()
        current_time = datetime.fromtimestamp(timestamp) # 日付と時刻
        if self.depth_image is None or self.image is None or self.bb_data is None:
            # rospy.loginfo("depth_image is None. Skipping bounding_boxes_callback.")
            rospy.loginfo("Something is None. Skipping pose_callback.")
            return
        
        self.bounding_boxes_list = self.bb_data.bounding_boxes
        rospy.loginfo("検出された人数: {} [人] ".format(len(self.bounding_boxes_list)))

        #------------------角度計算----------------------------
        self.pub_data = "" # 初期化
        mached_keypoints_list_image = []  # 画像座標
        mached_keypoints_list_world = []  # カメラ座標
        mached_keypoints_list_name = []  # マッチしたキーポイントの名前
        for pose in data.poses:
            for keypoint in pose.keypoints:
                part_name = keypoint.part

                # 画像座標
                x = int(keypoint.image_position.x)
                y = int(keypoint.image_position.y)
                # カメラ座標
                cam_x = keypoint.position.x
                cam_y = keypoint.position.y
                cam_z = keypoint.position.z
                
                if part_name == "rightShoulder" or part_name == "leftShoulder":
                    # バウンディングボックスの領域内にキーポイントがあった場合、IDと紐づけ
                    for bb in self.bb_data.bounding_boxes:
                        if bb.xmin <= x <= bb.xmax and bb.ymin <= y <= bb.ymax:
                            part_name = str(bb.id) + '_' + part_name  # IDをキーポイント名に追加
                            rospy.loginfo(f"確認：{part_name}")
                            # マッチしたキーポイント情報をリストに追加
                            # -----------------------------------------------------------------ここで世界座標計算(4.22)
                            # カメラ座標→マップ座標→世界座標への変換(2024.9.29)
                            camera_coordinate = [[cam_x, cam_y, cam_z]]
                            X_cord = copy.deepcopy(self.A_cord)
                            X_cord[0:3, 0] = camera_coordinate[0]
                            # マップ座標の追加(2024.9.25)
                            initial_coordinates = np.dot(self.initial_parameters, np.array(X_cord))
                            map3d_coordinate = (np.dot(self.extrinsic_parameters, initial_coordinates)[0:3]).T
                            world_coordinate = copy.deepcopy(map3d_coordinate)# コピー元を参照しない
                            #rospy.loginfo(f"世界座標調整前(マップ座標)：{world_coordinate}")
                            #原点のズレと回転を考慮
                            map3d_coordinate[0][0] += 0.4588*5.3
                            map3d_coordinate[0][1] += -0.4588*2.2
                            #rospy.loginfo("反映してる！！！！！！！！！！！！！")
                            #rospy.loginfo(f"世界座標調整前２(マップ座標)：{world_coordinate}")
                            world_coordinate[0][0] = map3d_coordinate[0][0]*math.cos(math.radians(kaiten)) - map3d_coordinate[0][1]*math.sin(math.radians(kaiten))
                            world_coordinate[0][1] = map3d_coordinate[0][0]*math.sin(math.radians(kaiten)) + map3d_coordinate[0][1]*math.cos(math.radians(kaiten))


                            rospy.loginfo("キーポイントの世界座標(x, y, z) =  ({}, {}, {})".format(world_coordinate[0][0],world_coordinate[0][1],world_coordinate[0][2]))
                            world_point = [
                            world_coordinate[0][0],
                            world_coordinate[0][1],
                            world_coordinate[0][2],
                            ]
                            #--------------------------------------------------------------------------------------------
                            mached_keypoints_list_image.append((x, y))
                            mached_keypoints_list_world.append((world_point[0], world_point[2]))
                            mached_keypoints_list_name.append(part_name)
                            break  # 一度バウンディングボックスにマッチしたら、ループを抜ける

        number = 0
        check_list=[] # 重複回避用リスト。同じIDの角度が複数回パブリッシュされるのを防ぐ。
        arc_dict={} # IDと角度をセットで保持する辞書
        for i in range(len(mached_keypoints_list_name) -1):
            num_1, char_1 = mached_keypoints_list_name[i].split('_')
            for j in range(i+1, len(mached_keypoints_list_name)):
                num_2, char_2 = mached_keypoints_list_name[j].split('_')
                # それぞれが右肩と左肩であるかつIDが同じ場合
                if (char_1 == "rightShoulder" and char_2 == "leftShoulder") or (char_1 == "leftShoulder" and char_2 == "rightShoulder"):
                    if num_1 == num_2:
                        # 角度計算
                        if char_1 == "rightShoulder" and char_2 == "leftShoulder":
                            shoulder_angle = int(self.calculate_shoulder_angle(mached_keypoints_list_world[j], mached_keypoints_list_world[i]))
                        elif char_1 == "leftShoulder" and char_2 == "rightShoulder":
                            shoulder_angle = int(self.calculate_shoulder_angle(mached_keypoints_list_world[i], mached_keypoints_list_world[j]))
                        if shoulder_angle < 0:
                            shoulder_angle = 360 + shoulder_angle
                        #angle_str = 'Shoulder Angle: {:.0f} degrees'.format(shoulder_angle)
                        number +=1
                        check_id = int(num_1)
                        if check_id not in check_list: # 重複回避用条件文
                            check_list.append(check_id)
                            arc_dict[num_1] = shoulder_angle # 結果を保持
                            rospy.loginfo(f"辞書：{arc_dict}") # デバック用
                            #rospy.loginfo(f"リスト{check_list}") # デバック用
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
        
        #cv2.imshow("Pose Visualization", self.image)
        #cv2.waitKey(1)
        # パブリッシュ
        if self.pub_data !="":
            self.angle_pub.publish(self.pub_data)
            rospy.loginfo(f"publish: {self.pub_data}")
        #rospy.loginfo(f"------------------------------------------------")
        #------------------------------------------------------


        count = 0
        multiple_data = ""
        for person_box in self.bounding_boxes_list:
            count += 1
            if person_box.Class == "person" and person_box.id is not None: # クラスがperson、かつ、idを持つ場合に処理開始
                x_center = int((person_box.xmax + person_box.xmin) / 2)
                y_center = int((person_box.ymax + person_box.ymin) / 2)

                #if 10 < x_center < 630 and 10 < y_center < 470:
                if 10 < x_center < 1270 and 10 < y_center < 710:

                    rospy.loginfo(
                        "[{}]\n　　　　　　　　　　　　　　オブジェクトID:  {} \n　　　　　　　　　　　　　　bboxの中央座標(x, y): ({}, {})".format(
                            count, person_box.id, x_center, y_center
                        )
                    )

                    depth_values = self.depth_image[
                        y_center - self.around : y_center + self.around,
                        x_center - self.around : x_center + self.around,
                    ]

                    average_depth = np.mean(depth_values) / 1000

                    formatted_average_depth = "{:.5f}".format(average_depth)
                    rospy.loginfo(
                        "カメラとの平均距離: {} [m] ".format(float(formatted_average_depth))
                    )

                    coordinate3D_x = (
                        (x_center - self.image_w / 2) * average_depth / self.K_fx
                    )
                    coordinate3D_y = (
                        (y_center - self.image_h / 2) * average_depth / self.K_fy
                    )

                    rospy.loginfo(
                        "カメラ座標(x, y, z) =  ({}, {}, {})".format(
                            coordinate3D_x, coordinate3D_y, average_depth
                        )
                    )
                    # Publish camera coordinates
                    camera_coord_msg = Float32MultiArray(data=[coordinate3D_x, coordinate3D_y, average_depth])
                    self.camera_coordinate_pub.publish(camera_coord_msg)
                    rospy.loginfo(
                        "カメラ座標をパブリッシュ"
                    )

                    # カメラ座標→マップ座標→世界座標への変換(2024.9.29)
                    camera_coordinate = [[coordinate3D_x, coordinate3D_y, average_depth]]
                    self.A_cord[0:3, 0] = camera_coordinate[0]
                    initial_coordinates = np.dot(self.initial_parameters, np.array(self.A_cord))
                    map3d_coordinate = (np.dot(self.extrinsic_parameters, initial_coordinates)[0:3]).T
                    world_coordinate = copy.deepcopy(map3d_coordinate)# コピー元を参照しない
                    rospy.loginfo(f"世界座標調整前(マップ座標)：{world_coordinate}")
                    #原点のズレと回転を考慮
                    map3d_coordinate[0][0] += 0.4588*5.3
                    map3d_coordinate[0][1] += -0.4588*2.2
                    #rospy.loginfo("反映してる！！！！！！！！！！！！！")
                    #rospy.loginfo(f"世界座標調整前２(マップ座標)：{world_coordinate}")
                    world_coordinate[0][0] = map3d_coordinate[0][0]*math.cos(math.radians(kaiten)) - map3d_coordinate[0][1]*math.sin(math.radians(kaiten))
                    world_coordinate[0][1] = map3d_coordinate[0][0]*math.sin(math.radians(kaiten)) + map3d_coordinate[0][1]*math.cos(math.radians(kaiten))
                    
                    rospy.loginfo(f"世界座標調整後")
                    rospy.loginfo(
                        "世界座標(x, y, z) =  ({}, {}, {})".format(
                            world_coordinate[0][0],
                            world_coordinate[0][1],
                            world_coordinate[0][2],
                        )
                    )

                    world_point = [
                        world_coordinate[0][0],
                        world_coordinate[0][1],
                        world_coordinate[0][2],
                    ]
                    world_point_msg = Float32MultiArray(data=world_point)
                    self.world_point_pub.publish(world_point_msg)

                    formatted_world_coordinate_x = "{:.5f}".format(world_coordinate[0][0])
                    formatted_world_coordinate_y = "{:.5f}".format(world_coordinate[0][1])

                    if count == 1:
                        multiple_data += (
                            str(person_box.id)
                            + (",")
                            + str(formatted_world_coordinate_x)
                            + (",")
                            + str(formatted_world_coordinate_y)
                        )
                        rospy.loginfo(f"チェック対象：{person_box.id}, 辞書：{arc_dict.keys()}")
                        if str(person_box.id) in arc_dict.keys():
                            #rospy.loginfo("1----通った！！！！！！！！！！！！！")
                            multiple_data += (",") + str(arc_dict[str(person_box.id)]) + (",") + str(formatted_average_depth)
                        else:
                            multiple_data += (",") + str("-1") + (",") + str(formatted_average_depth)
                    else:
                        multiple_data += (
                            (",")
                            + str(person_box.id)
                            + (",")
                            + str(formatted_world_coordinate_x)
                            + (",")
                            + str(formatted_world_coordinate_y)
                        )
                        if str(person_box.id) in arc_dict.keys():
                            #rospy.loginfo("2----通った！！！！！！！！！！！！！")
                            multiple_data += (",") + str(arc_dict[str(person_box.id)]) + (",") + str(formatted_average_depth)
                        else:
                            multiple_data += (",") + str("-1") + (",") + str(formatted_average_depth)

        all_data = f"{current_time}" + (",") + multiple_data  # 日時を含む送信用データ
        if len(multiple_data) > 2:
            self.multiple_data_pub.publish(multiple_data)
            self.all_data_pub.publish(all_data)
            rospy.loginfo("データをパブリッシュ: " + (multiple_data) + "\n ")
            rospy.loginfo("統合用データをパブリッシュ: " + (all_data) + "\n ")  # 確認用

        rospy.loginfo("----------------------------------------")

    # 世界座標の計算に必要な内部パラメータを取得する
    def camera_info_callback(self, msg):
        K = msg.K
        self.K_fx = K[0]
        self.K_fy = K[4]
        self.K_cx = K[2]
        self.K_cy = K[5]
        self.image_w = msg.width
        self.image_h = msg.height

    # -----------------------------------------↑csv_world_coordinate.py-------------------------------------------------------------

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    image_subscriber = PoseVisualizer()
    try:
        image_subscriber.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node killed!")
        pass

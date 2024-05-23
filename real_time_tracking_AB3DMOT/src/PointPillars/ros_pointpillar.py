import rospy
import numpy as np
import torch
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import message_filters  # for synchronizing messages
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
import time


from model import PointPillars  # make sure to import your model correctly
from utils import setup_seed, read_points, read_calib, read_label, \
    keep_bbox_from_image_range, keep_bbox_from_lidar_range, vis_pc, \
    vis_img_3d, bbox3d2corners_camera, points_camera2image, \
    bbox_camera2lidar,bbox3d2corners



class PointPillarNode:
    def __init__(self):
        rospy.init_node('point_pillar_node', anonymous=True)
        self.marker_pub = rospy.Publisher('/bounding_boxes', MarkerArray, queue_size=1)
        self.pc_pub = rospy.Publisher("lidar/points_pillar", PointCloud2, queue_size=1)
        # Parameters
        print(torch.cuda.is_available())
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        checkpoint_path = "pretrained/epoch_160.pth"

        # Model setup
        self.model = PointPillars(nclasses=3)  # Adjust number of classes as necessary
        self.model.load_state_dict(torch.load(checkpoint_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()

        # Subscribers
        self.pc_sub = rospy.Subscriber("/lidar/points", PointCloud2, self.pc_callback, queue_size=1)

        # Publishers
        # self.result_pub = rospy.Publisher('/detections', SomeCustomMessage, queue_size=10)

        # CV Bridge
        self.bridge = CvBridge()

    def point_range_filter(self, pts, point_range=[0, -39.68, -3, 69.12, 39.68, 1]):
        '''
        data_dict: dict(pts, gt_bboxes_3d, gt_labels, gt_names, difficulty)
        point_range: [x1, y1, z1, x2, y2, z2]
        '''
        # print(pts)
        if len(pts) == 0:
            # print("1", len(self.cluster_msg.poses))
            pass

        else:
            pts = pts[:, :4]
            flag_x_low = pts[:, 0] > point_range[0]
            flag_y_low = pts[:, 1] > point_range[1]
            flag_z_low = pts[:, 2] > point_range[2]
            flag_x_high = pts[:, 0] < point_range[3]
            flag_y_high = pts[:, 1] < point_range[4]
            flag_z_high = pts[:, 2] < point_range[5]
            keep_mask = flag_x_low & flag_y_low & flag_z_low & flag_x_high & flag_y_high & flag_z_high
            pts = pts[keep_mask]
        return pts

    def pc_callback(self, pc_msg):
        # print(pc_msg)
        self.msg = pc_msg
        try:
            # pc = read_points(pc_msg)
            # pc_msg = self.pointcloud2_to_nparray(pc_msg)
            pc_np = self.pointcloud2_to_xyz(pc_msg)
            self.pc_filtered = self.point_range_filter(pc_np)
            self.process_point_cloud(self.pc_filtered)
        except Exception as e:
            rospy.logerr(f"Failed to process point cloud: {str(e)}")

    def process_point_cloud(self, pc):
        # Convert numpy array to torch tensor
        pc_torch = torch.from_numpy(pc).to(self.device)
        with torch.no_grad():
            start_time = time.time()
            result = self.model(batched_pts=[pc_torch], mode='test')[0]
            total_time = time.time() - start_time
            print("model_inference_time",total_time)
        self.main(result)

    def pointcloud2_to_xyz(self, cloud_msg):
        point_list = []

        for point in pc2.read_points(cloud_msg, skip_nans=True):
            # kitty data에 intensite 값이 없어서 0값으로 임으로 넣어줌
            point_list.append((point[0], point[1], point[2], 0))
        point_np = np.array(point_list, np.float32)

        return point_np

    # import numpy as np

    # def publish_bounding_boxes(self, bboxes, labels, scores):
    #     marker_array = MarkerArray()
    #     for i, bbox in enumerate(bboxes):
    #         marker = Marker()
    #         marker.header.frame_id = "map"  # 혹은 포인트 클라우드 데이터에 맞는 적절한 frame_id
    #         marker.header.stamp = rospy.Time.now()
    #         marker.ns = "bboxes"
    #         marker.id = i
    #         marker.type = Marker.CUBE
    #         marker.action = Marker.ADD
    #
    #         # 바운딩 박스의 중심과 크기를 계산
    #         min_point = np.min(bbox, axis=0)
    #         max_point = np.max(bbox, axis=0)
    #         center = (max_point + min_point) / 2
    #         size = max_point - min_point
    #
    #         # 마커 설정
    #         marker.pose.position.x = center[0]
    #         marker.pose.position.y = center[1]
    #         marker.pose.position.z = center[2]
    #         marker.scale.x = size[0]
    #         marker.scale.y = size[1]
    #         marker.scale.z = size[2]
    #
    #         # 마커 색상과 투명도
    #         marker.color.r = 1.5
    #         marker.color.g = 1.5
    #         marker.color.b = 0.0
    #         marker.color.a = 0.5  # Alpha, 반투명
    #
    #         marker.lifetime = rospy.Duration(0.3)  # 0.1초 동안 지속
    #         marker_array.markers.append(marker)
    #
    #     self.marker_pub.publish(marker_array)
    #     self.pc_pub.publish(self.msg)

    def publish_bounding_boxes(self, bboxes, labels, scores):
        marker_array = MarkerArray()
        for i, (bbox, label, score) in enumerate(zip(bboxes, labels, scores)):
            marker = Marker()
            marker.header.frame_id = "map"  # 혹은 포인트 클라우드 데이터에 맞는 적절한 frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "bboxes"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # 바운딩 박스의 중심과 크기를 계산
            min_point = np.min(bbox, axis=0)
            max_point = np.max(bbox, axis=0)
            center = (max_point + min_point) / 2
            size = max_point - min_point

            # 마커 설정
            marker.pose.position.x = center[0]
            marker.pose.position.y = center[1]
            marker.pose.position.z = center[2]
            marker.scale.x = size[0]
            marker.scale.y = size[1]
            marker.scale.z = size[2]

            # 마커 색상과 투명도
            marker.color.r = 1.5
            marker.color.g = 1.5
            marker.color.b = 0.0
            marker.color.a = 0.5  # Alpha, 반투명

            marker.lifetime = rospy.Duration(0.3)  # 0.1초 동안 지속
            marker_array.markers.append(marker)

            # 텍스트 마커 생성
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "labels_scores"
            text_marker.id = i + 1000  # ID 충돌 방지
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = center[0]
            text_marker.pose.position.y = center[1]
            text_marker.pose.position.z = center[2] + 1  # 바운딩 박스 위로 조금 띄움
            text_marker.scale.z = 1.5  # 텍스트 크기
            if label == 0:
                label = "Pedestrian"
            elif label == 1:
                label = "Cyclist"
            elif label == 2:
                label = "Car"
            text_marker.text = f"{label} ({score:.2f})"
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.lifetime = rospy.Duration(0.5)
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)
        self.pc_pub.publish(self.msg)

    def main(self, result_filter):
        pcd_limit_range = np.array([0, -40, -3, 70.4, 40, 0.0], dtype=np.float32)
        start_time = time.time()
        result_filter = keep_bbox_from_lidar_range(result_filter, pcd_limit_range)

        lidar_bboxes = result_filter['lidar_bboxes']
        labels, scores = result_filter['labels'], result_filter['scores']
        # print(labels, scores)

        if len(lidar_bboxes.shape) == 2:
            # 각 바운딩 박스의 8개 코너점을 나타내는 형태로 변환
            bbox_corners=bbox3d2corners(lidar_bboxes)
            # print(bbox_corners.shape)  # (bbox 개수 , 8의 점 , 3차원 좌표)
            self.publish_bounding_boxes(bbox_corners, labels, scores)
        total_time = time.time() - start_time
        print("viz_time",total_time)
        # vis_pc(self.pc_filtered, bboxes=lidar_bboxes, labels=labels)


if __name__ == '__main__':
    try:
        node = PointPillarNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

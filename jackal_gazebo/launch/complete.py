#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from dynamic_reconfigure.client import Client
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class TEBParamAutoAdjust:
    def __init__(self):
        # 初始化参数
        self.latest_curvature = 0.0
        self.latest_distance = 5.0  # 默认最大距离
        self.latest_vx = 0.0

        # 动态参数客户端
        self.teb_ns = "/move_base/TebLocalPlannerROS"
        self.client = Client(self.teb_ns, timeout=5.0)

        # 订阅
        rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, self.path_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/cmd_vel', Twist, self.vel_callback)

        # 初始化模糊控制器
        self.init_fuzzy_controller()

        # 定时器，定期写参数
        rospy.Timer(rospy.Duration(0.5), self.update_teb_params)

    def init_fuzzy_controller(self):
        # 输入变量
        self.curvature = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'curvature')
        self.distance = ctrl.Antecedent(np.arange(0, 5.1, 0.1), 'distance')
        # 输出变量
        self.v_forward_max = ctrl.Consequent(np.arange(0, 2.1, 0.1), 'v_forward_max')
        self.v_backward_max = ctrl.Consequent(np.arange(0, 1.1, 0.1), 'v_backward_max')
        self.omega_max = ctrl.Consequent(np.arange(0, 2.1, 0.1), 'omega_max')
        self.a_linear_max = ctrl.Consequent(np.arange(0, 2.1, 0.1), 'a_linear_max')
        self.a_angular_max = ctrl.Consequent(np.arange(0, 3.1, 0.1), 'a_angular_max')

        # 输入模糊集
        self.curvature['small'] = fuzz.trimf(self.curvature.universe, [0, 0, 0.4])
        self.curvature['medium'] = fuzz.trimf(self.curvature.universe, [0.2, 0.5, 0.8])
        self.curvature['large'] = fuzz.trimf(self.curvature.universe, [0.6, 1, 1])
        self.distance['near'] = fuzz.trimf(self.distance.universe, [0, 0, 1.5])
        self.distance['medium'] = fuzz.trimf(self.distance.universe, [1, 2.5, 4])
        self.distance['far'] = fuzz.trimf(self.distance.universe, [3, 5, 5])

        # 输出模糊集
        for var in [self.v_forward_max, self.v_backward_max, self.omega_max, self.a_linear_max, self.a_angular_max]:
            var['low'] = fuzz.trimf(var.universe, [0, 0, var.universe[-1]*0.4])
            var['medium'] = fuzz.trimf(var.universe, [var.universe[-1]*0.2, var.universe[-1]*0.5, var.universe[-1]*0.8])
            var['high'] = fuzz.trimf(var.universe, [var.universe[-1]*0.6, var.universe[-1], var.universe[-1]])

        # 规则（与之前一致，建议拆分为单输出规则，或保持原有写法）
        rules = [
            ctrl.Rule(self.curvature['small'] & self.distance['far'],
                      [self.v_forward_max['high'], self.v_backward_max['low'], self.omega_max['low'], self.a_linear_max['high'], self.a_angular_max['low']]),
            ctrl.Rule(self.curvature['large'] & self.distance['near'],
                      [self.v_forward_max['low'], self.v_backward_max['medium'], self.omega_max['high'], self.a_linear_max['low'], self.a_angular_max['high']]),
            ctrl.Rule(self.curvature['medium'] & self.distance['medium'],
                      [self.v_forward_max['medium'], self.v_backward_max['medium'], self.omega_max['medium'], self.a_linear_max['medium'], self.a_angular_max['medium']]),
            ctrl.Rule(self.curvature['large'] & self.distance['far'],
                      [self.v_forward_max['medium'], self.v_backward_max['low'], self.omega_max['medium'], self.a_linear_max['medium'], self.a_angular_max['medium']]),
            ctrl.Rule(self.curvature['small'] & self.distance['near'],
                      [self.v_forward_max['low'], self.v_backward_max['high'], self.omega_max['low'], self.a_linear_max['medium'], self.a_angular_max['low']]),
            ctrl.Rule(self.curvature['small'] & self.distance['medium'],
                      [self.v_forward_max['medium'], self.v_backward_max['medium'], self.omega_max['low'], self.a_linear_max['medium'], self.a_angular_max['low']]),
            ctrl.Rule(self.curvature['medium'] & self.distance['near'],
                      [self.v_forward_max['low'], self.v_backward_max['medium'], self.omega_max['medium'], self.a_linear_max['low'], self.a_angular_max['high']]),
            ctrl.Rule(self.curvature['medium'] & self.distance['far'],
                      [self.v_forward_max['high'], self.v_backward_max['low'], self.omega_max['medium'], self.a_linear_max['high'], self.a_angular_max['medium']]),
            ctrl.Rule(self.curvature['large'] & self.distance['medium'],
                      [self.v_forward_max['medium'], self.v_backward_max['low'], self.omega_max['high'], self.a_linear_max['medium'], self.a_angular_max['high']]),
        ]
        self.system = ctrl.ControlSystem(rules)
        self.sim = ctrl.ControlSystemSimulation(self.system)

    def path_callback(self, msg):
        poses = msg.poses
        N = 7
        if len(poses) < N:
            return
        curvatures = []
        for i in range(N - 2):
            A = np.array([poses[i].pose.position.x, poses[i].pose.position.y])
            B = np.array([poses[i + 1].pose.position.x, poses[i + 1].pose.position.y])
            C = np.array([poses[i + 2].pose.position.x, poses[i + 2].pose.position.y])
            AB = B - A
            BC = C - B
            AC = C - A
            cross = np.cross(AB, AC)
            area = np.linalg.norm(cross) / 2.0
            len_AB = np.linalg.norm(AB)
            len_BC = np.linalg.norm(BC)
            len_CA = np.linalg.norm(AC)
            if len_AB * len_BC * len_CA == 0:
                kappa = 0.0
            else:
                kappa = 4 * area / (len_AB * len_BC * len_CA)
            curvatures.append(kappa)
        if curvatures:
            self.latest_curvature = np.mean(curvatures)
        else:
            self.latest_curvature = 0.0

    def vel_callback(self, msg):
        self.latest_vx = msg.linear.x

    def scan_callback(self, msg):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        # 判断方向
        if self.latest_vx >= 0.0:
            center_angle = 0.0
        else:
            center_angle = np.pi
            if center_angle > msg.angle_max:
                center_angle = -np.pi
        angle_window = np.radians(5)
        index_center = int((center_angle - angle_min) / angle_increment)
        index_min = int((center_angle - angle_window - angle_min) / angle_increment)
        index_max = int((center_angle + angle_window - angle_min) / angle_increment)
        index_min = max(0, index_min)
        index_max = min(len(ranges) - 1, index_max)
        valid_ranges = [
            ranges[i] for i in range(index_min, index_max + 1)
            if not np.isinf(ranges[i]) and not np.isnan(ranges[i])
        ]
        if valid_ranges:
            self.latest_distance = sum(valid_ranges) / len(valid_ranges)
        else:
            self.latest_distance = 5.0  # 没有数据时给最大距离

    def update_teb_params(self, event):
        # 输入归一化
        curvature = min(max(self.latest_curvature, 0.0), 1.0)
        distance = min(max(self.latest_distance, 0.0), 5.0)
        self.sim.input['curvature'] = curvature
        self.sim.input['distance'] = distance
        self.sim.compute()
        params = {
            'max_vel_x': float(self.sim.output['v_forward_max']),
            'max_vel_x_backwards': float(self.sim.output['v_backward_max']),
            'max_vel_theta': float(self.sim.output['omega_max']),
            'acc_lim_x': float(self.sim.output['a_linear_max']),
            'acc_lim_theta': float(self.sim.output['a_angular_max']),
        }
        try:
            self.client.update_configuration(params)
            rospy.loginfo("已自动调整TEB参数: %s", params)
        except Exception as e:
            rospy.logwarn("TEB参数调整失败: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('teb_param_auto_adjust')
    TEBParamAutoAdjust()
    rospy.loginfo("🚦 teb_param_auto_adjust 节点已启动，自动根据曲率和障碍物距离调整TEB参数。")
    rospy.spin()
    
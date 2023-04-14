import numpy as np
from scipy.optimize import linear_sum_assignment

class Drone:
    """四旋翼无人机类"""
    def __init__(self, x, y, z, vx, vy, vz):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.max_speed = 10.0 # 最大速度
        
    def move(self, dt, ax, ay, az):
        """根据动力学方程计算无人机移动"""
        self.vx = max(min(self.vx + ax*dt, self.max_speed), -self.max_speed)
        self.vy = max(min(self.vy + ay*dt, self.max_speed), -self.max_speed)
        self.vz = max(min(self.vz + az*dt, self.max_speed), -self.max_speed)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.z += self.vz * dt

# 初始化四个无人机
drone1 = Drone(0, 0, 0, 0, 0, 0)
drone2 = Drone(100, 0, 0, 0, 0, 0)
drone3 = Drone(0, 100, 0, 0, 0, 0)
drone4 = Drone(0, 0, 100, 0, 0, 0)

# 定义无人机列表
drones = [drone1, drone2, drone3, drone4]

# 定义无人机拦截目标列表，初始为None
targets = [None, None, None, None]

# 定义匈牙利算法的cost矩阵
cost_matrix = np.zeros((4, 4))

# 定义拦截时间
T = 10

# 定义时间步长
dt = 0.1

# 开始拦截循环
for t in np.arange(0, T, dt):

    # 更新cost矩阵
    for i, drone in enumerate(drones):
        for j, target in enumerate(targets):
            if target is not None:
                # 计算无人机到目标的距离
                dist = np.sqrt((drone.x - target.x)**2 + (drone.y - target.y)**2 + (drone.z - target.z)**2)
                # 计算无人机拦截时间
                intercept_time = dist / drone.max_speed
                # 更新cost矩阵
                cost_matrix[i][j] = intercept_time
    
    # 使用匈牙利算法匹配无人机和目标
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    
    # 移动无人机和目标
    for i, j in zip(row_ind, col_ind):
        drone = drones[i]
        target = targets[j]
        if target is not None:
            # 计算无人机到目标
            # 计算无人机到目标的方向向量
            dir_vector = np.array([target.x - drone.x, target.y - drone.y, target.z - drone.z])

            # 计算无人机到目标的距离
            dist = np.linalg.norm(dir_vector)

            # 如果无人机已经到达目标位置，则将目标设置为 None
            if dist <= threshold:
                targets[j] = None
                continue

            # 计算无人机到目标的单位方向向量
            dir_vector /= dist

            # 计算无人机朝向目标的旋转矩阵
            qw = math.cos(drone.theta / 2)
            qx = math.sin(drone.theta / 2) * drone.u
            qy = math.sin(drone.theta / 2) * drone.v
            qz = math.sin(drone.theta / 2) * drone.w
            rotation_matrix = np.array([
                [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
            ])

            # 计算无人机到目标的速度向量
            velocity_vector = np.array([drone.speed * drone.u, drone.speed * drone.v, drone.speed * drone.w])

            # 计算无人机到目标的相对速度向量
            relative_velocity_vector = np.dot(rotation_matrix.T, velocity_vector - target.velocity)

            # 计算追逐角速度
            chase_angular_velocity = np.cross(relative_velocity_vector, dir_vector)

            # 计算无人机的角速度
            drone.w = chase_angular_velocity[0], -chase_angular_velocity[1], -chase_angular_velocity[2], -qw

            # 计算无人机的欧拉角
            euler_angles = quaternion_to_euler_angles(drone.w)
            drone.roll = euler_angles[0]
            drone.pitch = euler_angles[1]
            drone.yaw = euler_angles[2]

            # 更新无人机的位置
            drone.move(dt)

return drones

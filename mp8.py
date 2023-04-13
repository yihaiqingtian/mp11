import numpy as np


# 初始化无人机的初始位置和速度
num_drones = 4
pos = np.zeros((num_drones, 3))
vel = np.zeros((num_drones, 3))

# 设置控制参数
k_pos = 1.0
k_vel = 1.0

# 设置模拟参数
dt = 0.01
t_end = 10.0

# 运行模拟
t = 0.0
while t < t_end:
    # 计算控制输入
    u = np.zeros((num_drones, 3))
    # 使用匈牙利算法计算无人机之间的拦截控制输入
    # ...
    
    # 更新位置和速度
    for j in range(num_drones):
        vel[j] += dt * u[j]
        pos[j] += dt * vel[j]
    
    # 更新时间
    t += dt

# 定义无人机类
class UAV:
    def __init__(self, pos, vel, theta, omega, v_max, omega_max):
        self.pos = np.array(pos)    # 位置向量
        self.vel = np.array(vel)    # 速度向量
        self.theta = theta          # 朝向角度
        self.omega = omega          # 角速度
        self.v_max = v_max          # 最大速度
        self.omega_max = omega_max  # 最大角速度

    # 动力学方程
    def dynamics(self, t, u, target_pos):
        # 系统参数
        m = 1.0    # 无人机质量
        k = 1.0    # 控制增益

        # 计算无人机到目标点的向量和距离
        r = target_pos - self.pos
        d = np.linalg.norm(r)

        # 计算期望速度和期望角速度
        v_d = self.v_max * r / d
        theta_d = np.arctan2(r[1], r[0])

        # 计算误差
        e_v = v_d - self.vel
        e_theta = theta_d - self.theta

        # 计算控制输入
        v_c = k * e_v
        omega_c = k * e_theta

        # 限制控制输入
        v_c = np.clip(v_c, -self.v_max, self.v_max)
        omega_c = np.clip(omega_c, -self.omega_max, self.omega_max)

        # 计算加速度和角加速度
        a = v_c
        alpha = omega_c

        # 更新速度和角速度
        self.vel += a * t
        self.omega += alpha * t

        # 限制速度和角速度
        self.vel = np.clip(self.vel, -self.v_max, self.v_max)
        self.omega = np.clip(self.omega, -self.omega_max, self.omega_max)

        # 更新位置和朝向角
        self.pos += self.vel * t
        self.theta += self.omega * t

# 初始化无人机位置和速度
pos = [np.array([0, 0]), np.array([10, 0]), np.array([0, 10]), np.array([10, 10])]
vel = [np.array([1, 1]), np.array([-1, 1]), np.array([1, -1]), np.array([-1, -1])]

# 设置拦截目标
target_pos = np.array([5, 5])

# 初始化控制输入
u = [np.array([0, 0]) for _ in range(num_drones)]

# 设置控制增益
k_p = 1.0
k_v = 1.0

# 设置模拟参数
dt = 0.01
capture_radius = 0.1
max_iterations = 1000

# 开始循环
for i in range(max_iterations):
    # 计算当前位置到目标的距离
    dist = [np.linalg.norm(pos[j] - target_pos) for j in range(num_drones)]

    # 判断是否全部拦截完成
    if all(d <= capture_radius for d in dist):
        print("All drones captured the target!")
        break

    # 计算匈牙利算法的最小权重匹配
    w = np.zeros((num_drones, num_drones))
    for j in range(num_drones):
        for k in range(num_drones):
            w[j, k] = dist[j] + np.linalg.norm(pos[j] - pos[k])
    row_ind, col_ind = scipy.optimize.linear_sum_assignment(w)

    # 计算控制输入
    for j in range(num_drones):
        u[j] = k_p * (pos[col_ind[j]] - pos[j]) + k_v * (vel[col_ind[j]] - vel[j])

    # 更新位置和速度
    for j in range(num_drones):
        vel[j] += dt * u[j]
        pos[j] += dt * vel[j]
        
# 第三部分：进行拦截和跟踪
for i in range(len(targets)):
    # 计算目标相对于无人机的距离和方位角
    dx = targets[i][0] - drones[i][0]
    dy = targets[i][1] - drones[i][1]
    distance = math.sqrt(dx**2 + dy**2)
    angle = math.atan2(dy, dx)
    
    # 判断目标是否在无人机的攻击范围内
    if distance <= attack_range:
        print(f"Drone {i+1} is attacking Target {i+1} at ({targets[i][0]}, {targets[i][1]})!")
        targets[i] = None  # 目标被摧毁，置为None
    else:
        # 无人机朝向目标方向前进
        drones[i][0] += drone_speed * math.cos(angle)
        drones[i][1] += drone_speed * math.sin(angle)
        print(f"Drone {i+1} is tracking Target {i+1} at ({targets[i][0]}, {targets[i][1]})...")
        
# 移除已被摧毁的目标
targets = [target for target in targets if target is not None]
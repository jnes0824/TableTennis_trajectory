import pybullet as p
import pybullet_data
import time
import numpy as np

class TableTennisScene:
    def __init__(self):
        #log angular velocity
        self.log_angular_velocity = False
        self.start_time = None
        self.is_contacting = False  # 添加接觸標記
        self.ball_trajectory = []  # 用於儲存球的歷史位置
        self.prev_ball_pos = None  # 上一步球的位置，用於畫即時軌跡

        # 初始化物理引擎
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)  # 設置重力
        
        # 設置相機參數
        self.camera_distance = 2.0  # 相機距離
        self.camera_yaw = 0  # 水平旋轉角度
        self.camera_pitch = -30  # 垂直旋轉角度
        self.camera_target = [0, 0, 0]  # 相機目標點
        
        # 桌球桌尺寸（單位：米）
        self.table_length = 2.74
        self.table_width = 1.525
        self.table_height = 0.76
        self.net_height = 0.1525
        
        # 球的參數
        self.ball_radius = 0.02  # 40mm直徑
        self.ball_mass = 0.01  

        # 初始發球力
        self.launch_force = [-8, 0, -2]  
        
        # 創建場景
        self.create_scene()
        
        # 設置初始相機視角
        self.update_camera()
        
    def create_scene(self):
        # 創建地面
        self.plane_id = p.loadURDF("plane.urdf")
        
        # 創建桌球桌
        self.create_table()
        
        # 創建球
        self.create_ball()
        
        # 創建球拍
        self.create_paddle()
        
    def create_table(self):
        # 創建桌球桌（使用簡單的盒子表示）
        table_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.table_length/2, self.table_width/2, 0.02]
        )
        table_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.table_length/2, self.table_width/2, 0.02],
            rgbaColor=[0, 0, 1, 1]  # 藍色桌面
        )
        
        self.table_id = p.createMultiBody(
            baseMass=0,  # 靜態物體
            baseCollisionShapeIndex=table_collision,
            baseVisualShapeIndex=table_visual,
            basePosition=[0, 0, self.table_height]
        )
        
        # 設置桌面的物理參數
        p.changeDynamics(self.table_id, -1, 
                        lateralFriction=0.45,  # 側向摩擦係數
                        restitution=0.9,      # 彈性係數
                        rollingFriction=0,   # 滾動摩擦係數
                        spinningFriction=0,  # 降低旋轉摩擦係數
                        contactStiffness=2000,
                        contactDamping=0)
                        
        
        # 創建球網
        net_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.01, self.table_width/2, self.net_height/2]
        )
        net_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.01, self.table_width/2, self.net_height/2],
            rgbaColor=[1, 1, 1, 1]  # 白色球網
        )
        
        self.net_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=net_collision,
            baseVisualShapeIndex=net_visual,
            basePosition=[0, 0, self.table_height + self.net_height/2]
        )
        
    def create_ball(self):
        # 創建桌球
        ball_collision = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=self.ball_radius
        )
        
        # 創建球體視覺效果
        ball_visual = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=self.ball_radius,
            rgbaColor=[1, 1, 1, 1]  # 白色球
        )
        
        # 創建紅色標記
        marker_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.ball_radius/4, self.ball_radius/4, self.ball_radius/4],
            rgbaColor=[1, 0, 0, 1]  # 紅色標記
        )
        
        # 設置球的初始位置
        start_pos = [self.table_length / 2 - 0.02, 0, self.table_height + 0.3]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        
        # 創建帶有標記的球
        self.ball_id = p.createMultiBody(
            baseMass=self.ball_mass,
            baseCollisionShapeIndex=ball_collision,
            baseVisualShapeIndex=ball_visual,
            basePosition=start_pos,
            baseOrientation=start_orn,
            linkMasses=[0],  # 標記的質量
            linkCollisionShapeIndices=[-1],  # 無碰撞
            linkVisualShapeIndices=[marker_visual],  # 標記的視覺效果
            linkPositions=[[self.ball_radius, 0, 0]],  # 標記的位置
            linkOrientations=[[0, 0, 0, 1]],  # 標記的方向
            linkInertialFramePositions=[[0, 0, 0]],
            linkInertialFrameOrientations=[[0, 0, 0, 1]],
            linkParentIndices=[0],
            linkJointTypes=[p.JOINT_FIXED],
            linkJointAxis=[[0, 0, 0]]
        )
        
        # 設置球的物理參數
        p.changeDynamics(self.ball_id, -1,
                        lateralFriction=0.1,     # 移除側向摩擦
                        restitution=0.9,        # 保持彈性係數
                        rollingFriction=0.001,    # 移除滾動摩擦
                        spinningFriction=0.001,   # 移除旋轉摩擦
                        linearDamping=0.01,      # 移除線性阻尼
                        angularDamping=0.0,     # 移除角阻尼
                        contactStiffness=10000,  # 增加接觸剛度
                        contactDamping=100    # 移除接觸阻尼
                        )
        
        # 給球施加一個向前的力
        p.applyExternalForce(
            self.ball_id,
            -1,  # -1 表示作用於物體的中心
            self.launch_force,
            start_pos,  # 力的作用點
            p.WORLD_FRAME  # 使用世界座標系
        )
        
    def create_paddle(self):
        # 創建球拍（改為大拍面、直立放在另一側）
        paddle_half_extents = [0.01, 0.15, 0.15]  # 增大拍面，直立為 y-z 面方向
        
        paddle_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=paddle_half_extents
        )
        paddle_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=paddle_half_extents,
            rgbaColor=[0.8, 0.8, 0.8, 1]
        )
        # 放置在桌子另一側，靠近球飛行軌跡
        start_pos = [-self.table_length / 2 + 0.05, 0, self.table_height + 0.17]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        self.paddle_id = p.createMultiBody(
            baseMass=0.08,
            baseCollisionShapeIndex=paddle_collision,
            baseVisualShapeIndex=paddle_visual,
            basePosition=start_pos,
            baseOrientation=start_orn
        )
        # 設置球拍的物理參數
        p.changeDynamics(self.paddle_id, -1, 
                         restitution=0.9, 
                         lateralFriction=1,
                         rollingFriction=0.005,
                         spinningFriction=0.005,)
        
    def reset_simulation(self, rotation_type=None):
        # 清空上一段軌跡
        self.ball_trajectory.clear()
        self.prev_ball_pos = None  # 清空上一段軌跡連接點
        # 重置球的位置和速度
        start_pos = [self.table_length / 2 - 0.02, 0, self.table_height + 0.3]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        p.resetBasePositionAndOrientation(self.ball_id, start_pos, start_orn)
        
        # 設置不同的旋轉效果
        # 角速度的三個分量分別代表繞 x, y, z 軸的旋轉
        # 正值表示順時針旋轉，負值表示逆時針旋轉
        angular_vel = [0, 0, 0]  # 初始值為無旋轉
        
        # 根據旋轉類型設置角速度
        if rotation_type == 1:
            # 上旋：繞 y 軸順時針旋轉
            angular_vel = [0, 200, 0]  # 增加旋轉速度
        elif rotation_type == 2:
            # 下旋：繞 y 軸逆時針旋轉
            angular_vel = [0, -500, 0]  # 增加旋轉速度
        elif rotation_type == 3:
            # 側旋：繞 z 軸旋轉
            self.log_angular_velocity = True
            self.start_time = time.time()
            angular_vel = [0, 0, 300]  # 增加旋轉速度
        elif rotation_type == 4:
            # 混合旋轉
            angular_vel = [200, 200, 200]  # 增加旋轉速度
            
        # 設置線速度和角速度
        p.resetBaseVelocity(self.ball_id, [0, 0, 0], angular_vel)
        
        # 重新施加初始力
        p.applyExternalForce(
            self.ball_id,
            -1,
            self.launch_force,
            start_pos,
            p.WORLD_FRAME
        )
        
    def update_camera(self):
        # 更新相機視角
        p.resetDebugVisualizerCamera(
            cameraDistance=self.camera_distance,
            cameraYaw=self.camera_yaw,
            cameraPitch=self.camera_pitch,
            cameraTargetPosition=self.camera_target
        )
        
    def run_simulation(self):
        # 運行模擬
        for _ in range(500):
            p.stepSimulation()  # 先做物理步

            ball_pos, _ = p.getBasePositionAndOrientation(self.ball_id)
            self.ball_trajectory.append(ball_pos)   # 再紀錄更新後的位置
            # 實時繪製紅色軌跡
            if self.prev_ball_pos is not None:
                p.addUserDebugLine(self.prev_ball_pos, ball_pos, [1, 0, 0], 2, 0)  # lifeTime=0 => 永久
            self.prev_ball_pos = ball_pos
            # paddle_pos, paddle_orn = p.getBasePositionAndOrientation(self.paddle_id)
            # target_x = ball_pos[0] - 0.05  # 讓球拍略提前迎擊
            # target_z = ball_pos[2]
            # new_pos = [target_x, 0, target_z]
            # p.resetBasePositionAndOrientation(self.paddle_id, new_pos, paddle_orn)

            # 印出角速度資訊（只印第一次上旋後的幾秒）
            if self.log_angular_velocity and self.start_time:
                now = time.time()
                if now - self.start_time <= 3:  # 只印前 3 秒
                    _, angular_vel = p.getBaseVelocity(self.ball_id)
                    print(f"[{now - self.start_time:.2f}s] Angular velocity:", angular_vel)
                else:
                    self.log_angular_velocity = False
                    self.start_time = None

            print(f"球位置: {ball_pos}")
            # # 檢查球是否接觸桌子（使用更寬鬆的條件）
            # if abs(ball_pos[2] - (self.table_height + self.ball_radius)) < 0.02:  # 增加容許誤差
            #     if not self.is_contacting:  # 只在開始接觸時印出
            #         _, angular_vel = p.getBaseVelocity(self.ball_id)
            #         print(f"球接觸桌面 - 位置: {ball_pos}, 角速度: {angular_vel}")
            #         self.is_contacting = True
            # else:
            #     self.is_contacting = False  # 重置接觸標記

            time.sleep(1./240.)  # 模擬時間步長
            
            # 檢查按鍵
            keys = p.getKeyboardEvents()
            for key, state in keys.items():
                if state & p.KEY_WAS_TRIGGERED:
                    if key == ord('r'):
                        # 重置無旋轉
                        self.reset_simulation()
                    elif key == ord('1'):
                        # 上旋
                        self.reset_simulation(1)
                        self.is_contacting = False
                    elif key == ord('2'):
                        # 下旋
                        self.reset_simulation(2)
                    elif key == ord('3'):
                        # 側旋
                        self.reset_simulation(3)
                    elif key == ord('4'):
                        # 混合旋轉
                        self.reset_simulation(4)
                    # 相機控制
                    elif key == ord('w'):  # 放大
                        self.camera_distance = max(0.5, self.camera_distance - 0.1)
                        self.update_camera()
                    elif key == ord('s'):  # 縮小
                        self.camera_distance = min(5.0, self.camera_distance + 0.1)
                        self.update_camera()
                    elif key == ord('a'):  # 左轉
                        self.camera_yaw = (self.camera_yaw - 5) % 360
                        self.update_camera()
                    elif key == ord('d'):  # 右轉
                        self.camera_yaw = (self.camera_yaw + 5) % 360
                        self.update_camera()
                    elif key == ord('q'):  # 上仰
                        self.camera_pitch = min(89, self.camera_pitch + 5)
                        self.update_camera()
                    elif key == ord('e'):  # 下俯
                        self.camera_pitch = max(-89, self.camera_pitch - 5)
                        self.update_camera()
        # 儲存軌跡到檔案
        np.savetxt("ball_trajectory.csv", self.ball_trajectory, delimiter=",", header="x,y,z", comments='')
        print("已儲存球軌跡至 ball_trajectory.csv")
        
    def cleanup(self):
        p.disconnect()

if __name__ == "__main__":
    # 創建並運行場景
    scene = TableTennisScene()
    try:
        scene.run_simulation()
    finally:
        scene.cleanup()
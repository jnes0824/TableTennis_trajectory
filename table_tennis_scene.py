import pybullet as p
import pybullet_data
import time
import numpy as np

class TableTennisScene:
    def __init__(self):
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
        self.ball_mass = 0.0027  # 2.7g
        
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
                        lateralFriction=0.3,  # 側向摩擦係數
                        restitution=0.8,      # 彈性係數
                        rollingFriction=0.1,  # 滾動摩擦係數
                        spinningFriction=0.1) # 旋轉摩擦係數
        
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
        
        # 設置球的初始位置（在桌子一側上方）
        start_pos = [self.table_length/4, 0, self.table_height + 0.3]
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
                        lateralFriction=0.1,   # 側向摩擦係數
                        restitution=0.8,       # 彈性係數
                        rollingFriction=0.1,   # 滾動摩擦係數
                        spinningFriction=0.1,  # 旋轉摩擦係數
                        linearDamping=0.1,     # 線性阻尼
                        angularDamping=0.1)    # 角阻尼
        
        # 給球施加一個向前的力
        p.applyExternalForce(
            self.ball_id,
            -1,  # -1 表示作用於物體的中心
            [-2.5, 0, -1.5],  # 力的大小和方向 (x, y, z)
            start_pos,  # 力的作用點
            p.WORLD_FRAME  # 使用世界座標系
        )
        
    def create_paddle(self):
        # 創建球拍（使用簡單的圓盤表示） 
        paddle_radius = 0.15  # 球拍半徑
        paddle_thickness = 0.01  # 球拍厚度
        
        paddle_collision = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=paddle_radius,
            height=paddle_thickness
        )
        paddle_visual = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=paddle_radius,
            length=paddle_thickness,
            rgbaColor=[0.8, 0.8, 0.8, 1]  # 灰色球拍
        )
        
        # 設置球拍的初始位置（在桌子另一側）
        start_pos = [-self.table_length/4, 0, self.table_height + 0.2]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        
        self.paddle_id = p.createMultiBody(
            baseMass=0.2,  # 球拍質量
            baseCollisionShapeIndex=paddle_collision,
            baseVisualShapeIndex=paddle_visual,
            basePosition=start_pos,
            baseOrientation=start_orn
        )
        
    def reset_simulation(self, rotation_type=None):
        # 重置球的位置和速度
        start_pos = [self.table_length/4, 0, self.table_height + 0.3]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        p.resetBasePositionAndOrientation(self.ball_id, start_pos, start_orn)
        
        # 設置不同的旋轉效果
        # 角速度的三個分量分別代表繞 x, y, z 軸的旋轉
        # 正值表示順時針旋轉，負值表示逆時針旋轉
        angular_vel = [0, 0, 0]  # 初始值為無旋轉
        
        # 根據旋轉類型設置角速度
        if rotation_type == 1:
            # 上旋：繞 y 軸順時針旋轉
            angular_vel = [0, 10, 0]
        elif rotation_type == 2:
            # 下旋：繞 y 軸逆時針旋轉
            angular_vel = [0, -10, 0]
        elif rotation_type == 3:
            # 側旋：繞 z 軸旋轉
            angular_vel = [0, 0, 10]
        elif rotation_type == 4:
            # 混合旋轉
            angular_vel = [5, 5, 5]
            
        # 設置線速度和角速度
        p.resetBaseVelocity(self.ball_id, [0, 0, 0], angular_vel)
        
        # 重新施加初始力
        p.applyExternalForce(
            self.ball_id,
            -1,
            [-2.5, 0, -1.5],
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
        for _ in range(10000):
            p.stepSimulation()
            time.sleep(1./1000.)  # 模擬時間步長
            
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
        
    def cleanup(self):
        p.disconnect()

if __name__ == "__main__":
    # 創建並運行場景
    scene = TableTennisScene()
    try:
        scene.run_simulation()
    finally:
        scene.cleanup() 
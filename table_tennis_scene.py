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
        ball_visual = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=self.ball_radius,
            rgbaColor=[1, 1, 1, 1]  # 白色球
        )
        
        # 設置球的初始位置（在桌子一側上方）
        start_pos = [self.table_length/4, 0, self.table_height + 0.3]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])
        
        self.ball_id = p.createMultiBody(
            baseMass=self.ball_mass,
            baseCollisionShapeIndex=ball_collision,
            baseVisualShapeIndex=ball_visual,
            basePosition=start_pos,
            baseOrientation=start_orn
        )
        
        # 設置球的物理參數
        p.changeDynamics(self.ball_id, -1,
                        lateralFriction=0.1,   # 側向摩擦係數
                        restitution=0.8,       # 彈性係數
                        rollingFriction=0.1,   # 滾動摩擦係數
                        spinningFriction=0.1,  # 旋轉摩擦係數
                        linearDamping=0.1,     # 線性阻尼
                        angularDamping=0.1)    # 角阻尼
        
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
        
    def run_simulation(self):
        # 運行模擬
        for _ in range(10000):
            p.stepSimulation()
            time.sleep(1./240.)  # 模擬時間步長
            
    def cleanup(self):
        p.disconnect()

if __name__ == "__main__":
    # 創建並運行場景
    scene = TableTennisScene()
    try:
        scene.run_simulation()
    finally:
        scene.cleanup() 
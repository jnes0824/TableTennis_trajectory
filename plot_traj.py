import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# 讀取 CSV（pandas 會自動處理 header 及 UTF-8 BOM）
df = pd.read_csv("ball_trajectory.csv")

# 四捨五入到小數點後三位
df = df.round(3)

# 取出 x, y, z 三欄
x, y, z = df["x"].to_numpy(), df["y"].to_numpy(), df["z"].to_numpy()

# 繪製 3D 軌跡
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, color="red", label="Ball Trajectory")
ax.scatter(x[0], y[0], z[0], color="green", s=30, label="start")
ax.scatter(x[-1], y[-1], z[-1], color="blue", s=30, label="end")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3D Ball Trajectory")
ax.legend()
plt.show()
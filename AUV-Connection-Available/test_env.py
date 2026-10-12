import holoocean
import time

print("正在启动 Unreal Engine 并加载码头场景 (Pier-Hovering)...")
# 加载场景
env = holoocean.make("Pier-Hovering")

# 重置并初始化物理世界
env.reset()
print("场景加载完成！")
print("保持渲染中... (在终端按 Ctrl+C 退出)")

try:
    # 必须要有持续的 step 循环，画面才会实时刷新
    while True:
        # 给 8 个推进器发送 0 推力（让 AUV 原地悬停）
        env.step([0, 0, 0, 0, 0, 0, 0, 0])
        
        # 稍微加一点延时，防止 CPU 占用率飙到 100%
        time.sleep(0.01) 
        
except KeyboardInterrupt:
    print("\n收到退出指令，正在关闭仿真环境...")
finally:
    env.__exit__()
import holoocean
import numpy as np
import cv2  # 引入视觉库

print("正在启动码头场景...")
env = holoocean.make("PierHarbor-HoveringCamera")
env.reset()

straight_action = [-10, -10, 5, 5, 0, 0, 0, 0]

try:
    for _ in range(5000):
        state = env.step(straight_action)
        
        # 1. 打印坐标（保留你之前的逻辑）
        if 'PoseSensor' in state:
            current_pos = state['PoseSensor'][0:3, 3]
            print(f"坐标: {current_pos}")
            
        # 2. 提取并显示左目摄像头画面
        if 'LeftCamera' in state:
            # HoloOcean 返回的图像是 RGBA 格式的 numpy 数组
            image_rgba = state['LeftCamera']
            
            # 转换为 OpenCV 标准的 BGR 格式以便显示
            image_bgr = cv2.cvtColor(image_rgba, cv2.COLOR_RGBA2BGR)
            
            # 显示画面
            cv2.imshow("AUV Camera View", image_bgr)
            
            # 刷新图像并检测是否按下了 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("\n收到退出指令，正在关闭...")
finally:
    cv2.destroyAllWindows()
    env.__exit__()
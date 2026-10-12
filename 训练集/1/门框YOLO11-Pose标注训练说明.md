# 门框 YOLO11-Pose 标注与训练说明

## 方案

- 原有 YOLO11n 检测模型继续识别 `red_ball`、`blue_ball` 和 `gate`。
- 独立 YOLO11n-Pose 模型只负责门框四角。
- 关键点固定顺序：`1左上 → 2右上 → 3右下 → 4左下`。
- 第一版只标注四角均清楚可见的门框；局部出画、严重遮挡或无法判断角点的图片按 `K` 跳过。
- 所有新增标注写入 `gate_pose_annotations`，不会修改原三类检测标签。

## 1. 标注训练集

```bash
cd /home/cjx/cjx_project/auv比赛/训练集/1
conda activate auv_env
python3 label_gate_pose.py --split train
```

每个门框依次点击：

```text
1 左上角
2 右上角
3 右下角
4 左下角
```

按键：

| 按键 | 功能 |
|---|---|
| 鼠标左键 | 标记当前角点 |
| `Z` | 撤销当前门框最后一个角点 |
| `Tab` | 切换到同一图片中的下一个门框 |
| `X` | 删除当前误检或重复门框 |
| `Enter` / `D` / 空格 | 四角全部完成后保存并进入下一张 |
| `K` | 跳过不适合Pose训练的图片 |
| `A` | 返回上一张 |
| `Q` | 退出程序；之前按 `Enter` 完成的图片已经保存，当前图片不保存 |

如果图片中只有一个实际门框却显示两个候选框，保留一个并按 `X` 删除另一个。四个角必须标在红色门框管材的几何中心线上，并在全部图片中保持相同顺序。

一个门框的四点完成后，工具会自动选中下一个未完成候选框：

- 如果画面中确实有第二个完整门框，继续按相同顺序点击四角；
- 如果第二个框只是重复框或误检框，直接按 `X` 删除；
- 也可以随时按 `Tab` 手动切换候选框。

按 `K` 跳过的图片会被记录，重新启动时不会再次出现。如果以后想重新检查这些图片：

```bash
python3 label_gate_pose.py --split train --include-skipped
```

## 2. 标注验证集

```bash
python3 label_gate_pose.py --split val
```

建议至少完成：

- 训练集 300～500 张；
- 验证集 80～120 张；
- 覆盖远近、左右偏航、上下俯视、明暗和浑水场景。

检查当前数量及格式：

```bash
python3 label_gate_pose.py --split train --check-only
python3 label_gate_pose.py --split val --check-only
```

复查已经标注的图片：

```bash
python3 label_gate_pose.py --split train --include-reviewed
```

## 3. 生成训练目录

```bash
python3 prepare_gate_pose_dataset.py
```

需要重新生成时：

```bash
python3 prepare_gate_pose_dataset.py --force
```

输出目录：

```text
/home/cjx/cjx_project/auv比赛/训练集/1/gate_pose_yolo11_dataset
```

## 4. 使用显卡训练

```bash
python3 train_gate_yolo11n_pose.py \
  --device 0 \
  --epochs 150 \
  --batch 32 \
  --imgsz 640
```

如果显存不足，把 `--batch 32` 改成 `--batch 16`。最佳模型默认位于：

```text
/home/cjx/cjx_project/auv比赛/训练集/1/training_runs/gate_yolo11n_pose/weights/best.pt
```

## 5. 后续控制用途

Pose模型输出的四角用于计算门框中心、像素宽高、透视倾斜和PnP姿态。控制层仍应以二维中心误差为主，PnP距离为辅；接近门框后锁定陀螺仪航向和压力深度，进入固定前推的穿越状态。

## 6. 第一轮模型自动预标注

自动预测只写入独立的 `gate_pose_pseudo` 暂存目录，不会进入正式训练集：

```bash
python3 auto_label_gate_pose.py
```

人工审核高置信度预标注：

```bash
python3 label_gate_pose.py --split train --review-pseudo
```

审核操作与人工标注相同。预测正确直接按 `Enter`；错误点按 `Z` 撤销后重新点击；重复实例用 `X` 删除；无法使用的图片按 `K` 跳过。只有按 `Enter` 确认的标签才会写入正式 `gate_pose_annotations`。

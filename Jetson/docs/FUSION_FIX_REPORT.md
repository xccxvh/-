# 融合坐标跳变修复：2026-11-12

## 原因与修复

原算法在颜色质心与几何坐标相距不足38.4px时融合，并使用颜色质心作为输出。+2cm失败录像第11/16帧分别输出746.57/695.87px，而真实钢珠几何位置约720px。
新版 hybrid 的坐标及宽高均取几何轮廓；颜色只辅助置信度。匹配选择距离最小的一对一候选，且距离小于几何宽度的一半与图宽3%两者的较小值。没有增加时间滤波或按目标位置裁剪。

新增 measurement_contract.py，坐标版本 geometry-envelope-v2；旧标定缺少此版本时拒绝加载、继续添加和重新拟合，避免误用。
完整修复前快照：/home/user/ball_vision_before_fusion_20261112_210037。
独立审查 Volta 未发现重要问题。

## 验证

- 先看到4个融合测试和1个旧标定兼容性测试失败，再修复；本机及Nano Python3.6全套36项测试均通过。
- 原+2cm失败录像46个测量全部保留：最大误差0.44061→0.03059cm，MAE0.00556cm，抖动标准差0.00734cm（新版重算标定）。
- 为排除重标定掩盖错误，离线固定旧分段映射，仅代入修复后的像素坐标，最大误差0.01829cm。该诊断不解除旧标定的运行时禁用。
- 旧中点录像仍294/298 DETECTED，无球录像仍0/298 DETECTED。
- 新拍的69帧中黑色背景已移开，0帧DETECTED，不能作为新的厘米精度通过证据。
- 原+2cm录像已参与问题定位，现在属于回归数据，不再称为新的独立验证；需恢复背景、重新取样并验证其他刻度。

## 新标定与使用

复用原始确认帧区间重新处理7/12/17cm录像，没有使用+2cm录像拟合。
新三点：-5=325.45162px，0=599.74985px，+5=895.48525px。
线性拟合最大残差0.12530cm，低于现有0.15cm选择阈值，因此采用线性模型（不再是旧分段模型）。拟合残差不等于独立精度。

Nano修复版：/home/jetson/ball_vision_fusion_20261112_2102
配置：config_fusion_v2.json
标定：calibration_fusion_v2.json

```bash
cd /home/jetson/ball_vision_fusion_20261112_2102
OPENBLAS_CORETYPE=ARMV8 python3 main.py --config config_fusion_v2.json --csv --display
```

旧Nano工程和候选目录保留用于回滚；后续使用上面的新目录和配置。UART仍因不可信采集时间而保持控制无效，没有修改控制器或舵机。
原始测试输出和CSV位于 ../test_results/fusion_v2/ 。


## 新现场复测（背景恢复后）

新录像 raw_retest_plus2_fusion_20260324_162954_1280x720.avi，用户确认纸尺14cm=+2cm；未用于拟合或修改参数。
Nano结果：55帧中初始4帧建立跟踪，随后51/51有效且姿态检查通过。平均测量1.98742cm，MAE0.01258cm，最大绝对误差0.01782cm，抖动标准差0.00313cm，峰峰值0.01146cm。
含启动整段有效比例92.73%，因此全段报告passed=false；建立跟踪后报告passed=true。不能据此宣称全轨道或动态精度。
报告：test_results/fusion_v2/retest_plus2_full.json、retest_plus2_tracked.json。标定SHA-256核对未改变。
下一步纸尺10cm（-2cm）独立验证。


## -2cm独立验证

纸尺10cm新录像 raw_verify_ruler10_minus2_20260324_163142_1280x720.avi：初始4帧建立跟踪，随后46/46有效。平均-2.14145cm，MAE0.14145cm，最大绝对误差0.14571cm，抖动标准差0.00215cm。建立跟踪后该点通过；全段46/50含启动未达95%。本点有稳定负偏差，不能用低抖动代替绝对精度；未据此修改标定。配置/标定SHA-256对应既有新版标定。
报告：test_results/fusion_v2/verify_minus2_full.json、verify_minus2_tracked.json。
下一步纸尺8cm（-4cm）、16cm（+4cm）验证。


## -4cm独立验证

纸尺8cm，原始录像raw_verify_ruler8_minus4_20260324_163318_1280x720.avi。初始4帧建立跟踪，之后46/46有效；平均-4.11215cm，MAE0.11215cm，最大绝对误差0.12135cm，标准差0.00316cm，峰峰值0.01525cm。建立跟踪后该点通过；全段46/50含启动有效率92%。标定SHA-256不变。报告test_results/fusion_v2/verify_minus4_full.json、verify_minus4_tracked.json。下一步纸尺16cm（+4cm）。

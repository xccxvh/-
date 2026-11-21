# 旋转加速快照 · 离板厘米验证增量

**加速版 left_print 的 raw 和 points 均未通过逐摆放静态验收。** 重新拟合后，两种模式的系数、全部逐帧测量和误差与旧版一致。Nano 定时回放已归档的 digits 日志按处理帧数除以约 30.01 秒计算，372 帧提升至 684 帧；本次厘米验证在主机全帧回放。[旧版 left_print 报告](motion_left_print_20261121_v1/REPORT.md)

|项目|0e6af5e|fast_bundle|
|---|---:|---:|
|Nano 定时回放处理率，digits|12.4 FPS|22.8 FPS|
|left_print 拟合 −6 / O / +6，各 150 帧|146 / 145 / 143|146 / 145 / 143|
|第一轮有效数；raw / points MAE（cm）|582/600；0.152 / 0.033|582/600；0.152 / 0.033|
|第二轮有效数；raw / points MAE（cm）|574/600；0.143 / 0.088|574/600；0.143 / 0.088|
|两轮有效数；raw / points 最大绝对误差（cm）|1156/1200；0.338 / 0.205|1156/1200；0.338 / 0.205|
|独立 O；raw / points 最大绝对误差（cm）|145/150；0.094 / 0.132|145/150；0.094 / 0.132|
|固定 O 倾角；参考有效 / 轴向有效（共 895 帧）|860 / 786|860 / 786|
|倾角 raw / points MAE（cm）；最长连续缺测|0.430 / 0.433；47 帧|0.430 / 0.433；47 帧|
|digits 的 +6 cm 拟合点，raw / points 有效数|0/150、0/150|0/150、0/150|

第二轮 −5 cm 两种模式均只有 137/150 帧，低于 95% 所需的 143 帧。第二轮 −3 cm 的 raw 最大误差 0.338 cm，超过 0.300 cm。points 全域最大误差 0.205 cm。固定 O 倾角的最大绝对偏差为 raw 0.518 cm、points 0.514 cm。digits 的 +6 原片全部 150 帧均为 `reference_rotation_recheck_weak_or_ambiguous_patch`，无法补足该位置最低 20 个拟合样本。digits 本次仅复核该段。

|left_print 绑定|0e6af5e|fast_bundle|
|---|---|---|
|raw|`3305f0161444c368b74a67253a9f444cdfd636e89d2076c6c56bc926fdde8b32`|`d032120b00008f4869b3ba69c0d83dd473a4a914dfda88b1e57308612b211354`|
|points|`e5de9361d43a2e1b13e56e33c0f3b9a266e794e337c127bda184075e3d0d4d76`|`7999ae6a2c919a251e26c23cc17f35ad9c59396837c49bf3d68e01ea9a4446d7`|

`MOTION_REPLAY_JETSON` 指向 `7/motion_20261121/fast_bundle/Jetson`。构成算法绑定的八个源码文件中，变更项为 `ruler_reference.py`（快照 SHA-256 `0ec8c8a2dd873e0243226e58d69d2924eb6e0870b6859bf23aefa26ae985926f`）。使用快照原配置、参考图和内参；14 段静态原片沿用独立摆放、两段错放排除及首段名义时钟。对照旧档逐字段核验静态 4200 条、倾角 1790 条、digits +6 的 300 条双模式记录，唯一差异为 `rail_binding_id`。加速版系数仅供审计，加载器对两份审计文件均拒绝启用。

主机工作区保留全量回放记录。[静态两轮与拟合结果](</home/user/桌面/jetson_nano/7/motion_calibration/fast_left_static/evaluation.json>) · [逐段有效数与视频哈希](</home/user/桌面/jetson_nano/7/motion_calibration/fast_left_static/placements.csv>) · [配置与时钟哈希](</home/user/桌面/jetson_nano/7/motion_calibration/fast_left_static/inputs.json>) · [固定 O 倾角](</home/user/桌面/jetson_nano/7/motion_calibration/fast_left_tilt/summary.json>) · [digits +6](</home/user/桌面/jetson_nano/7/motion_calibration/fast_digits_p6/placements.csv>)

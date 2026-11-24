# 杆上钢球的候选检测与融合：三个上游实现

本项目可优先试验两种方法：在已标定的杆面内用轮廓补充候选，用拖影的短轴核对球径。帧差和轨迹预测可以缩小搜索范围；发布球心仍需当前帧的球缘、位置和球体外观通过核验。以下按 2026-11-22 查询到的 GitHub 默认分支最新提交核对。README 说明作者的目标，算法细节取自可执行源码；本文未引用论文性能数据，也未在本机复测上游指标。

## FlydreamKM/ball-beam-project · 杆带内的轮廓质心

README 分别介绍视觉原型和闭环控制程序。指定的 `vision_pc/main.cpp` 用 Canny 和概率霍夫直线寻找近水平杆边，按线长累加到纵坐标直方图，以两个峰确定杆带。跟踪时取上次**实测**球心附近的小窗，与有效杆带相交。窗内灰度阈值分割后做开、闭运算，筛除面积越界或圆度低于 0.45 的外轮廓，取“面积 × 圆度”最高者，以轮廓矩求质心。漏检期间最多 30 帧仍可发送匀速卡尔曼外推的位置和速度。可借鉴局部搜索和廉价轮廓评分；杆端裁掉部分球面时，剩余轮廓的质心会向窗内偏移，应回到当前图像核对完整球缘。[README](https://github.com/FlydreamKM/ball-beam-project/blob/fd00eec7141624868ac081b2168b6d8ed7615cf4/README.md#L132-L145) · [球与杆带检测](https://github.com/FlydreamKM/ball-beam-project/blob/fd00eec7141624868ac081b2168b6d8ed7615cf4/vision_pc/main.cpp#L159-L240) · [搜索、漏检与串口输出](https://github.com/FlydreamKM/ball-beam-project/blob/fd00eec7141624868ac081b2168b6d8ed7615cf4/vision_pc/main.cpp#L438-L482)

## AndreiMunteanu2911/Ball-Detection-Computer-Vision · 多源候选与单球关联

- **候选从哪里来。** 形状分支在灰度边缘图上提取外轮廓，用最小包围圆给出中心和半径，再按宽高比、圆度、填充率和面积评分。运动分支对两幅灰度图做绝对差、阈值和开闭运算，另计运动像素密度。差分可能同时留下钢球出发和到达处的轮廓，静止球也可能没有响应；运动分支适合提示搜索位置，最终球心应取自当前帧。[circle_blob.py](https://github.com/AndreiMunteanu2911/Ball-Detection-Computer-Vision/blob/809dee889ab430bd1e8be4a571aeec8cab092243/src/ball_tracker/detectors/circle_blob.py#L37-L150) · [motion.py](https://github.com/AndreiMunteanu2911/Ball-Detection-Computer-Vision/blob/809dee889ab430bd1e8be4a571aeec8cab092243/src/ball_tracker/detectors/motion.py#L83-L190)
- **融合发生在哪里。** 管线汇集候选、补充外观描述，逐个重算置信度并排序，候选坐标保持各自独立。关联先按置信度、预测距离及可用的半径稳定性和外观指标筛选，再综合距离、置信度、尺寸和可用外观分数选一个候选。卡尔曼按实际时间差预测二维位置、速度，用二维位置观测校正。README 的断档跟踪包括单列的预测状态。本项目可沿用候选来源与分项评分，在高光、刻度或工具产生竞争候选时保留歧义拒绝，仅将预测用于关联。[README](https://github.com/AndreiMunteanu2911/Ball-Detection-Computer-Vision/blob/809dee889ab430bd1e8be4a571aeec8cab092243/README.md#L7-L15) · [候选汇集](https://github.com/AndreiMunteanu2911/Ball-Detection-Computer-Vision/blob/809dee889ab430bd1e8be4a571aeec8cab092243/src/ball_tracker/pipeline.py#L264-L272) · [逐候选排序](https://github.com/AndreiMunteanu2911/Ball-Detection-Computer-Vision/blob/809dee889ab430bd1e8be4a571aeec8cab092243/src/ball_tracker/detectors/appearance.py#L139-L148) · [association.py](https://github.com/AndreiMunteanu2911/Ball-Detection-Computer-Vision/blob/809dee889ab430bd1e8be4a571aeec8cab092243/src/ball_tracker/tracking/association.py#L26-L111) · [kalman.py](https://github.com/AndreiMunteanu2911/Ball-Detection-Computer-Vision/blob/809dee889ab430bd1e8be4a571aeec8cab092243/src/ball_tracker/tracking/kalman.py#L67-L106)

## tsnyder701/trajec · 高速抛球的椭圆与拖影

README 面向高帧率抛球视频。入口先将画面配准到参考帧；检测器在 HSV 的 V 通道做 CLAHE，经颜色阈值和形态学处理，对外轮廓拟合椭圆。判据包括短轴与标定球径的偏差、离心率、轮廓面积与凸包面积之比，以及中心是否落在允许区域。对沿杆运动的钢球，值得检验的是拖影短轴能否保持球径，以及当前两侧球缘能否支持这个中心；亮黄色阈值依赖上游拍摄对象。[README](https://github.com/tsnyder701/trajec/blob/06873ee6a39c894bcf94614bf9bcf1e356cec1cf/README.md#L71-L83) · [处理入口](https://github.com/tsnyder701/trajec/blob/06873ee6a39c894bcf94614bf9bcf1e356cec1cf/src/trajec/core.clj#L98-L131) · [椭圆检测](https://github.com/tsnyder701/trajec/blob/06873ee6a39c894bcf94614bf9bcf1e356cec1cf/src/trajec/detection.clj#L16-L145)

已确认轨迹先按预测距离贪心匹配，剩余检测再匹配暂定轨迹，连续两次命中才确认。代码从成功匹配中拟合“长轴减短轴”与速度的比例；匹配函数实际只检查距离。丢帧时使用带重力输入的卡尔曼预测，并把补点标成 `interpolated`。两次实测确认可用于评估重捕；重力抛物线和补点适用于其飞行轨迹，杆上钢球的静止、反向和离开视野应分别用实拍回放验证。[运动模型](https://github.com/tsnyder701/trajec/blob/06873ee6a39c894bcf94614bf9bcf1e356cec1cf/src/trajec/tracking.clj#L9-L55) · [补点、匹配与确认](https://github.com/tsnyder701/trajec/blob/06873ee6a39c894bcf94614bf9bcf1e356cec1cf/src/trajec/tracking.clj#L190-L295) · [拖影统计](https://github.com/tsnyder701/trajec/blob/06873ee6a39c894bcf94614bf9bcf1e356cec1cf/src/trajec/tracking.clj#L312-L331)

## 许可与使用范围

FlydreamKM 和 AndreiMunteanu2911 的固定提交中均未找到项目许可证文件。trajec 的 LICENSE 和 README 标明 EPL-2.0，构建元数据另列带 Classpath 例外的 GPL 备选声明。此次仅记录算法思路与差异，未复制、翻译移植或引入任何上游源码。[FlydreamKM 文件树](https://github.com/FlydreamKM/ball-beam-project/tree/fd00eec7141624868ac081b2168b6d8ed7615cf4) · [AndreiMunteanu2911 文件树](https://github.com/AndreiMunteanu2911/Ball-Detection-Computer-Vision/tree/809dee889ab430bd1e8be4a571aeec8cab092243) · [trajec LICENSE](https://github.com/tsnyder701/trajec/blob/06873ee6a39c894bcf94614bf9bcf1e356cec1cf/LICENSE) · [许可说明](https://github.com/tsnyder701/trajec/blob/06873ee6a39c894bcf94614bf9bcf1e356cec1cf/README.md#L85-L91) · [构建元数据](https://github.com/tsnyder701/trajec/blob/06873ee6a39c894bcf94614bf9bcf1e356cec1cf/project.clj#L1-L4)

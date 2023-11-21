# 控制器通道分配

## 控制通道：
- X1：左侧直线模组（PUL，DIR，1230mm）
- X2：右侧直线模组（PUL，DIR，1230mm）
-  Y：横向直线模组（PUL，DIR，1180mm）
-  Z：纵向步进推杆（PUL，DIR，300mm）
-  C：主轴切割电机（Z/F，EN）

## 电机通道：
- CH0 - X1
- CH1 - X2
- CH2 -  Y
- CH3 -  Z

## 数字 IO 通道：
- DI0： X1 原点/负限位
- DI1： X1 正限位
- DI2： X2 原点/负限位
- DI3： X2 正限位
- DI4：  Y 原点/负限位
- DI5：  Y 正限位
- DI6：  Z 原点/负限位
- DI7：  Z 正限位
- DI8：ALM 主轴 C 报警
- DO1： EN 主轴 C 启停
- DO2：Z/F 主轴 C 方向
- DO3：LSR 激光瞄准开关
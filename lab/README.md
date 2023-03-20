# LAB Note for later TAs

因为有太多坑了，我就使用中文写了，后面如果有英语TA，劳烦翻译一下

## N3 flight controller troubleshooting

> 飞控请使用1.7.1.5 固件

### 为什么新开的N3 配置好过后无法起飞

> 因为没有设置 允许无GPS和磁力计 起飞

1. 需要将N3 连接到DJI内部的调试软件DJI assistant 2 for auto-pilot(和你能公开下载到的不是同一个)进入配置界面之前将Debug设置改为1
2. 点击 '参数表' 设置
   1. allow mag 为 1
   2. allow gps 为 1
3. 完成 就能起桨了

### 为什么新开的N3 始终无法和manifold 建立通讯

> 历史遗留问题，和串口的设置有关（具体什么因为时代太久远，我也无从查起，只是提供解决方案）

1. 文件夹下有 **N3_JCX_DRONE_PARAMETER** 文件，在DJI assistant 2 for auto-pilot(能公开下载到的) 确认版本为1.7.1.5过后，再将参数刷入
2. 连接Manifold 应该就看不到无法连接的报错了，转而变为activate 报错，此时多启动几次就好了

（这个问题我们查了两周，颇有一种在屎山代码里找金针菇的感觉💩）

### 为什么遥控器连接不上Light-bridge

> 因为遥控器是坏的(🤷) 换吧

换吧

## Manifold-2G troubleshooting

### 2G里面的代码不好使

> 不可能，如果你看到代码是基于Swarm的，并且有严重的xuhao痕迹，赶紧换成我们的镜像 我们的镜像在以下连接：

如果无效的话请联系Peize LIU(他可能会没有时间并且推销他的NxtPX4)

### 硬件问题

#### 铝制结构件

#### 脚架结构件

#### BOM 以及购买链接如下

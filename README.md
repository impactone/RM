# 以下是MEIC战队Robomaster基地车的全部代码 整套下载烧录
#MEIC_USER 中是函数入口点文件以及一些过程文件 list文件

基地车的视觉操作是
1 识别敌方装甲颜色 

2 在敌方装甲颜色靠近时停顿

3 识别出敌方大概位置(两片装连线)走正六边形规避方向

4 视觉用矩形框圈出敌方装甲并反击
#MEIC_TASK 是基地车的任务


#MEIC_SYSTEM 是UCOS系统函数装载文件


#MEIC_DRIVER 是各个传感器模块的驱动文件


#MEIC_BSP 是各个传感器模块的初始化文件

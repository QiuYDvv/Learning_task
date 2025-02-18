# 学习笔记
## 1在PID控制器的基础上使用前馈控制让实际速度更接近目标速度
### 使用前馈前误差在0.1左右
![0](https://github.com/QiuYDvv/picture/blob/master/0f.png?raw=true)
### 使用前馈后误差0.01左右
![1](https://github.com/QiuYDvv/picture/blob/master/1f.png?raw=true)
### 最终效果
![1](https://github.com/QiuYDvv/picture/blob/master/s.png?raw=true)
## 2编写控制器的步骤
### 控制器代码中导出为插件PLUGINLIB_EXPORT_CLASS
### 编写插件描述文件
### package.xml填写export标签
## 3使用rm_hw驱动电机
## 4完成效果
### 启动仿真和加载控制器
```shelll
roscore
mon launch fan_description fan.launch
```
### 启动控制器
```shell
通过rqt-controller-manager
```
### 使用前馈
```shell
controller.yaml中use_feedforward参数=1
```
### 切换速度模式
```shell
rostopic pub /controller/my_controller/command std_msgs/Int64 "data: 0"
#0为大能量机关
#1为小能量机关
```


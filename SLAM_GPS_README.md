# SLAM Trajectory to GPS Conversion - MATLAB Code

这个MATLAB代码可以将SLAM轨迹数据从局部坐标系转换为GPS坐标（经纬度）和姿态角。

This MATLAB code converts SLAM trajectory data from local coordinates to GPS coordinates (latitude/longitude) and attitude angles.

## 文件说明 (Files Description)

### 1. `slam_to_gps.m`
主函数，用于转换SLAM轨迹到GPS坐标。

Main function to convert SLAM trajectory to GPS coordinates.

**输入参数 (Inputs):**
- `data`: Nx10矩阵，列为 [timestamp, x, y, z, rx, ry, rz, rw, grav_x, grav_y, grav_z]
  - x, y, z: 米为单位的局部坐标 (x=右, y=前, z=上)
  - rx, ry, rz, rw: 四元数分量
- `start_lat`: 起始纬度（度）
- `start_lon`: 起始经度（度）
- `start_alt`: 起始海拔高度（米）（可选，默认为0）

**输出参数 (Outputs):**
- `lat`: 纬度数组（度）
- `lon`: 经度数组（度）
- `altitude`: 海拔高度数组（米）
- `roll`: 横滚角数组（度）
- `pitch`: 俯仰角数组（度）
- `yaw`: 偏航角数组（度）

### 2. `example_slam_conversion.m`
示例脚本，演示如何使用转换函数。

Example script demonstrating how to use the conversion function.

包含以下功能 (Features):
- 示例数据（来自问题描述）
- 转换为GPS坐标和姿态角
- 结果保存到文本文件
- 轨迹可视化（3D图、GPS路径图、高度剖面图、姿态角图）

## 使用方法 (Usage)

### 方法1：使用示例脚本 (Method 1: Using Example Script)

1. 在MATLAB中打开 `example_slam_conversion.m`
2. 修改起始GPS坐标：
```matlab
start_lat = 39.9042;    % 修改为你的起始纬度
start_lon = 116.4074;   % 修改为你的起始经度
start_alt = 50.0;       % 修改为你的起始海拔
```
3. 运行脚本：
```matlab
example_slam_conversion
```

### 方法2：直接调用函数 (Method 2: Direct Function Call)

```matlab
% 准备数据矩阵
data = [
    % timestamp, x, y, z, rx, ry, rz, rw, grav_x, grav_y, grav_z
    94772.623022, -0.006730, 0.049222, 0.102169, 0.051367, -0.021900, 0.002431, 0.998437, 0.000000, 0.000000, -9.809000;
    % ... 更多数据行
];

% 设置起始位置
start_lat = 39.9042;   % 纬度
start_lon = 116.4074;  % 经度
start_alt = 50.0;      % 海拔（米）

% 执行转换
[lat, lon, altitude, roll, pitch, yaw] = slam_to_gps(data, start_lat, start_lon, start_alt);

% 查看结果
disp('纬度：'); disp(lat);
disp('经度：'); disp(lon);
disp('高度：'); disp(altitude);
disp('姿态角（横滚、俯仰、偏航）：');
disp([roll, pitch, yaw]);
```

## 坐标系统说明 (Coordinate System)

### SLAM局部坐标系 (SLAM Local Coordinate System)
- X轴：向右（对应东向 East）
- Y轴：向前（对应北向 North）
- Z轴：向上（对应上向 Up）

使用ENU（东-北-上）坐标系统。

### 四元数转欧拉角 (Quaternion to Euler Angles)
使用ZYX旋转顺序转换：
- Roll（横滚）：绕X轴旋转
- Pitch（俯仰）：绕Y轴旋转
- Yaw（偏航）：绕Z轴旋转

## 输出文件 (Output File)

运行示例脚本后，会生成 `gps_trajectory_output.txt` 文件，包含：
- 时间戳
- 纬度（度）
- 经度（度）
- 海拔高度（米）
- 横滚角（度）
- 俯仰角（度）
- 偏航角（度）

## 可视化 (Visualization)

示例脚本会生成包含4个子图的图形窗口：
1. 3D局部坐标轨迹
2. GPS经纬度轨迹
3. 高度剖面图
4. 姿态角变化图

## 注意事项 (Notes)

1. 此代码使用小角度近似进行坐标转换，适用于相对较小的区域（几公里范围内）
2. 对于更大范围的轨迹，可能需要使用更精确的大地测量学转换方法
3. 地球半径使用WGS84椭球体的赤道半径（6378137米）
4. 确保你的起始GPS坐标准确，因为所有转换都基于这个参考点

## 系统要求 (Requirements)

- MATLAB R2016b或更高版本
- 无需额外工具箱

## 示例数据说明 (Sample Data Description)

提供的示例数据来自实际SLAM轨迹：
- 34个数据点
- 时间跨度约3.3秒
- 局部坐标范围约0.08米 × 0.07米 × 0.02米
- 四元数表示的方向变化

## 许可证 (License)

此代码按原样提供，可自由使用和修改。

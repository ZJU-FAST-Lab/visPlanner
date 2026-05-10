# visPlanner 中将 B 样条替换为 MINCO 的落地方案（含代码模板）

> 目标：把 `planner` 中的轨迹优化核心从 `BsplineOptimizer + UniformBspline` 迁移到 `MINCO`，并尽量降低一次性改动风险。

## 0. 现状梳理（必须先确认）

当前 B 样条相关耦合点：

- 管理器依赖 B 样条优化器与轨迹类型：
  - `src/planner/plan_manage/include/plan_manage/planner_manager.h`
  - `src/planner/plan_manage/src/planner_manager.cpp`
- FSM 发布 `traj_utils/Bspline`：
  - `src/planner/plan_manage/src/ego_replan_fsm_tracker.cpp`
- 执行端 `traj_server` 订阅 `planning/bspline` 并反序列化为 `UniformBspline`：
  - `src/planner/plan_manage/src/traj_server.cpp`
- 消息定义仍是 B 样条：
  - `src/planner/traj_utils/msg/Bspline.msg`

**建议采用两阶段迁移：**

1. **阶段 A（先跑通）**：优化器换 MINCO，但输出仍适配为旧 `Bspline.msg`，不改执行链。
2. **阶段 B（完全替换）**：新增 `MincoTraj.msg`，`traj_server` 增加 MINCO 回调，最终移除 B 样条发布链。

---


## 0.1 构建前置（CMake>=4 + ROS Noetic/catkin）

如果你本机是 **CMake 4.x**，Noetic 的 catkin 顶层 `toplevel.cmake` 仍含旧策略，可能在
`catkin_make` 配置阶段直接报：

- `Compatibility with CMake < 3.5 has been removed`

这不是 MINCO 代码问题，而是 **catkin 顶层配置与新 CMake 的兼容性冲突**。

请统一使用：

```bash
catkin_make --force-cmake --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

若你只想编译部分包（用于阶段 A/B 迭代）：

```bash
catkin_make --force-cmake --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  --pkg minco_opt --pkg traj_utils --pkg ego_planner
```

> 说明：这个参数只影响 CMake policy 兼容，不会改变你的算法逻辑。

---


### 0.2 若报错 `Could not find ... minco_optConfig.cmake`

这是 `ego_planner/plan_manage` 已经依赖 `minco_opt`，但工作区里该包未被 catkin 识别。按下面顺序排查：

```bash
# 1) 确认包目录存在
test -f src/planner/minco_opt/package.xml && echo OK || echo MISSING

# 2) 确认包名正确
grep -n "<name>" src/planner/minco_opt/package.xml
# 需要是: <name>minco_opt</name>

# 3) 清理并重配（必须带 policy 参数）
rm -rf build devel
catkin_make --force-cmake --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# 4) 若仍失败，检查被发现的包列表里是否有 minco_opt
catkin list | grep minco_opt
```

若第 1 步显示 `MISSING`，请先把 `src/planner/minco_opt` 放回工作区（包含 `CMakeLists.txt` 与 `package.xml`）。

若你暂时不想引入 MINCO，可在 `plan_manage/CMakeLists.txt` 的 `find_package(catkin REQUIRED COMPONENTS ...)` 中移除 `minco_opt`，先回退到 `bspline_opt` 路径。

---

### 0.3 CUDA 选项（local_sensing）按 GPU 设置

`uav_simulator/local_sensing` 的深度渲染默认依赖 CUDA。若你的主机 CUDA 工具链/驱动未准备好，建议先关闭 CUDA 走 CPU 路径完成 MINCO 迁移联调：

```bash
catkin_make --force-cmake --cmake-args \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DENABLE_CUDA=OFF
```

若要启用 CUDA，请根据显卡设置 `CUDA_ARCH`（例如 61/75/86）：

```bash
catkin_make --force-cmake --cmake-args \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DENABLE_CUDA=ON -DCUDA_ARCH=86
```

---

## 1. 新增 `minco_opt` 包（建议独立）

在 `src/planner/minco_opt` 新建：

### 1.1 CMakeLists.txt（模板）

```cmake
cmake_minimum_required(VERSION 3.5)
project(minco_opt)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  traj_utils
  plan_env
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES minco_opt
  CATKIN_DEPENDS roscpp traj_utils plan_env
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(minco_opt
  src/minco_optimizer.cpp
)

target_link_libraries(minco_opt
  ${catkin_LIBRARIES}
)
```

### 1.2 include/minco_opt/minco_optimizer.h（模板）

```cpp
#pragma once
#include <Eigen/Eigen>
#include <memory>
#include <vector>

namespace ego_planner {

struct MincoResult {
  // N 段 5 次多项式: p(t)=a0+a1 t+...+a5 t^5
  // coeff_x/y/z: [N x 6]
  Eigen::MatrixXd coeff_x;
  Eigen::MatrixXd coeff_y;
  Eigen::MatrixXd coeff_z;
  Eigen::VectorXd durations;  // size N
};

class MincoOptimizer {
public:
  using Ptr = std::shared_ptr<MincoOptimizer>;

  void setLimits(double max_vel, double max_acc);

  bool optimize(const Eigen::Vector3d& start_p,
                const Eigen::Vector3d& start_v,
                const Eigen::Vector3d& start_a,
                const Eigen::Vector3d& goal_p,
                const Eigen::Vector3d& goal_v,
                MincoResult& out);
};

} // namespace ego_planner
```

---

## 2. 改 `plan_manage`：把优化入口从 B 样条切到 MINCO

### 2.1 改头文件依赖

文件：`src/planner/plan_manage/include/plan_manage/planner_manager.h`

- 删除：

```cpp
#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
```

- 新增：

```cpp
#include <minco_opt/minco_optimizer.h>
```

- 成员替换：

```cpp
// BsplineOptimizer::Ptr bspline_optimizer_;
MincoOptimizer::Ptr minco_optimizer_;
```

### 2.2 初始化替换

文件：`src/planner/plan_manage/src/planner_manager.cpp`

```cpp
minco_optimizer_.reset(new MincoOptimizer);
minco_optimizer_->setLimits(pp_.max_vel_, pp_.max_acc_);
```

### 2.3 将 `reboundReplan(...)` 输出改为 MINCO 结果

在 `reboundReplan(...)` 内，调用：

```cpp
MincoResult result;
bool ok = minco_optimizer_->optimize(start_pt, start_vel, start_acc,
                                     local_target_pt, local_target_vel, result);
if (!ok) return false;

// 建议：在 LocalTrajData 增加 minco 轨迹缓存字段
// local_data_.minco_coeff_x_ = result.coeff_x; ...
// local_data_.duration_ = result.durations.sum();
```

---

## 3. 轨迹容器扩展（不破坏旧代码）

文件：`src/planner/traj_utils/include/traj_utils/plan_container.hpp`

在 `LocalTrajData` 中增加 MINCO 存储：

```cpp
struct LocalTrajData
{
  int traj_id_;
  double duration_;
  ros::Time start_time_;
  Eigen::Vector3d start_pos_;

  // 旧字段保留（阶段 A 兼容）
  UniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_;

  // 新增 MINCO 字段
  Eigen::MatrixXd minco_coeff_x_, minco_coeff_y_, minco_coeff_z_; // [N x 6]
  Eigen::VectorXd minco_durations_;                               // [N]
};
```

并新增一个求值函数（建议放 cpp）：

```cpp
inline Eigen::Vector3d evalMincoPos(const Eigen::MatrixXd& cx,
                                    const Eigen::MatrixXd& cy,
                                    const Eigen::MatrixXd& cz,
                                    const Eigen::VectorXd& ts,
                                    double t)
{
  int seg = 0;
  double acc = 0.0;
  while (seg + 1 < ts.size() && t > acc + ts(seg)) {
    acc += ts(seg);
    ++seg;
  }
  double tau = t - acc;
  Eigen::Matrix<double,6,1> p;
  p << 1.0, tau, tau*tau, tau*tau*tau, tau*tau*tau*tau, tau*tau*tau*tau*tau;

  return Eigen::Vector3d(cx.row(seg).dot(p), cy.row(seg).dot(p), cz.row(seg).dot(p));
}
```

---

## 4. FSM 发布层改造（两种方案）

文件：`src/planner/plan_manage/src/ego_replan_fsm_tracker.cpp`

### 方案 A（推荐先用）
- 仍发布 `traj_utils::Bspline`。
- 把 MINCO 轨迹先离散成点，再拟合/参数化成 B 样条控制点发布。
- 优点：`traj_server` 无改动即可运行。

### 方案 B（完全替换）
- 新增 `traj_utils/MincoTraj.msg`，发布 `planning/minco`。
- `traj_server` 新增订阅回调直接解 MINCO。

---

## 5. `traj_server` 增加 MINCO 回调（阶段 B）

文件：`src/planner/plan_manage/src/traj_server.cpp`

新增订阅：

```cpp
ros::Subscriber minco_sub = nh.subscribe("planning/minco", 10, mincoCallback,
                                         ros::TransportHints().tcpNoDelay());
```

新增回调模板：

```cpp
void mincoCallback(const traj_utils::MincoTrajConstPtr& msg)
{
  // 1) 缓存系数与段时长
  // 2) cmdCallback 中按当前 t 直接计算 pos/vel/acc
  // 3) yaw 可以沿用已有 look-ahead 逻辑
  receive_traj_ = true;
}
```

---

## 6. CMake / package.xml 需要同步的改动

### 6.1 `plan_manage/CMakeLists.txt`
- `find_package(catkin REQUIRED COMPONENTS ...)` 中将 `bspline_opt` 替换/并存为 `minco_opt`。
- `catkin_package(CATKIN_DEPENDS ...)` 同步替换。

### 6.2 `plan_manage/package.xml`
- `build_depend` / `exec_depend` 增加 `minco_opt`。
- 过渡期若保留旧路径，可同时保留 `bspline_opt`。

### 6.3 `traj_utils/CMakeLists.txt`
- 阶段 B 时新增 `MincoTraj.msg` 到 `add_message_files(...)`。

---

## 7. 参数迁移建议

把 `advanced_param_tracker.xml` 中强 B 样条参数迁到 `minco/*`，例如：

```xml
<param name="minco/limit_vel" value="$(arg max_vel)" type="double"/>
<param name="minco/limit_acc" value="$(arg max_acc)" type="double"/>
<param name="minco/lambda_smooth" value="100.0" type="double"/>
<param name="minco/lambda_collision" value="1.0" type="double"/>
```

并在 `planner_manager.cpp` 初始化时统一读取。

---

## 8. 调试顺序（很关键）

1. **只编译 `minco_opt` 与 `plan_manage`**，先消除头文件/链接错误。
2. 先跑“阶段 A”兼容发布，确认 `/position_cmd` 正常。
3. 再切“阶段 B”消息替换，验证 `traj_server` 的 MINCO 回调输出。
4. 最后再删旧 `Bspline` 路径。

---

## 9. 典型报错与对应处理

- `undefined reference to ...MincoOptimizer...`
  - `CMakeLists.txt` 未链接 `minco_opt`。
- `No such file or directory: minco_opt/minco_optimizer.h`
  - include 目录或 `catkin_package(INCLUDE_DIRS ...)` 未配置。
- `cannot convert ... to traj_utils::Bspline`
  - 仍走旧 topic，但未做 MINCO->Bspline 适配。
- `traj_server 收到轨迹但不动`
  - `start_time`、时间基准和段时长累计逻辑有误，优先检查 `cmdCallback` 中 `t_cur`。


## 10. MINCO 与 B-spline 轨迹对比（新增对比节点 + CSV）

新增节点：`ego_planner/traj_compare_node`，输入两路 `traj_utils/Bspline` 轨迹并输出统计对比：

- `mean_dist`：按采样点平均位置误差
- `max_dist`：最大位置误差
- `end_dist`：终点误差
- `len_a / len_b`：两条轨迹长度
- CSV 持久化：每次新轨迹对比结果会写入 CSV

### 10.1 从头运行（详细命令）

#### Step 0: 编译

```bash
cd ~/visPlanner
rm -rf build devel
catkin_make --force-cmake --cmake-args \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DENABLE_CUDA=OFF
source devel/setup.bash
```

#### Step 1: 跑 MINCO 版本并录包

```bash
# 终端A
roslaunch ego_planner tracking.launch

# 终端B
rosbag record -O /tmp/minco_traj.bag /drone_0_planning/bspline
```

> 同时把 `advanced_param_tracker.xml` 里的 `manager/use_minco` 设为 `true`。

#### Step 2: 跑 B-spline 版本并录包

```bash
# 终端A
roslaunch ego_planner tracking.launch

# 终端B
rosbag record -O /tmp/bspline_traj.bag /drone_0_planning/bspline
```

> 同时把 `advanced_param_tracker.xml` 里的 `manager/use_minco` 设为 `false`。

#### Step 3: 回放两路轨迹并做对比

```bash
# 终端A：启动比较节点（含CSV）
roslaunch ego_planner compare_minco_bspline.launch \
  topic_a:=/drone_0_planning/bspline_minco \
  topic_b:=/drone_0_planning/bspline_bspline \
  sample_dt:=0.05 \
  enable_csv:=true \
  csv_path:=/tmp/traj_compare.csv

# 终端B：回放MINCO包（重映射到对比节点topic_a）
rosbag play /tmp/minco_traj.bag --clock \
  /drone_0_planning/bspline:=/drone_0_planning/bspline_minco

# 终端C：回放B-spline包（重映射到对比节点topic_b）
rosbag play /tmp/bspline_traj.bag --clock \
  /drone_0_planning/bspline:=/drone_0_planning/bspline_bspline
```

#### Step 4: 查看CSV

```bash
head -n 5 /tmp/traj_compare.csv
column -s, -t /tmp/traj_compare.csv | head -n 20
```

### 10.2 CSV 字段说明

- `wall_time`: 对比发生时刻（系统时间）
- `topic_a/topic_b`: 对比的两路话题名
- `traj_id_a/traj_id_b`: 轨迹ID
- `start_time_a/start_time_b`: 两轨迹的起始时间
- `duration`: 实际对齐比较时长（取两者最小）
- `mean_dist/max_dist/end_dist`: 几何误差指标
- `len_a/len_b`: 路径长度
- `sample_dt`: 采样间隔

> 当前对比节点只做几何对比，未纳入碰撞代价/可见性代价；如需更深入评估，可再加耗时、最小障碍距离等指标。

---



## 11. 两无人机与障碍物防撞验证（点云地图）

新增节点：`collision_safety_monitor.py`，基于 `/global_map` 点云和两机 odom 进行实时最小距离检查。

- 输入：
  - 点云：`/global_map`（可改）
  - 无人机里程计：`/drone_0_visual_slam/odom`、`/drone_1_visual_slam/odom`（可改）
- 输出：
  - 话题：`/collision_safety_monitor/collision_risk` (`std_msgs/Bool`)
  - 控制台：`SAFE` 或 `RISK` 日志，含 `d01/d0_obs/d1_obs` 与阈值

### 11.1 从头运行命令（建议直接复制）

```bash
cd ~/visPlanner
catkin_make --force-cmake --cmake-args \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DENABLE_CUDA=OFF
source devel/setup.bash

# 终端A：启动仿真与规划
roslaunch ego_planner tracking.launch

# 终端B：启动防撞监测（点云地图验证）
roslaunch ego_planner monitor_collision_safety.launch \
  map_topic:=/global_map \
  odom0_topic:=/drone_0_visual_slam/odom \
  odom1_topic:=/drone_1_visual_slam/odom \
  drone_radius:=0.35 \
  obstacle_clearance:=0.70 \
  inter_drone_clearance:=1.20 \
  check_rate:=2.0 \
  point_stride:=5 \
  max_check_distance:=8.0
```

### 11.2 如何判断是否通过

```bash
# 查看风险布尔值（true 表示风险）
rostopic echo /collision_safety_monitor/collision_risk

# 检查地图点云是否存在
rostopic hz /global_map

# 检查两机 odom 是否存在
rostopic hz /drone_0_visual_slam/odom
rostopic hz /drone_1_visual_slam/odom
```

若持续输出 `SAFE` 且 `collision_risk` 长时间为 `false`，可认为在当前阈值下通过验证。

---

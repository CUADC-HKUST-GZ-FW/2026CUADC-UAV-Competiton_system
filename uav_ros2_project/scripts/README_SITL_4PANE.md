# UAV SITL 四分屏工具

## 作用

一条命令启动完整 SITL 测试环境：

```text
左上：ArduPlane SITL / MAVProxy
右上：MAVROS
左下：项目节点
右下：Mission Control
```

工具会自动完成：

```text
启动 SITL
→ 启动 MAVROS
→ 启动项目节点
→ 加载航点
→ ARM
→ AUTO
→ 等待 MissionManager 进入 STANDBY
→ 自动 enable + confirm
```

---

## 第一次使用

```bash
chmod +x scripts/install_sitl_tools.sh
./scripts/install_sitl_tools.sh
source ~/.bashrc
```

安装后可使用：

```bash
uav_start4
uav_restart4
uav_stop4
uav_attach4
uav_config4
```

---

## 修改测试配置

执行：

```bash
uav_config4
```

主要修改这 3 项：

```bash
# 参数文件
export UAV_PARAM_FILE="..."

# 航点文件
export UAV_WAYPOINT_FILE="..."

# SITL 起飞点
export UAV_CUSTOM_LOCATION="纬度,经度,高度,航向"
```

例如：

```bash
export UAV_PARAM_FILE="${HOME}/uav_param_filter/filter_result_v3/params_to_import.param"

export UAV_WAYPOINT_FILE="${UAV_ROS_WS}/wp/0826全流程.waypoints"

export UAV_CUSTOM_LOCATION="22.8817230,113.4882814,20,0.0"
```

以后换航点、起飞点或参数文件，**只改这个配置文件，不要改启动脚本。**

---

## 启动

```bash
uav_start4
```

重启全部环境：

```bash
uav_restart4
```

停止：

```bash
uav_stop4
```

重新进入正在运行的 tmux：

```bash
uav_attach4
```

---

## Mission Control

视觉目标由项目自动产生：

```text
result.json
→ vision_target_bridge
→ MissionManager
```

Pane 4 不会手动发布新的目标。

它会等待日志出现：

```text
pending target activated target cached
```

然后自动执行：

```text
enable false
→ enable true
→ confirm_target
```

之后任务进入：

```text
STANDBY
→ MISSION_READY
→ EXECUTING
```

---

## Git 使用规则

Git 中提交：

```text
scripts/start_uav_sitl_4pane.sh
scripts/stop_uav_sitl_4pane.sh
scripts/install_sitl_tools.sh
config/sitl_local.env.example
```

每个人自己的：

```text
config/sitl_local.env
```

不要提交 Git。

这样不同队员可以使用不同的：

```text
航点文件
参数文件
起飞位置
本机路径
```

而不会互相影响。

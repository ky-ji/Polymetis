# Franka 可复现运行时与真机验收

本目录构建独立硬件客户端；`../server` 构建控制服务端；`../servo` 构建 Ruckig/TorchScript 参考轨迹插件。服务端与硬件端分别打包，禁止共享同一个 LD_LIBRARY_PATH。旧 Conda 环境只作为明确选定的构建依赖，不写入它的 bin、lib 或 site-packages。

## 已实现

- 精确链接 libfranka 0.18.2；硬件端使用与其重编译匹配的 Pinocchio。服务端保留独立的 Torch 1.13 / Pinocchio 2.5 依赖集合。
- 0.18.0 会把含加速度计支链的机器人模型中的错误 frame 当作法兰；官方 0.18.1 修复了该问题。新客户端在设置碰撞参数或进入控制前，核对模型计算与机器人反馈的末端位姿，超过 1 mm 位置误差或旋转矩阵元素误差 0.001 即拒绝。该核对不能替代物理 TCP 与负载标定。
- 原生 RPC 有截止时间，非有限力矩和错误维度会失败；同一限位重复越界仍然抛错。默认关闭自动错误恢复。
- 服务端在访问反馈向量前检查维度与有限值，错误客户端不会因空向量导致进程崩溃。
- 1 kHz 关节参考保持位置、速度、加速度连续，限制 jerk。每个关节独立规划。社区版 Ruckig 的位置界限由完整轨迹极值检查补足；无云调用。
- 输入断流锁存、减速，到停止后结束控制策略；新的输入包不能自行清除故障。显式 deadman 暂停与意外断流区分处理。
- 五秒环形故障记录：原始 RobotState、计算／应用力矩、参考轨迹、周期、RPC 耗时、libfranka 异常尾帧。记录写盘在控制回调退出后执行。
- 提供 FR3 裸臂参考模型和来源；它不是已经标定的夹具／相机模型，也不用于碰撞几何。
- 夹爪客户端支持 `readonly: true`，此模式不发送运动命令；启动回零改为显式 `home_on_start: true`，默认不回零，两者不能同时开启。反馈按单调时钟运行于 30 Hz；RPC 有截止时间，异步命令使用独立副本和完整时间戳。

## 从干净目录构建

准备锁文件指定的官方 libfranka 源码（包含固定版本 common 子模块）和 Ruckig 源码。`dependencies.lock.json` 同时校验提交标识和实际源码树哈希。归档构建只给 libfranka 的打包版本增加显式入口，不伪造 Git 历史。

```bash
python3 polymetis/native/build_runtime.py \
  --deps-prefix "$SERVER_DEPS" --model-prefix "$MODEL_DEPS" \
  --libfranka-source "$FRANKA_SOURCE" --ruckig-source "$RUCKIG_SOURCE" \
  --output "$NEW_BUILD_DIRECTORY" --jobs 2
```

脚本在新目录编译、运行原生与轨迹测试、分别冻结动态库依赖，并运行仅使用合成反馈的服务端往返测试。实际加载库必须闭合；版本化 SONAME 不匹配会拒绝打包。MKL 等供应商未版本化的链接名会保留其实际解析文件，不被误判为旧版本 ABI 兼容证明。

`source-manifest.json` 记录输入源码树、依赖锁和构建命令。`native-bundle/manifest.json`、`server-bundle/manifest.json` 记录二进制和动态库 SHA-256。调用者必须用各自 bundle/lib 启动对应进程。Python 上传端使用新生成 protobuf 路径优先于 python 源目录，并显式设置 `POLYMETIS_TORCH_LIB_DIR` 指向服务端 bundle/lib；该变量指定错误时不会退回旧库。

## 真机第一步：只读核对

现场人员就位并完成工作空间检查后，先用原工作站退出流程停止旧采集／FCI 服务。不要同时运行两个 FCI 客户端。

```bash
LD_LIBRARY_PATH="$NATIVE_BUNDLE/lib" "$NATIVE_BUNDLE/bin/inspect_franka" \
  "$ROBOT_ADDRESS" "$NEW_INSPECTION_DIRECTORY"
```

该工具不调用 control、automaticErrorRecovery、setLoad 或碰撞阈值设置。输出机械臂自身模型、当前关节、TCP 变换、末端质量／惯量、原生动力学结果；`fci_protocol_version` 不是 Desk 系统镜像版本。系统版本仍应从 Desk 核实。

`prepare_station_model.py INSPECTION_DIRECTORY NEW_MODEL_DIRECTORY` 从通过位姿核对的快照导出机器人返回的模型，并加入登记负载与 TCP。不使用通用 Panda 或另一代 FR3 的惯量替代该模型。导出结果始终保留 `model_verified=false` 和 `payload_physically_verified=false`；控制器登记的值不证明已覆盖真实相机、支架、夹指、抓取物或线缆作用。

用服务端模型独立计算同一状态的末端位姿、雅可比、重力与质量矩阵，与原生快照对照。原生 `model_frames` 测试包含加速度计支链、独立串联运动学和有限差分雅可比；导出工具测试可执行 `python3 -m unittest discover -s polymetis/native -p 'test_prepare_station_model.py'`。

对齐实际机器人型号、关节范围、模型与 TCP 后重新生成元数据、Vela profile 和 servo config。通过误差检查并完成负载标定后才将 `model_verified` 设为 true。保留碰撞检测；不以提高碰撞阈值掩盖错误负载。软件工作空间界限必须与真实 TCP 一致，不能把 flange 与 EE 当成同一个点。

## 验收顺序与边界

1. 无使能验证：模型哈希、Python import 路径、库清单、串口唯一占用、GELLO 新鲜样本频率、相机帧与状态时间戳。
2. 现场低速、小范围跟随，验证 deadman、断流、失联、停止与重新开启会话。停止操作不应恢复之前的运动目标。
3. 同一段操作对比原始输入、1 kHz 参考与实测关节；记录延迟分布、跟踪误差、限幅、控制周期和通信成功率。逐级调整速度、加速度和关节阻抗。
4. 完成快速动作采集再评估任务成功率。离线合成反馈不证明真机稳定性或全机械臂无碰撞。

当前轨迹生成器保证参考的关节运动学界限，不能保证机器人实际跟踪误差，也不能保证整个轨迹的笛卡尔碰撞自由。原生硬限制和碰撞反射仍保留。主动触觉／主端阻尼需要主端电机控制或机械改造；被动 GELLO 的 torque-off 读取链路不会凭软件产生力反馈。

若验收不通过，停止新进程后用原工作站启动器恢复旧版本（保持 manual-enable），新旧源码和配置均保留，禁止跨版本复制单个 .so 或重新创建 ABI 软链接。

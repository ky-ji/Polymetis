# 连续采集运行时整合

本版本将 `d262492ad202ef52dedcc31afa37e2bb5ea46bc8` 的连续 servo 与
`3a131908308baf3b45b75b29b653cc6e02c3ef5a` 的夹爪命令恢复合并。
二者修改不同文件，整合不改变已实现的控制逻辑。
应用层 GELLO 输入、标定、边界映射、复位和录制仍由 Vela-Franka 维护。

## 夹爪命令失败

`pollGripperCommand` 只接住 `franka::CommandException`：本次命令标记失败，
清除 moving 状态，继续读取并发送真实夹爪反馈。客户端不会自动重发 grasp；
下一次显式命令仍可执行。网络断连等其他异常继续上抛，不伪装成有效反馈。
`gripper_command_test` 覆盖等待、拒绝、下一次成功、返回 false 和网络异常。

## 版本与构建

现有 r25 服务器使用 `d262492` 构建。原生 bundle 基线为 `dd9aa093d36ac13811987609193fc3e317fb023c`，
其中 `bin/franka_hand_client` 来自 `3a131908`。合并源码尚未被重新构建为现场统一 bundle，
不能把本次仓库 HEAD 写成旧二进制的构建来源。

下一次构建统一使用本分支，通过 `build_runtime.py` 生成新的输出目录、source manifest
和两份 bundle manifest。按 README 的依赖锁和合成反馈检查完成构建后，再单独安排切换。
保留每个组件实际构建提交及 SHA-256；不要把旧 .so 混入新 bundle。

`lookahead_s` 缺省为 0；启用时同步部署 Python 策略与原生库。参考轨迹约束不等于实测
关节速度、跟踪误差或碰撞保证。站点增益、负载、关节范围和 55 ms 前瞻不是通用默认值。
完整站点配置与运行哈希保存在私有部署记录中，不放入本公共仓库。

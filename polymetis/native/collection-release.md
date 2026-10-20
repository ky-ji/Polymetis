# FR3 数采运行时与组件来源

历史 r25 整合将 `d262492ad202ef52dedcc31afa37e2bb5ea46bc8` 的连续 servo 与
`3a131908308baf3b45b75b29b653cc6e02c3ef5a` 的夹爪命令恢复合并。
二者修改不同文件，整合不改变已实现的控制逻辑。
应用层 GELLO 输入、标定、边界映射、复位和录制仍由 Vela-Franka 维护。

## 当前整合：双输入与姿态保持

应用侧在 GELLO 与 SpaceMouse 之间显式切换控制器。此仓库新增 XYZ 速度／姿态保持
策略、连续工作空间恢复、姿态优先的力矩预算，并修复首次参数更新在控制锁内触发
JIT 优化的问题。默认 RobotInterface 控制方式不变，站点通过配置选择策略。

r37 已部署组件并非同一次构建，必须分别记录：

| 组件 | 实际源码提交 |
| --- | --- |
| Python 控制器（含姿态优先） | `98063e85c404ec9149a3f615a532f4ef3f044c1b` |
| server 参数更新修复 | `95864db8e1a8c9c32efccfc3edc7c7b6a9fa2995` |
| 关节 servo 动态库 | `d262492ad202ef52dedcc31afa37e2bb5ea46bc8` |
| native 基线 | `dd9aa093d36ac13811987609193fc3e317fb023c` |
| native gripper 替换组件 | `3a131908308baf3b45b75b29b653cc6e02c3ef5a` |

合并 GitHub 分支不重启现场，也不把新 HEAD 追认成既有二进制来源。每个运行文件的
SHA-256 与 bundle manifest 保存在私有站点归档；Python 记录源码的 75 个文件均与
上述提交匹配。各 bundle 内沿用组件以文件哈希识别，不能仅凭顶层版本名推断。

针对姿态预算的 14 项策略／TorchScript 检查、合成反馈服务端测试，以及 40 组静态
记录姿态上的输出预算检查已完成。合成反馈首次更新最大 ControlUpdate 耗时约
2.315 ms；这不是手柄到真机的端到端延迟。动态翘边改善尚无量化验收结果。

控制算法及验证入口见 [servo 说明](../servo/README.md)。

## 夹爪命令失败

`pollGripperCommand` 只接住 `franka::CommandException`：本次命令标记失败，
清除 moving 状态，继续读取并发送真实夹爪反馈。客户端不会自动重发 grasp；
下一次显式命令仍可执行。网络断连等其他异常继续上抛，不伪装成有效反馈。
`gripper_command_test` 覆盖等待、拒绝、下一次成功、返回 false 和网络异常。

## 历史 r25 与下一次构建

现有 r25 服务器使用 `d262492` 构建。原生 bundle 基线为 `dd9aa093d36ac13811987609193fc3e317fb023c`，
其中 `bin/franka_hand_client` 来自 `3a131908`。合并源码尚未被重新构建为现场统一 bundle，
不能把本次仓库 HEAD 写成旧二进制的构建来源。

下一次构建统一使用本分支，通过 `build_runtime.py` 生成新的输出目录、source manifest
和两份 bundle manifest。按 README 的依赖锁和合成反馈检查完成构建后，再单独安排切换。
保留每个组件实际构建提交及 SHA-256；不要把旧 .so 混入新 bundle。

`lookahead_s` 缺省为 0；启用时同步部署 Python 策略与原生库。参考轨迹约束不等于实测
关节速度、跟踪误差或碰撞保证。站点增益、负载、关节范围和 55 ms 前瞻不是通用默认值。
完整站点配置与运行哈希保存在私有部署记录中，不放入本公共仓库。

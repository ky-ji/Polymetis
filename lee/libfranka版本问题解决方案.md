# Polymetis libfranka 版本不兼容问题解决方案

## 问题现象

启动 Polymetis 服务器时出现错误：

```
terminate called after throwing an instance of 'franka::IncompatibleVersionException'
what(): libfranka: Incompatible library version (server version: 9, library version: 5)
```

## 问题原因

**版本不匹配：**
- Polymetis 自带的 libfranka：0.9.0（支持 FCI version 5）
- Franka 机械臂固件：FCI version 9
- 需要 libfranka 0.14.0+ 才能支持 FCI version 9

## libfranka 版本与 FCI 版本对应关系

| libfranka 版本 | FCI Server Version | 兼容性 |
|----------------|-------------------|--------|
| 0.9.x          | 5                 | ✗ 不兼容 |
| 0.13.x         | 7-8               | ✗ 不兼容 |
| 0.14.x+        | 9                 | ✓ 兼容 |
| 0.15.x         | 9                 | ✓ 兼容 |

参考：https://frankaemika.github.io/docs/compatibility.html

---

## 诊断步骤

### 1. 检查系统中的 libfranka 版本

```bash
# 查找所有 libfranka 库
find /home/$USER -name "*libfranka*" -type f 2>/dev/null

# 查看 Polymetis 环境中的版本
ls -lh /home/$USER/anaconda3/envs/polymetis/lib/libfranka*

# 查看独立编译的版本
ls -lh /home/$USER/Desktop/Workspace/libfranka*/build/libfranka.so*
```

**诊断结果示例：**
```
Polymetis 环境:
-rwxrwxr-x libfranka.so.0.9.0   (414K)  ← 旧版本

独立编译:
-rwxrwxr-x libfranka.so.0.13.3  (522K)  ← 中间版本
-rwxrwxr-x libfranka.so.0.15.0  (839K)  ← 新版本（推荐）
```

### 2. 检查当前 Polymetis 使用的版本

```bash
# 查看符号链接指向
ls -lh /home/$USER/anaconda3/envs/polymetis/lib/libfranka.so*

# 检查可执行文件依赖
ldd /home/$USER/anaconda3/envs/polymetis/bin/franka_panda_client | grep franka
```

### 3. 测试启动服务器（看错误信息）

```bash
conda activate polymetis
cd /home/$USER/Desktop/Workspace/robot/polymetis
python polymetis/python/scripts/launch_robot.py robot_client=franka_hardware
```

根据错误信息中的 `library version: X` 判断当前使用的版本。

---

## 解决方案

### 步骤 1: 备份旧版本

```bash
cd /home/$USER/anaconda3/envs/polymetis/lib
cp libfranka.so.0.9.0 libfranka.so.0.9.0.backup
echo "✓ 已备份原版本"
```

### 步骤 2: 复制新版本库文件

```bash
# 使用 libfranka 0.15.0（推荐）
cp /home/$USER/Desktop/Workspace/libfranka/build/libfranka.so.0.15.0 \
   /home/$USER/anaconda3/envs/polymetis/lib/

echo "✓ 已复制 libfranka 0.15.0"
```

### 步骤 3: 更新符号链接

```bash
cd /home/$USER/anaconda3/envs/polymetis/lib

# 删除旧的符号链接
rm -f libfranka.so libfranka.so.0.9 libfranka.so.0.13

# 创建新的符号链接
ln -s libfranka.so.0.15.0 libfranka.so.0.15
ln -s libfranka.so.0.15 libfranka.so.0.9      # franka_panda_client 需要 0.9
ln -s libfranka.so.0.15 libfranka.so

# 验证
ls -lh libfranka.so*
```

**预期输出：**
```
lrwxrwxrwx libfranka.so -> libfranka.so.0.15
lrwxrwxrwx libfranka.so.0.15 -> libfranka.so.0.15.0
-rwxrwxr-x libfranka.so.0.15.0  (839K)
lrwxrwxrwx libfranka.so.0.9 -> libfranka.so.0.15
-rwxrwxr-x libfranka.so.0.9.0.backup  (414K)  ← 备份
```

### 步骤 4: 测试新版本

```bash
# 清理旧进程
sudo pkill -9 run_server
sleep 1

# 启动服务器
conda activate polymetis
cd /home/$USER/Desktop/Workspace/robot/polymetis
python polymetis/python/scripts/launch_robot.py robot_client=franka_hardware
```

**成功标志：**
```
[info] Connecting to Franka Emika...
[info] Connected.
[info] Server listening on 0.0.0.0:50051
```

**不再出现版本不兼容错误！**

---

## 使用方法（升级后）

### 终端 1：启动 Polymetis 服务器

```bash
conda activate polymetis
cd /home/$USER/Desktop/Workspace/robot/polymetis
python polymetis/python/scripts/launch_robot.py robot_client=franka_hardware
```

保持运行，看到 `Connected.` 表示成功。

### 终端 2：运行控制程序

```bash
conda activate polymetis
cd /home/$USER/Desktop/Workspace/robot/polymetis/lee
python keyboard_control.py
```

---

## 故障排除

### 问题 1: 找不到 libfranka.so.0.9

**错误：**
```
error while loading shared libraries: libfranka.so.0.9: cannot open shared object file
```

**解决：**
```bash
cd /home/$USER/anaconda3/envs/polymetis/lib
ln -s libfranka.so.0.15.0 libfranka.so.0.9
```

### 问题 2: 仍然报版本不兼容

**检查：**
```bash
# 确认 franka_panda_client 使用的库版本
ldd /home/$USER/anaconda3/envs/polymetis/bin/franka_panda_client | grep libfranka

# 应该指向新版本
libfranka.so.0.9 => .../lib/libfranka.so.0.9 (0x...)
```

### 问题 3: 端口被占用

**错误：**
```
AssertionError: Port unavailable
```

**解决：**
```bash
sudo pkill -9 run_server
sleep 2
```

---

## 回滚到原版本（如果需要）

```bash
cd /home/$USER/anaconda3/envs/polymetis/lib

# 恢复备份
rm libfranka.so libfranka.so.0.9 libfranka.so.0.15
cp libfranka.so.0.9.0.backup libfranka.so.0.9.0
ln -s libfranka.so.0.9.0 libfranka.so.0.9
ln -s libfranka.so.0.9 libfranka.so

echo "✓ 已回滚到原版本"
```

---

## 技术细节

### 为什么需要 libfranka.so.0.9 符号链接？

`franka_panda_client` 可执行文件在编译时链接了 `libfranka.so.0.9`，运行时会查找这个特定版本。通过创建符号链接 `libfranka.so.0.9 → libfranka.so.0.15.0`，可以在不重新编译的情况下使用新版本库。

### 符号链接结构

```
libfranka.so              ← 通用链接
    ↓
libfranka.so.0.15         ← 主版本链接
    ↓
libfranka.so.0.15.0       ← 实际库文件
    ↑
libfranka.so.0.9          ← 兼容性链接（franka_panda_client 需要）
```

---

## 验证清单

- [x] 备份了原 libfranka 0.9.0
- [x] 复制了 libfranka 0.15.0
- [x] 创建了正确的符号链接
- [x] franka_panda_client 能找到库文件
- [x] 服务器成功连接到机械臂
- [x] 客户端程序能连接到服务器

---

## 参考资料

- Franka Emika 兼容性文档: https://frankaemika.github.io/docs/compatibility.html
- libfranka GitHub: https://github.com/frankaemika/libfranka
- Polymetis 文档: https://facebookresearch.github.io/fairo/polymetis/

---

**最后更新：** 2025-11-22  
**适用版本：** Polymetis (conda 安装), Franka FCI version 9

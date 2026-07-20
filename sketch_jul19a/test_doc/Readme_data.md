当前工程里的数据处理，实际上是分成两条线的：

1. `BU04 串口1` 走 USB 调试/命令通道
2. `BU04 串口2` 走真正的数据通道，交给当前编译模式去处理

默认模式现在还是 `TCP`，因为 [src/AppMode.h](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/AppMode.h#L5) 里 `BU04_APP_USE_FOLLOW = 0`。

**整体数据流**

- 主入口在 [sketch_jul19a.ino](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/sketch_jul19a.ino#L16)
- `Serial1` 被映射成 BU04 命令/调试口
- `Serial2` 被映射成 BU04 PDOA/TWR 数据口
- `loop()` 里：
  - `console.update(bu04Cmd)` 处理 USB 输入和串口1镜像
  - `appController.update(bu04Data)` 处理串口2数据

---

**1. 串口1现在怎么处理**

串口1主要是给你发 AT 命令、看 BU04 回复。

- 构造在 [sketch_jul19a.ino](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/sketch_jul19a.ino#L16-L20)
- USB 输入在 [src/bu04/Bu04Console.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/bu04/Bu04Console.cpp#L11-L35) 里读取
- 如果开启 `mirror`，会把 `Serial1` 的返回内容通过 USB 打出来
- 命令行模式里还能直接发：
  - `pass on/off`
  - `mirror on/off`
  - `ping`
  - `base`
  - `tagrole`
  - `addtag`
  - `uwbmode`
  - `dump`

所以串口1现在更像“控制口 + 回显口”，不是主数据处理口。

---

**2. 串口2在 TCP 模式下怎么处理**

这是当前默认的数据处理链路。

入口在 [src/net/Bu04PdoaBridge.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/net/Bu04PdoaBridge.cpp#L39-L51)。

流程是：

- 先 `ensureWifi()`
- 再 `ensureServer()`
- 然后从 `bu04Data` 逐字节读入
- 识别帧头 `JS`
- 读 4 位十六进制长度
- 再读 payload
- payload 里如果包含 `"TWR"` 就解析字段

字段解析在这里：

- [src/net/Bu04PdoaBridge.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/net/Bu04PdoaBridge.cpp#L237-L266)

当前会解析这些字段：

- `a16`
- `R`
- `T`
- `D`
- `P`
- `Xcm`
- `Ycm`
- `O`
- `V`
- `X`
- `Y`
- `Z`

---

**3. TCP 模式下的数据滤波和发送逻辑**

TCP 模式不是“收到一帧就发一帧”，中间还有一层聚合。

### 3.1 16 帧一组
- [src/net/Bu04PdoaBridge.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/net/Bu04PdoaBridge.cpp#L284-L331)
- 累积到 16 帧后触发 `flushAveragedGroup()`

### 3.2 只对中间 10 帧做均值
- 取第 4 到第 13 帧，也就是索引 `3..12`
- 平均字段：
  - `D`
  - `P`
  - `Xcm`
  - `Ycm`
  - `X`
  - `Y`
  - `Z`
- `T` 和 `a16` 保留最后一帧的数据

### 3.3 角度计算
- 平均后的 `Xcm/Ycm` 会算角度
- 公式在 [src/net/Bu04PdoaBridge.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/net/Bu04PdoaBridge.cpp#L269-L281)
- 当前用的是：
  - `atan(xcm / ycm) * 180 / PI`

### 3.4 队列缓存
- [src/net/Bu04PdoaBridge.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/net/Bu04PdoaBridge.cpp#L333-L346)
- 队列长度是 3
- 新数据进来时，如果队列满了，会丢最旧的一条

### 3.5 每 1 秒发送一次
- [src/net/Bu04PdoaBridge.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/net/Bu04PdoaBridge.cpp#L348-L369)
- 只要服务器连着，就每 1000 ms 发送队头数据
- 发送内容是一个简化 JSON：
  - `type`
  - `a16`
  - `T`
  - `Xcm`
  - `Ycm`
  - `D`
  - `angle`

---

**4. 跟随模式下的数据处理不同**

如果把 `BU04_APP_USE_FOLLOW` 改成 `1`，主处理器会变成 follow 路径。

入口是：

- [sketch_jul19a.ino](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/sketch_jul19a.ino#L8-L13)

follow 的处理在 [src/follow/UwbFollowRest.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/follow/UwbFollowRest.cpp#L60-L78)。

它的流程更像“跟随控制”而不是“简单转发”：

- 解析同样的 BU04 帧
- 只重点用：
  - `T`
  - `D`
  - `Xcm`
  - `Ycm`
- 对输入做节流，默认 200 ms 一次
- 对 `Xcm/Ycm` 做：
  - 跳变限制
  - EMA 平滑
  - 速度估计
- 根据 `D` 判断是否太近、是否停止跟进
- 向底盘 REST API 查询当前位置
- 把 tag 坐标转换成地图坐标
- 组装目标点进入队列
- 按策略下发 `MoveTo` 动作
- 动作运行中还会根据新点做重规划

相关关键位置：

- 帧解析: [src/follow/UwbFollowRest.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/follow/UwbFollowRest.cpp#L190-L218)
- 平滑与速度估计: [src/follow/UwbFollowRest.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/follow/UwbFollowRest.cpp#L305-L334)
- 入队过滤: [src/follow/UwbFollowRest.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/follow/UwbFollowRest.cpp#L388-L406)
- 动作派发: [src/follow/UwbFollowRest.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/follow/UwbFollowRest.cpp#L424-L469)

---

**5. 现在你能看到什么输出**

现在 USB 上你能看到三类信息：

- 你输入给 BU04 串口1 的命令
- BU04 串口1 的回包，前提是 `mirror on`
- 当前模式的状态日志：
  - TCP 模式是 [src/net/Bu04PdoaBridge.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/net/Bu04PdoaBridge.cpp#L396-L424)
  - follow 模式是 [src/follow/UwbFollowRest.cpp](/E:/Arduino_Project/0719_PDOA/sketch_jul19a/src/follow/UwbFollowRest.cpp#L511-L525)

---

**6. 一句话总结**

当前工程的数据处理链路是：

- `Serial1` = 控制 BU04 + 可镜像回显到 USB
- `Serial2` = 接收 BU04 PDOA/TWR 原始数据
- `TCP 模式` = 16 帧分组、取中间 10 帧均值、算角度、3 队列缓存、1 秒一发
- `follow 模式` = 解析后做滤波、速度估计、坐标转换、REST 下发跟随动作

如果你愿意，我下一步可以直接给你画一张“串口1 / 串口2 / TCP / follow”的流程图，方便你一眼看懂整套数据走向。
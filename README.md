# 麦克纳姆轮四驱底盘：控制核 + PC 台架 + STM32H743 移植

四轮麦轮底盘（WS 系列低压伺服驱动器，RS485 / Modbus RTU），用手柄或键盘遥控。
仓库里有两套同源实现：**PC 侧 Node 控制核**（已在实机跑通）和**移植到 STM32H743 的 C 控制核**
（已写完、编译通过、整镜像命令行链接通过，**尚未在板子上运行过**）。

## 先说两条安全前提（不是免责声明，是硬件事实）

- **这台驱动器没有"通讯中断自动停车"功能**。链路断了、程序崩了，电机就保持最后收到的速度一直转。
  所以上位侧必须有看门狗：输入超过 400 ms 没更新 → 四轮立即写 0；连续 3 次总线交互失败 → 故障闭锁，
  只能显式解除。物理总开关和急停始终在人手里。
- **速度寄存器 `Pn-303` 是掉电保持参数，手册故障表里有"EEPROM 超出写入次数"**。所以控制核只在
  量化后的目标值发生变化时才写，并且带 250 ms 限频；稳态行驶是真正的零写入（实机 6400 拍验证过：
  写入 92 次、失败 0 次）。任何改动都不能把这条改成"每拍都写"。

另外：电机与轮子之间有减速（粗测 30:1，误差 ±10%），所以**驱动器自报的转速全是电机侧的量**，
判"轮子有没有转"不能靠肉眼，要看 `0x0706 / 0x0707` 的回读。车速换算直接乘这个比值，比值错车速就错。

## 快速开始（PC 侧）

```
cd control
npm install          # 唯一依赖是 serialport，Linux/Windows 都有预编译原生二进制
npm test             # 130 条单测，全程不碰硬件
npm run table        # 只打印四轮指令对照表（离线，最安全，先看这个）
node src/cli.js --list-ports    # 枚举系统串口并和映射对照，不 open 任何口
node src/cli.js --preflight     # 只读体检四轮状态，只发读帧、不下发任何指令
node src/cli.js --mps 0.5       # 会动车：预检通过后进入 20 Hz 控制环
```

串口名不写死在代码里，由 `control/src/ports.js` 解析，优先级：
单轮 `CHASSIS_PORT_FL/_FR/_BL/_BR` > 整串平移 `CHASSIS_PORT_FIRST=<1 号口名>` > 平台默认
（Windows `COM12/13/14/15`，Linux `/dev/ttyCH343USB0..3`）。换编号请用环境变量，不要改源码。
Linux 侧需要先装 WCH 官方的 `ch343` 串口内核模块，并把用户加进 `dialout` 组。

要改驱动器限幅（`Pn-30D`，EEPROM 参数）走单独入口：`node src/setceiling.js`，默认只打印计划，
必须显式 `--confirm <值>` 才写。**控制环入口永远不写参数**，这是有意的分层。

## 目录

| 路径 | 内容 |
| --- | --- |
| `control/src/` | JS 控制核：`config.js` 标定常量、`kinematics.js` 麦轮正逆解、`limits.js` 量化/斜坡/限频、`safety.js` 看门狗与闭锁、`controller.js` 20 Hz 环、`bus.js` Modbus RTU 主站、`ports.js` 串口名解析 |
| `control/public/` | 网页手柄面板（Gamepad API，20 Hz `POST /twist`） |
| `control/test/` | 130 条单测（`node --test`，不需要串口硬件） |
| `firmware/core/` | C 控制核，与 JS 逐模块对应 |
| `firmware/port/` | 与芯片无关的传输层 + H743 板级三层（引脚表 / UART 适配 / 起环） |
| `firmware/test/` | 173 条 C 断言，期望值从已实机验证的 JS 核算出来抄写，兼作"翻译没走样"的证据 |
| `firmware/tools/` | 命令行编译检查与整镜像链接脚本（用 Keil 的 armclang/armasm/armlink） |
| `*.md` | 现场操作流程与点动测试教程 |

## 固件侧（需要 Keil MDK，工具链路径写死在脚本里）

```
powershell -File firmware/tools/check_build.ps1     # 只编译检查 core + port + test
powershell -File firmware/tools/build_board.ps1     # 整镜像：编译 + armlink 链接 + fromelf 出 bin/hex
```

两个脚本都**不烧写**。`build_board.ps1` 需要 `firmware/board/` 里的板级模板（HAL/CMSIS 源码树和
Keil 语法的启动文件）—— 那个目录故意没有进这个仓库，原因见下一节，所以新克隆的机器上这一步跑不起来。

## 有意没放进仓库的东西

- `firmware/board/`：野火（EmbedFire）H743 例程模板树。ST 的 HAL/CMSIS 是 BSD-3，可以再分发但要保留
  版权声明；野火自己的 `bsp_*.c` 只带 `@author embedfire` 的文件头、没有附授权条款，整套搬进公开仓库是个
  灰色地带。需要板级工程时从野火官方资料里取，然后把我这几个文件放回去：`User/main.c`（改成起
  `chassis_app_boot()/chassis_app_tick()`）、`tools/board_h743.sct`（内存布局逐字抄自模板自己生成的那份
  scatter file）。
- 野火官方例程包（解包 1.3 GB）与它的 zip、`docs/` 下三份厂商手册的抽取全文 —— 版权与体积。
- `logs/*.csv` 与标定记录 —— 实验原始数据。
- `node_modules/`、Keil 产物（`Output/`、`Objects/`、`Listings/`、`*.axf` 等）。

想放开哪条，删 `.gitignore` 里对应那行即可；但那个 1.3 GB 的例程包一旦提交就得重写历史才能删干净。

## 已验证 / 未验证的边界

已验证（实机）：四轮符号矩阵（前轮 +、后轮 −）、0.5 m/s 档四档逐档离地跑、稳态零写入、
看门狗与急停闭锁、四口并行 20 Hz。
未验证：C 控制核的运行时行为（这台机器上没有宿主机 C 编译器，173 条断言一次都没跑过；
armclang 只证明了类型与语法）；整镜像链接过但不等于跑得对；第二路 RS485 的 UART 与方向脚还没定，
板上四个通道没凑齐时 `chassis_app_boot()` 会直接返回、整台车不动（宁可不动，也不要"只有一只能动"的
麦轮车原地横着拧）；1.0 m/s 以上、以及落地行走都属于未验证区。

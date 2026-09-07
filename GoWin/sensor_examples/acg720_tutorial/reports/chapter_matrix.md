# ACG720 教程 70 章工程/模块矩阵

完整模块名见 `chapter_matrix.csv` 的 `module_names` 列和 `all_unique_modules.csv`。`board_25k` 只表示片上合成激励/串口自检，不代表对应外接模块已接到 25K。

| 章 | 主题 | 工程 | 原始唯一模块 | 重新综合 | P&R（含兼容） | 时序 | 25K | 主要外设 |
|---:|---|---:|---:|---|---|---|---|---|
| 1 | 第1章 Gowin软件安装指导 | 0 | 0 | NOT_APPLICABLE | NOT_APPLICABLE | NOT_AVAILABLE | NOT_APPLICABLE | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 2 | 第2章 Gowin软件基本开发流程 | 1 | 1 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 3 | 第3章 Gowin联合Modelsim仿真实验 | 1 | 2 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 4 | 第4章 Gowin在线逻辑分析仪的使用 | 1 | 3 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 5 | 第5章 时序逻辑电路设计之计数器 | 1 | 2 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | PHYSICAL_BIST_PASS | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 6 | 第6章 串口发送模块设计与验证 | 1 | 3 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | PHYSICAL_BIST_PASS | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 7 | 第7章 串口接收模块设计与验证 | 1 | 5 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | PHYSICAL_BIST_PASS | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 8 | 第8章 串口控制LED灯 | 1 | 5 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 9 | 第9章 多字节串口收发模块设计与验证 | 1 | 8 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 10 | 第10章 状态机设计实例 | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 11 | 第11章 独立按键消抖设计与验证 | 1 | 3 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 12 | 第12章 模块化设计基础之加减法计数器 | 1 | 6 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NORMALIZED_SIM_PASS | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 13 | 第13章 8位7段数码管驱动设计与验证 | 1 | 5 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 14 | 第14章 IP核使用之ROM | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 15 | 第15章 IP核使用之RAM | 1 | 2 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 16 | 第16章 IP核使用之FIFO | 1 | 2 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 17 | 第17章 IP核使用之PLL | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 18 | 第18章 基于AD9767高速DAC的DDS信号发生器设计 | 2 | 13 | PASS_2_OF_2 | PASS_2_OF_2 | VIOLATED_2_OF_2 | NOT_PHYSICALLY_RUN | AD9767高速DAC模块 |
| 19 | 第19章 无源蜂鸣器驱动设计与验证 | 1 | 3 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | PHYSICAL_BIST_PASS | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 20 | 第20章 基于PWM波的音乐发生器 | 1 | 5 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 21 | 第21章 矩阵键盘驱动设计与验证 | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 4x4矩阵键盘 |
| 22 | 第22章 HT6221红外遥控解码设计与验证 | 1 | 3 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | HT6221红外接收 |
| 23 | 第23章 线性序列机与串行接口DAC驱动设计与验证 | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | TLV5618 DAC |
| 24 | 第24章 线性序列机与串行接口ADC驱动设计与验证 | 1 | 5 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | ADC128S102/兼容ADC |
| 25 | 第25章 八通道数字电压表设计与验证 | 1 | 11 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 八通道ADC采集板 |
| 26 | 第26章 VGA显示驱动设计与验证 | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | GM7123/VGA输出 |
| 27 | 第27章 多分辨率适配型VGA控制器设计 | 0 | 0 | NOT_APPLICABLE | NOT_APPLICABLE | NOT_AVAILABLE | NOT_APPLICABLE | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 28 | 第28章 TFT显示屏驱动设计与验证 | 1 | 5 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | TFT显示屏 |
| 29 | 第29章 基于FPGA的HDMI | 1 | 10 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | HDMI/DVI输出 |
| 30 | 第30章 基于HDMI和TFT显示屏的图片显示 | 1 | 11 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | TFT与HDMI显示 |
| 31 | 第31章 HDMI和TFT显示屏的静态字符显示 | 2 | 11 | PASS_2_OF_2 | PASS_2_OF_2 | CLEAN_2_OF_2 | NOT_PHYSICALLY_RUN | TFT与HDMI显示 |
| 32 | 第32章 I2C接口控制器设计与验证 | 1 | 6 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 33 | 第33章 基于I2C接口的串口读写EEPROM | 2 | 12 | PASS_2_OF_2 | PASS_2_OF_2 | CLEAN_2_OF_2 | NOT_PHYSICALLY_RUN | I2C EEPROM |
| 34 | 第34章 基于I2C接口的PCF8563数字时钟显示 | 3 | 27 | PASS_3_OF_3 | PASS_3_OF_3 | VIOLATED_1_OF_3 | NOT_PHYSICALLY_RUN | PCF8563 RTC与显示 |
| 35 | 第35章 基于ES8388的音频回环实验 | 3 | 17 | PASS_3_OF_3 | PASS_3_OF_3 | VIOLATED_2_OF_3 | NOT_PHYSICALLY_RUN | ES8388音频编解码器 |
| 36 | 第36章 DDR控制器的使用 | 1 | 124 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 板载DDR3 |
| 37 | 第37章 二端口DDR3控制器 | 1 | 9 | PASS_1_OF_1 | FAIL_1_OF_1 | NOT_AVAILABLE | NOT_PHYSICALLY_RUN | 板载DDR3 |
| 38 | 第38章 基于DDR3的串口传图帧缓存系统设计实现 | 1 | 28 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 板载DDR3、TFT、HDMI及UART图像源 |
| 39 | 第39章 彩色图像灰度化的设计实现(HDMI和TFT显示) | 1 | 31 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | PHYSICAL_BIST_PASS | 板载DDR3、TFT、HDMI及UART图像源 |
| 40 | 第40章 灰度图像中值滤波设计实现(HDMI和TFT显示) | 1 | 37 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 板载DDR3、TFT、HDMI及UART图像源 |
| 41 | 第41章 灰度图像均值滤波设计实现(HDMI和TFT显示) | 1 | 36 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 板载DDR3、TFT、HDMI及UART图像源 |
| 42 | 第42章 灰度图像高斯滤波设计实现(HDMI和TFT显示) | 1 | 36 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 板载DDR3、TFT、HDMI及UART图像源 |
| 43 | 第43章 Sobel算子边缘检测设计实现(HDMI和TFT) | 1 | 38 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 板载DDR3、TFT、HDMI及UART图像源 |
| 44 | 第44章 基于OV5640摄像头理论知识讲解 | 0 | 7 | NORMALIZED_2_OF_2_PASS | NOT_APPLICABLE | NOT_AVAILABLE | PHYSICAL_BIST_PASS | OV5640（散装核心可用合成输入验证） |
| 45 | 第45章 OV5640基于FPGA的编程实战 | 1 | 29 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | OV5640、DDR3、TFT |
| 46 | 第46章 OV5640摄像头采集VGA显示屏显示 | 1 | 26 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | OV5640、DDR3、VGA |
| 47 | 第47章 OV5640摄像头采集HDMI显示屏显示 | 1 | 29 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | OV5640、DDR3、HDMI |
| 48 | 第48章 以太网硬件 UDP 协议栈设计和开发 | 2 | 27 | PASS_2_OF_2 | PASS_2_OF_2_WITH_NORMALIZED_COMPAT | VIOLATED_1_OF_2 | NOT_PHYSICALLY_RUN | 千兆以太网PHY/RGMII |
| 49 | 第49章 以太网相关通信接口详解 | 0 | 0 | NOT_APPLICABLE | NOT_APPLICABLE | NOT_AVAILABLE | NOT_APPLICABLE | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 50 | 第50章 以太网（MAC）帧协议介绍 | 0 | 0 | NOT_APPLICABLE | NOT_APPLICABLE | NOT_AVAILABLE | NOT_APPLICABLE | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 51 | 第51章 RGMII与GMII转换电路设计 | 1 | 5 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 千兆以太网PHY/RGMII |
| 52 | 第52章 ARP协议的原理与FPGA实现 | 2 | 10 | PASS_2_OF_2 | PASS_2_OF_2 | CLEAN_2_OF_2 | NOT_PHYSICALLY_RUN | 千兆以太网PHY/RGMII |
| 53 | 第53章 IP协议介绍与IP校验和算法实现 | 1 | 2 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | PHYSICAL_BIST_PASS | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 54 | 第54章 UDP协议原理与FPGA实现 | 2 | 11 | PASS_2_OF_2 | PASS_2_OF_2_WITH_NORMALIZED_COMPAT | CLEAN_2_OF_2 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 55 | 第55章 实用性UDP发送逻辑设计与实现 | 2 | 10 | PASS_2_OF_2 | PASS_2_OF_2 | CLEAN_2_OF_2 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 56 | 第56章 实用性UDP接收逻辑设计与实现 | 1 | 9 | PASS_1_OF_1 | PASS_1_OF_1_WITH_NORMALIZED_COMPAT | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 板载时钟/复位/UART/LED或仅片上逻辑 |
| 57 | 第57章 MDIO控制器设计 | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | 千兆以太网PHY/RGMII |
| 58 | 第58章 千兆以太网UDP回环测试 | 1 | 13 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 千兆以太网PHY/RGMII |
| 59 | 第59章 基于OV5640的以太网RGMII图像传输系统设计 | 1 | 23 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | OV5640与千兆以太网PHY |
| 60 | 第60章 基于双目OV5640的以太网RGMII图像传输设计 | 1 | 22 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 双OV5640与千兆以太网PHY |
| 61 | 第61章 千兆以太网传图接收TFT显示系统设计 | 1 | 22 | PASS_1_OF_1 | PASS_1_OF_1_WITH_NORMALIZED_COMPAT | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | 千兆以太网PHY、DDR3、TFT |
| 62 | 第62章 千兆以太网传输ACM7606数据采集 | 1 | 43 | PASS_1_OF_1 | PASS_1_OF_1_WITH_NORMALIZED_COMPAT | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | ACM7606/AD7606、DDR3、千兆以太网PHY |
| 63 | 第63章 千兆以太网传输ACM1030数据采集 | 1 | 44 | PASS_1_OF_1 | PASS_1_OF_1_WITH_NORMALIZED_COMPAT | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | ACM1030、DDR3、千兆以太网PHY |
| 64 | 第64章 USB2.0数据传输系统应用环境简介 | 0 | 0 | NOT_APPLICABLE | NOT_APPLICABLE | NOT_AVAILABLE | NOT_APPLICABLE | Cypress FX2工具/驱动/固件与PC软件 |
| 65 | 第65章 USB2.0的FPGA控制SlaveFIFO 回送测试 | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | Cypress FX2 SlaveFIFO |
| 66 | 第66章 USB2.0接口Stream_out测试 | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | Cypress FX2 SlaveFIFO |
| 67 | 第67章 USB2.0基于Stream_OUT传输的USB传图实验 | 1 | 28 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | Cypress FX2、DDR3、TFT/HDMI |
| 68 | 第68章 USB2.0接口FPGA控制Stream_In测试 | 1 | 2 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | Cypress FX2 SlaveFIFO |
| 69 | 第69章 USB2.0基于Stream_IN的JPEG图像传输测试 | 1 | 4 | PASS_1_OF_1 | PASS_1_OF_1 | CLEAN_1_OF_1 | NOT_PHYSICALLY_RUN | Cypress FX2与JPEG数据源 |
| 70 | 第70章 USB2.0基于Stream_IN的OV5640 RGB摄像头实验 | 1 | 16 | PASS_1_OF_1 | PASS_1_OF_1 | VIOLATED_1_OF_1 | NOT_PHYSICALLY_RUN | Cypress FX2与OV5640 |

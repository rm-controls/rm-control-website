# 硬件规格
根据视觉识别要求, 我们提供了三套主控解决方案:

[![硬件方案](https://s1.ax1x.com/2020/10/24/BVWKXR.png)](https://imgchr.com/i/BVWKXR)

1. Jetson 系列: 深度学习视觉算法
   * 内置CAN
   * 内置GPIO

2. Intel NUC: 传统视觉算法
   * USB转CAN (Candlelight)
   * CAN转GPIO

3. UP Board: 无视觉算法
   * SPI转CAN (MCP2515)
   * 内置GPIO

> [!Note]
> 
> CAN总线最大延迟均在1ms内, 满足RM要求.
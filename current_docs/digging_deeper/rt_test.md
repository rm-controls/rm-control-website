---
id: rt_test
sidebar_position: 4
---

#测试内核实时性
##安装测试脚本
安装[测试脚本](https://github.com/qiayuanliao/Ubuntu-RT-UP-Board),
将此仓库拉到本地即可。
##安装脚本依赖
`sudo apt install rt-tests stress gnuplot`
##进行测试
###进入/test 文件夹并运行实时性测试脚本
`cd test`

`sudo sh ./rh-test.sh`
###等待测试完成,测试结果会生成图片放在测试脚本所在文件夹
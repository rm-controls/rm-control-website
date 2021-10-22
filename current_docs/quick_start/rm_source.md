---
id: rm_source
sidebar_position: 3
---

# 添加源
我们建立了私有软件源,以方便你能够以更便捷的方式获取并安装最新版本的rm-controls。
:::caution
私有软件源仅提供对Ubuntu 20.04（ros-noetic）的支持。
:::
## 添加公钥
首先，为了保证你获取的软件包的来源可信，你需要获取软件源的公钥并将其加入到你的apt中。
```sh
wget -O - https://rmsource.gdutelc.com/key/public.key | sudo apt-key add -
```
## 添加源
将软件源添加到apt的sources.list文件。
```sh
echo "deb https://rmsource.gdutelc.com/ focal main" | sudo tee -a /etc/apt/sources.list
```
更新软件源。
```sh
sudo apt update
```
完成以上操作后，即可在apt list中查找到可安装的最新版本的rm-controls。
```sh
sudo apt list ros-noetic-rm*
```
---
id: rm_source
sidebar_position: 3
---

# 添加源

我们建立了私有软件源,以方便你能够以更便捷的方式获取并安装最新构建的`rm-controls`和`rm-conrtollers`。
:::caution
私有软件源仅提供对 Ubuntu 20.04（ros-noetic）的支持。
:::

## 添加公钥

首先，为了保证你获取的软件包的来源可信，你需要获取软件源的公钥并将其加入到你的`apt`中。

```sh
wget -O - https://rmsource.gdutelc.com/ubuntu/key/public.key | sudo apt-key add -
```

## 添加源

将软件源添加到`apt`的 sources.list 文件。

```sh
echo "deb https://rmsource.gdutelc.com/ubuntu/ focal main" | sudo tee -a /etc/apt/sources.list
```

更新软件源。

```sh
sudo apt update
```

完成以上操作后，即可在 apt list 中查找到可安装的最新版本的`rm-controls`以及`rm-conrtollers`。

```sh
sudo apt search ros-noetic-rm
```

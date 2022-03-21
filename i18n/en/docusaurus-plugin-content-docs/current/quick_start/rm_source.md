---
id: rm_source
sidebar_position: 3
---

# Setting Sources

We have created a private repository,so that you can install the latest builds of `rm-controls` and `rm-conrtollers` in a more convenient way.
:::caution
The private repository only provides support for Ubuntu 20.04 (ros-noetic).
:::

## Setting public key

To ensure that the packages you get are from a trusted source, you need to get the public key and add it to your `apt`.

```sh
wget -O - https://rmsource.gdutelc.com/ubuntu/key/public.key | sudo apt-key add -
```

## Setting sources

Set the software source to `apt` sources.list file.

```sh
echo "deb https://rmsource.gdutelc.com/ubuntu/ focal main" | sudo tee -a /etc/apt/sources.list
```

Update software sources.

```sh
sudo apt update
```

Once you have done this, you can find the latest version of `rm-controls` and `rm-conrtollers` in the `apt` list.

```sh
sudo apt search ros-noetic-rm
```

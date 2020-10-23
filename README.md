# Docker for Stone Caring Robot

基于Docker的[多传感器融合定位/Sensor Fusion](https://www.shenlanxueyuan.com/my/course/261)学习环境.

---

## 安装配置Docker以及Docker-Compose

在开始使用前，首先需要在本地配置`Docker`以及`Docker-Compose`环境.

---

### 安装Docker

请参考[Docker官方文档](https://docs.docker.com/engine/install/ubuntu/)完成`Docker`环境的安装

安装完成后, 还需要进行`如下操作`, 以保证环境的易用性:

#### 将当前用户加入Docker Group

为了能在非`sudo`模式下使用`Docker`, 需要将当前用户加入`Docker Group`.

* 执行命令:
    
    ```bash
    sudo usermod -aG docker $USER
    ```

* **为了使上述变更生效，请先Logout，再Login**

---

### 安装Docker-Compose

`Docker-Compose`是基于Docker解决方案的Orchestrator. 

请参考[Docker Compose官方文档](https://docs.docker.com/compose/install/)完成`Docker-Compose`环境的安装

---

## 编译镜像

在Repo根目录下，执行如下操作，编译镜像. 镜像的内容有依赖，编译需要按顺序执行:

---

## 获取镜像

在安装完成`Docker`以及`Docker-Compose`之后，需要从`阿里云`源上获得所需镜像.

```bash
```

---

## Use Case Overview

Below are the tutorials for possible use cases:

* [Function Development](https://gitee.com/shanghai_toinch_intelligence/stone_caring_robot_docker#use-case-03----for-function-development)

---

## Use Case 03 -- For Function Development

[Back to Overview](https://gitee.com/shanghai_toinch_intelligence/stone_caring_robot_docker#use-case-overview)

---
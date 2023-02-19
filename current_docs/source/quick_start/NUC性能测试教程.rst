NUC性能测试教程
^^^^

测试环境搭建
----

1. **从GitHub上下载压缩包，进行源码下载**

下载地址：https://github.com/akopytov/sysbench/releases

.. image:: https://typoranote-lht.oss-cn-guangzhou.aliyuncs.com/image/image-20210621194156865.png 

2. **安装环境搭建** :: 

    sudo apt -y install make automake libtool pkg-config libaio-dev

3. **安装** :: 

    cd sysbench-1.0.20 		#进入安装文件夹
    ./autogen.sh
    ./configure
    make 
    make install
    sysbench --version  #检测是否安装成功，以及版本是否准确

如果显示 `sysbench 1.0.20`，即表示安装成功

.. [Ref] 如果看教程看不懂或者遇到问题的，可以在安装文件夹中找到 README.md 文件查看说明文档，那里有官方的安装教程

开始测试
----

需要测试的内容有五项：CPU、memory内存分配、thread线程调度、文件io测试、互斥锁测试

并需要打印两项内容：测试设备的硬件信息、测试系统信息

1. **cpu性能测试** :: 

    sysbench --num-threads=12 --max-requests=10000 --debug=on --test=cpu --cpu-max-prime=20000 run


结果分析：

.. image:: https://typoranote-lht.oss-cn-guangzhou.aliyuncs.com/image/image-20210621200124644.png 

.. [Ref] 往后的测试中，都需要在测试结果中查看对应上图中的信息

2. **memory内存分配测试** :: 

    # 8k顺序分配
    sysbench --num-threads=12 --max-requests=10000 --test=memory --memory-block-size=8K --memory-total-size=100G --memory-access-mode=seq run
    # 8k随机分配
    sysbench --num-threads=12 --max-requests=10000 --test=memory --memory-block-size=8K --memory-total-size=100G --memory-access-mode=rnd run
    # 16k顺序分配
    sysbench --num-threads=12 --max-requests=10000 --test=memory --memory-block-size=16K --memory-total-size=100G --memory-access-mode=seq run
    # 16k随机分配
    sysbench --num-threads=12 --max-requests=10000 --test=memory --memory-block-size=16K --memory-total-size=100G --memory-access-mode=rnd run


3. **thread线程调度测试** :: 

    sysbench --num-threads=12 --max-requests=10000  --test=threads --thread-yields=100 --thread-locks=2 run

4. **文件 io 测试** :: 

    sysbench --num-threads=12 --max-requests=10000  --test=fileio --file-total-size=3G --file-test-mode=rndrw prepare

5. **互斥锁测试** :: 

    sysbench  --num-threads=12  --test=mutex --mutex-num=1024 --mutex-locks=10000 --mutex-loops=10000 run

6. **打印设备硬件信息、测试系统信息** :: 

    lshw -short
    lsb_release -a

单个设备的测试流程至此结束


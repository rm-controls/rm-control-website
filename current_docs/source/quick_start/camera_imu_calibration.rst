相机imu联合标定
==================================

相机和imu的联合标定主要分为三个步骤：

1、相机标定

2、imu标定

3、相机imu联合标定

其中前两个步骤可同时进行。

主要参考 `这篇文章 <https://immortalqx.github.io/2021/07/04/kalibr-for-camera-imu-calibration/>`_，写得较为详细。

还有
`这个视频 <https://www.youtube.com/watch?v=puNXsnrYWTY&t=192s>`_，这个视频是kalibr官方出的，较为可靠。

要点：

1、标定相机时，推荐的速率是每秒4帧左右。运动方式为：先绕相机的各自三个轴，每个轴旋转三次，然后再沿着相机的三个轴，每个轴平移三次，基本就可以了，运动期间要保证相机基本能一直看到标定板的全部信息。因为这是标定Camera，所以也可以将打印出来的标定板拿好，然后在相机前面来回运动。rosbag录制时间为一分钟左右。

2、标定imu时，录制rosbag需让imu静止不动录制比较长的时间，imu_utils的github主页要求录制两小时以上。

3、相机imu联合标定时，我们将标定板固定在某处，让相机对着标定板运动，运动的方式是和之前一样的“三转、三移”。这里需要注意的是，一定要让相机对着标定板运动，因为我们需要标定imu，如果相机不动，imu的加速度就一直为0，无法进行标定。rosbag录制时间为一分钟左右。

4、对于相机-IMU标定，`官方文档 <https://github.com/ethz-asl/kalibr/wiki/Multi-IMU-and-IMU-intrinsic-calibration>`_ 给出的建议帧率是：Camera 20FPS，IMU 200FPS。


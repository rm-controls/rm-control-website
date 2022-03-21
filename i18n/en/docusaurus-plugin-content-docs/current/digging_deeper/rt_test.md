---
id: rt_test
sidebar_position: 4
---

## Test kernel real-time

## Install the test script

The test scripts are in rm_bringup, click [here](https://github.com/rm-controls/rm_bringup),
Just pull this repository locally.

## Install script dependencies

`sudo apt install rt-tests stress gnuplot`

## Run the test

### Go to the script folder and run the live test script

```shell
cd rm_bringup/scripts
sudo sh . /rh-test.sh
```

# Wait for the test to complete.

### Wait for the test to finish, the test result will generate an image and put it in the test script folder

The following image shows the comparison before and after installing the live patch, you can clearly see that the live performance has been greatly improved after installing the patch.

![before patch](/img/digging_deeper/rt_test0.png)

![After patching](/img/digging_deeper/rt_test1.png)

Analysis.

1. more samples on the left side usually means lower latency
2. more clustered samples means less chatter
3. maximum latency should not deviate too far from the average (usually below 100us)

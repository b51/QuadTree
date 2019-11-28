# QuadTree

## What's this repository for
生成一百个随机点并放入四叉树, 生成一个随机点(rand_point)搜索该点的最邻近点并打印出来，同时以 --r 为范围搜索该范围内有多少个点，并打印出来所有该范围内点的坐标

## Build and Run

```bash
$ mkdir build
$ cd build
$ cmake .. && make -j4
$ ./QuadTree --r 10 (--r 表示以该点为中心的搜索范围)
```

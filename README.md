# Welcome to the OS_Project17_Segfault wiki!

_This page is created for our robot project of EURECOM - Operating Systems 2017 class_

![](https://i.imgur.com/X3zBYDX.jpg)

Robot Name: **SegFault**

Robot Bluetooth Address: A0:E6:F8:FC:C2:59 

The details of the project can be found under : 

[Algorithms](https://github.com/Horbaje/OS_Project17_Segfault/wiki/Algorithms) 

[Robot Architecture](https://github.com/Horbaje/OS_Project17_Segfault/wiki/Architecture-of-the-robot) 

[Description](https://github.com/Horbaje/OS_Project17_Segfault/wiki/Project-Description) 

[Gallery](https://github.com/Horbaje/OS_Project17_Segfault/wiki/Gallery) 

## How to compile

To Compile:
```
  gcc -I./ev3dev-c/source/ev3 -O2 -std=gnu99 -W -Wall -Wno-comment  -c tester.c -o tester.o
  gcc tester.o -Wall -lm -lev3dev-c -lbluetooth -o tester
```

To Run:
```
  ./tester
```

## Group Members:


![](https://i.imgur.com/L5Nsr78.jpg)


Jermaine Easton

Berkay Köksal

Ariane Horbach

Yasmine Dissem

# xtensor ROS

This repository contains bindings for xtensor to the ROS Robotics Operating System.
It offers helper functions to easily send and receive xtensor and xarray datastructures
as ROS messages. This allows easy embedding of multidimensional arrays in ROS C++
programs. We are planning to develop an accompanying Python library which would parse
the messages into NumPy datastructures for full interoperability.

### About [xtensor](https://github.com/QuantStack/xtensor)

Multi-dimensional arrays with broadcasting and lazy computing.

xtensor is a C++ library meant for numerical analysis with multi-dimensional array expressions.

xtensor provides

- an extensible expression system enabling lazy broadcasting.
- an API following the idioms of the C++ standard library.
- tools to manipulate array expressions and build upon xtensor.

More documentation: [https://xtensor.readthedocs.io/en/latest/?badge=latest]
Numpy to xtensor cheatsheet: [https://xtensor.readthedocs.io/en/latest/numpy.html]

### Example

A publisher:

```cpp
#include "ros/ros.h"
#include <iostream>
#include <xtensor/xrandom.hpp>
#include <xtensor/xio.hpp>

#include "conversions.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<xt::xarray<double>>("chatter", 1000);

    ros::Rate loop_rate(10);

    int count = 0;

    // fill an xarray with ones
    xt::xarray<double> a = xt::ones<unsigned char>({3, 3});

    while (ros::ok())
    {
        chatter_pub.publish(a);
        ros::spinOnce();
        loop_rate.sleep();

        // add 1 to all elements in xarray
        a += 1;
    }
    return 0;
}
```

and a subscriber to multidimensional arrays:

```cpp
#include "ros/ros.h"

#include <xtensor/xio.hpp>

#include "conversions.hpp"

void chatterCallback(const xt::xarray<double>& msg_arr)
{
    ROS_INFO("I heard: something");
    std::cout << msg_arr << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::spin();

    return 0;
}
```

If you have messages which include more than one field, conversion is unfortunately slightly trickier, due
to constraints of the message generation of ROS. Internally, xtensor_ros has a message for each type: f64, f32, 
u8, u16, u32, u64, i8, i16, i32, i64. These can be used in your message definition, for example:

```
uint8 id
xtensor_ros/f64 arr
```

This will add an xtensor message field to your custom message. Automatic conversion is unfortunately not yet possible,
but we have the following syntax:

```cpp
// consider a message with an ID field, and a xtensor_ros/f64 array field
msg.id = 123;
msg.array = as_msg(a);
// and then, to go from the internal xtensor_ros/f64 message to an xarray of the 
// correct type (on the subscriber side)
auto arr = from_msg(msg.arr);
```
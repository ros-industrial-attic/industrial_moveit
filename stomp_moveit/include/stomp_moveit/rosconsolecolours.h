#ifndef ROSCONSOLECOLOURS_H
#define ROSCONSOLECOLOURS_H

#include <vector>
#include <deque>
#include <ros/console.h>
#include <iostream>


#define ROS_RED_STREAM(SSS) ROS_INFO_STREAM("\033[31;1m" << SSS << "\033[0m\n")
#define ROS_GREEN_STREAM(SSS) ROS_INFO_STREAM("\033[32;1m" << SSS << "\033[0m\n")
#define ROS_YELLOW_STREAM(SSS) ROS_INFO_STREAM("\033[33;1m" << SSS << "\033[0m\n")
#define ROS_BLUE_STREAM(SSS) ROS_INFO_STREAM("\033[34;1m" << SSS << "\033[0m\n")
#define ROS_MAGENTA_STREAM(SSS) ROS_INFO_STREAM("\033[35;1m" << SSS << "\033[0m\n")
#define ROS_CYAN_STREAM(SSS) ROS_INFO_STREAM("\033[36;1m" << SSS << "\033[0m\n")
#define ROS_WHITE_STREAM(SSS) ROS_INFO_STREAM("\033[30;47m" << SSS << "\033[0m\n")

template <typename T>
std::ostream& operator<< (std::ostream& stream, const std::vector<T>& list)
{
  for(const T& elem : list)
    stream << elem << ", ";
  stream.seekp(-2, std::ios_base::end); // remove last ", "
  return stream;
}

template <typename T>
std::ostream& operator<< (std::ostream& stream, const std::deque<T>& list)
{
  for(const T& elem : list)
    stream << elem << ", ";
  stream.seekp(-2, std::ios_base::end); // remove last ", "
  return stream;
}

/* free test code here
ROS_RED_STREAM("bold red text");
ROS_YELLOW_STREAM("yellow stuff");
ROS_GREEN_STREAM("Green stuff");
ROS_BLUE_STREAM("blue");
ROS_CYAN_STREAM("cyan is different fromb blue");
ROS_MAGENTA_STREAM("magenta is not pink or is it?");
ROS_WHITE_STREAM("white bricks");
*/

#endif
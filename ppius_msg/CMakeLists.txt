cmake_minimum_required(VERSION 2.8.3)
project(ppius_msg)



#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)



add_message_files(
  FILES
  ppius.msg
  compress.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  
  CATKIN_DEPENDS message_runtime 
) 

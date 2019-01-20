#ifndef COMPRESS_NODE_
#define COMPRESS_NODE_

#include <math.h>
#include <stdlib.h>
#include <string> 
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <cstdint>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include "filter_library.h"
#include <ppius_msg/ppius.h> 
#include <ppius_msg/compress.h> 

class CompressNode
{
public:
    CompressNode(ros::NodeHandle* nodehandle);
private:
    ros::NodeHandle nh_;
    ros::Publisher compress_pub_;
    ros::Subscriber sound_sub_;
    
    void initializeSubscriber();
    void initializePublisher();
    
    void soundSubscriberCallback(const ppius_msg::ppius& sound_message);
    

    void p_conv_f32(const double *x, const double *h, double *r, int nx, int nh);
    struct filter filter_definition(char *c);
    struct koeficjenti dwt(double *signal,struct filter hp,unsigned short int nx);
    //char *hp_name_ = (char *)"hp_coefficients.txt";
    char *hp_name_ = (char *)"/home/branko/catkin_ws/src/ppius/src/hp_coefficients.txt";
    struct filter hp_;
    struct signali sig;
    struct koeficjenti coef, tmp;
    
    
    
};

#endif

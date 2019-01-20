#ifndef DECOMPRESS_NODE_
#define DECOMPRESS_NODE_

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
#include <ppius_msg/compress.h> 
#include <ppius_msg/sound.h> 


class DecompressNode
{
public:
    DecompressNode(ros::NodeHandle* nodehandle);
private:
    ros::NodeHandle nh_;

    ros::Subscriber compress_sub_;
    ros::Publisher sound_pub_;
    void initializeSubscriber();
    void initializePublisher();
    
    void compressSubscriberCallback(const ppius_msg::compress& compressed_sound);
    

    void p_conv_f32(const double *x, const double *h, double *r, int nx, int nh);
    struct filter filter_definition(char *c);
    struct signali idwt(struct koeficjenti c, struct filter fp);
    char *fp_name_ = (char *)"/home/branko/catkin_ws/src/ppius/src//fp_coefficients.txt";
    struct filter fp_;
    struct signali sig;
	struct koeficjenti coef, tmp;
    
    
    
};

#endif

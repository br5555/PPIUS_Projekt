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

/// <summary>
/// CompressNode accpets sound message and compress it
/// </summary>
class CompressNode
{
public:
	/// <summary>
	/// Public constructor creates instance of class CompressNode
	/// </summary>
	/// <param name="nodehandle">ROS communication handler</param>
	/// <returns></returns>
    CompressNode(ros::NodeHandle* nodehandle);
private:
	//ros node handle for communication with ros master
    ros::NodeHandle nh_;
	// ros publisher for publishing compressed message
    ros::Publisher compress_pub_;
	//ros subscriber for accepting sound message
    ros::Subscriber sound_sub_;

	/// <summary>
	/// Create instance of ros::Subscriber and subscribe to ros topic 
	/// </summary>
	/// <returns></returns>
    void initializeSubscriber();

	/// <summary>
	/// Create instance of ros::Publisher 
	/// </summary>
	/// <returns></returns>
    void initializePublisher();

	/// <summary>
	/// ros::Subscriber callback function executes when a new message come to subscribed ros topic
	/// and send compressed sound to desire topic
	/// </summary>
	/// <param name="sound_message">new ros message</param>
	/// <returns></returns>
    void soundSubscriberCallback(const ppius_msg::ppius& sound_message);

	/**
	 * Computes the convolution of two vectors 'x' and 'h', and places the
	 * results in vector 'r'.
	 *
	 * @param x       Pointer to input vector of size 'nr' elements
	 *
	 * @param h       Pointer to 'nh' filter coefficients
	 *
	 * @param r       Output vector of size 'nr+nh-1'
	 *
	 * @param nx      The number of input samples
	 *
	 * @param nh      The number of coefficients of the filter
	 *
	 * @return        None
	 *
	 */
    void p_conv_f32(const double *x, const double *h, double *r, int nx, int nh);

	/**
	 * Creates structure of filter from given file name
	 *  structure is pholiphase representation of decomposition or reconstruction filter
	 *
	 * @param c     name of txt file containing definition of filter
	 *              contains 4 columns and r rows, wich are defined in first line of file.
	 *
	 * @return      structure of filter
	 */
    struct filter filter_definition(char *c);

	/**
	 *          Racunanje dwt -a
	 * @param signal    ulazni signal za dekompoziciju
	 * @param hp        struktura filtera hp
	 * @param nx        duljina signala za dekompoziciju
	 * @return          wraca strukturu koeficjenata
	 */
    struct koeficjenti dwt(double *signal,struct filter hp,unsigned short int nx);
    
    char *hp_name_ = (char *)"/home/branko/catkin_ws/src/ppius/src/hp_coefficients.txt";
	

    struct filter hp_;
    struct signali sig;
    struct koeficjenti coef, tmp;
    
    
    
};

#endif

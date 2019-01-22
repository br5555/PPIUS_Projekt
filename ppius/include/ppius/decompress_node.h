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


/// <summary>
/// DecompressNode accpets compressed sound message and decompress it
/// </summary>
class DecompressNode
{
public:
	/// <summary>
	/// Public constructor creates instance of class DecompressNode
	/// </summary>
	/// <param name="nodehandle">ROS communication handler</param>
	/// <returns></returns>
    DecompressNode(ros::NodeHandle* nodehandle);
private:
	//ros node handle for communication with ros master
    ros::NodeHandle nh_;
	// ros subscriber for accpeting compressed message
    ros::Subscriber compress_sub_;
	// ros publisher for publishing decompressed message
    ros::Publisher sound_pub_;

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
	/// and send decompressed sound to desire topic
	/// </summary>
	/// <param name="compressed_sound">new ros message</param>
	/// <returns></returns>
    void compressSubscriberCallback(const ppius_msg::compress& compressed_sound);
    
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
	 * IDWT                 computers 1 level idwt
	 * @param c             coeffs
	 * @param fp            structure of filter
	 * @param n             length of signal
	 * @return              pointer to reconstructed signal
	 */
    struct signali idwt(struct koeficjenti c, struct filter fp);

    char *fp_name_ = (char *)"/home/branko/catkin_ws/src/ppius/src//fp_coefficients.txt";
    struct filter fp_;
    struct signali sig;
	struct koeficjenti coef, tmp;
    
    
    
};

#endif

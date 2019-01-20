#include <ppius/compress_node.h>

CompressNode::CompressNode(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 
    initializeSubscriber(); 
    initializePublisher();
    hp_=filter_definition(hp_name_);
       
}

void CompressNode::initializeSubscriber()
{
    
    sound_sub_ = nh_.subscribe("sound_topic", 1024, &CompressNode::soundSubscriberCallback,this);  
    
}


void CompressNode::initializePublisher()
{

    compress_pub_ = nh_.advertise<ppius_msg::compress>("compress_sound_topic", 1, true); 
    
}


void CompressNode::soundSubscriberCallback(const ppius_msg::ppius& sound_message){
    

	sig.s_size= sound_message.dbl_vec.size() ;
	sig.s = new double[sound_message.dbl_vec.size()];
	for(int i = 0; i< sound_message.dbl_vec.size(); i ++){
        sig.s[i]=sound_message.dbl_vec[i];
    }
	
    tmp=dwt(sig.s,hp_,sig.s_size);
    coef=dwt(tmp.A,hp_,tmp.c_size);
    
    ppius_msg::compress compress_msg;
    
    compress_msg.A.resize(sizeof( coef.A) / sizeof( *coef.A));
    compress_msg.D.resize(sizeof( coef.D) / sizeof( *coef.D));
    compress_msg.c_size = coef.c_size;
    for(int i = 0; i< compress_msg.A.size(); i ++){
        compress_msg.A[i] = coef.A[i];
    }
    for(int i = 0; i< compress_msg.D.size(); i ++){
        compress_msg.D[i] = coef.D[i];
    }
    
    compress_pub_.publish(compress_msg);


}

void CompressNode::p_conv_f32(const double *x, const double *h, double *r, int nx, int nh)
{
    double *rx = r;
    int i,j,k;
    for ( i = 0; i < nx+nh-1; i++) { *(rx++) = 0; }
    rx = r;
    for ( i = 0; i < nh; i++) {
        for (j = 0; j < nx; j++) {
            k=i+j;
            *(rx + k) += *(h+i) * *(x+j);
        }
    }
}




/**
 * Creates structure of filter from given file name
 *  structure is pholiphase representation of decomposition or reconstruction filter
 *
 * @param c     name of txt file containing definition of filter
 *              contains 4 columns and r rows, wich are defined in first line of file.
 *
 * @return      structure of filter
 */


struct filter CompressNode::filter_definition(char *c) {

    FILE *fp;
    struct filter ff;
    int n;



    //printf("%s\n",c);
    if ((fp = fopen (c, "r")) == NULL)
    {
        //printf ("Error! opening file");

        // Program exits if the file pointer returns NULL.
        exit (1);
    }
    //fscanf (fp, "%ud\n", &ff.f_size);
    fscanf (fp, "%hu\n", &ff.f_size);
    //printf ("%d\n", ff.f_size);

    ff.f00 =(double *) malloc (ff.f_size * sizeof (double));
    ff.f10 =(double *) malloc (ff.f_size * sizeof (double));
    ff.f01 =(double *) malloc (ff.f_size * sizeof (double));
    ff.f11 =(double *) malloc (ff.f_size * sizeof (double));
    for (n = 0; n < ff.f_size; n++)
    {
        fscanf (fp, "%lf;%lf;%lf;%lf;\n", ff.f00+n, ff.f01+n, ff.f10+n, ff.f11+n);
        //printf ("%lf %lf %lf %lf\n", *(ff.f00+n),*(ff.f01+n),*(ff.f10+n),*(ff.f11+n));
    }
    fclose(fp);

    return(ff);
}


struct koeficjenti CompressNode::dwt(double *signal,struct filter hp,unsigned short int nx)
{
    int n,l;
    int j;
    double *signal_even;
    double *signal_uneven;
    double *a1,*d1,*d2;
    double *a2;
    struct koeficjenti Coef;


    j=(int) nx/2;
    l=hp.f_size + nx -1;
    signal_even = (double *) calloc(j+nx%2, sizeof(double));
    signal_uneven= (double *) calloc(j+nx%2, sizeof(double));
    a1=(double *) calloc(l, sizeof(double));
    a2=(double *) calloc(l, sizeof(double));
    d1=(double *) calloc(l, sizeof(double));
    d2=(double *) calloc(l, sizeof(double));



    for (n = 0; n < j; n++) {
        signal_even[n] = signal[2 * n];
        signal_uneven[n] = signal[2 * n + 1];
    }

    //printf("\t Paran\t Neparan\n");
    for (n = 0; n < j; n++) {
        //printf("\t %lf\t %lf\n", signal_even[n], signal_uneven[n]);
    }


    p_conv_f32(signal_even, hp.f00, a1, j, hp.f_size);
    p_conv_f32(signal_uneven, hp.f01, a2, j, hp.f_size);

    for (n = 0; n < l ; n++) {
        a1[n] = a1[n] + a2[n];
    }
    //printf("\naproksimacija 1 razine\n");
    for (n = 0; n < l; n++) {
        //printf("%lf \n", a1[n]);
    }

    Coef.c_size=j+hp.f_size-1;
    Coef.A=a1;
    //printf("%d\n",Coef.c_size);
    p_conv_f32(signal_even, hp.f10, d1, j, hp.f_size);
    p_conv_f32(signal_uneven, hp.f11, d2, j, hp.f_size);

    for (n = 0; n < l ; n++) {
        d1[n] = d1[n] + d2[n];
    }
    //printf("\ndetalji 1 razine\n");
    for (n = 0; n < l; n++) {
        //printf("%lf \n", d1[n]);
    }

    Coef.D=d1;

    free(a2);
    free(d2);
    free(signal_uneven);
    free(signal_even);

    return Coef;
}




int main(int argc, char** argv) 
{

    ros::init(argc, argv, "CompressNode"); 

    ros::NodeHandle nh; 


    CompressNode compressNode(&nh); 
    while (ros::ok()) {
        
        ros::spinOnce();
        
    }
    return 0;
} 

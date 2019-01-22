#include <ppius/decompress_node.h>

DecompressNode::DecompressNode(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ 
    initializeSubscriber();
    initializePublisher(); 
    fp_=filter_definition(fp_name_);
       
}

void DecompressNode::initializeSubscriber()
{
    
    compress_sub_ = nh_.subscribe("compress_sound_topic", 1024, &DecompressNode::compressSubscriberCallback,this);  
    
}
void DecompressNode::initializePublisher()
{

    sound_pub_ = nh_.advertise<ppius_msg::sound>("sound", 1024, true); 
    
}
void DecompressNode::compressSubscriberCallback(const ppius_msg::compress& compressed_sound){

    //struct koeficjenti t;
    
    coef.c_size = compressed_sound.c_size;
    coef.A = new double[compressed_sound.A.size()];
    coef.D = new double[compressed_sound.D.size()];
    //std::cout << "A je izlaz " << std::endl;
    for(int i = 0; i< compressed_sound.A.size(); i ++){
        coef.A[i] = compressed_sound.A[i];
        //std::cout << coef.A[i] << " ";
    }
    
    //std::cout << std::endl;
    //std::cout << "D je izlaz " << std::endl;
    for(int i = 0; i< compressed_sound.D.size(); i ++){
        coef.D[i] = compressed_sound.D[i];
        //std::cout << coef.D[i] << " ";
    }
    //std::cout << std::endl;
    /*
    t.c_size = compressed_sound.c_size_tmp;
    t.A = new double[compressed_sound.A_tmp.size()];
    t.D = new double[compressed_sound.D_tmp.size()];
    for(int i = 0; i< compressed_sound.A_tmp.size(); i ++){
        t.A[i] = compressed_sound.A_tmp[i];
    }
    for(int i = 0; i< compressed_sound.D.size(); i ++){
        t.D[i] = compressed_sound.D_tmp[i];
    }
    */
    
    
    //sig=idwt(coef,fp_);
    //t.c_size=sig.s_size;
    //t.A=sig.s;
    
    ////std::cout << "Proso jedan" << std::endl;
    sig=idwt(coef,fp_);
    ////std::cout << "Proso dva" << std::endl;
    //ROS_INFO_STREAM("Proso /n");
    
    ppius_msg::sound sound_msg;
    
    sound_msg.sound_size = sig.s_size;
    sound_msg.sound.resize(sig.s_size);
    //std::cout << "Sound end" << std::endl;
    for(int i = 0; i< sig.s_size; i++){
        sound_msg.sound[i] = sig.s[i];
        //std::cout << sig.s[i] << "  ";
    }
    //std::cout  << std::endl;
    sound_pub_.publish(sound_msg);

}


struct signali DecompressNode::idwt(struct koeficjenti c, struct filter fp) {
    struct signali s;
    double * r;
    int i; int n=c.c_size;
    double *a_even=(double *) malloc ((n+fp.f_size-1)*sizeof (double));
	double *a_uneven=(double *) malloc ((n+fp.f_size-1)* sizeof (double));

    double *d_even=(double *) malloc ((n+fp.f_size-1)*sizeof (double));
    double *d_uneven=(double *) malloc ((n+fp.f_size-1)* sizeof (double));

	////printf("krenula je kompozivcja natrag %lf\n", fp.f00[0]);

    p_conv_f32(c.A, fp.f00, a_even, n, fp.f_size);
    p_conv_f32(c.A, fp.f01, a_uneven, n, fp.f_size);
    p_conv_f32(c.D, fp.f10, d_even, n, fp.f_size);
    p_conv_f32(c.D, fp.f11, d_uneven, n, fp.f_size);


    ////printf("kek\n\n");

    s.s_size=2*(n-fp.f_size+1); //nov velicina signala
    r=(double *) malloc (s.s_size * sizeof (double));
    //printf("%d\n\n",s.s_size);
    for (i=0;i<s.s_size/2;i++){
        *(r+(2*i))=a_uneven[fp.f_size-1+i] + d_uneven[fp.f_size-1+i];
        //printf("%lf\t %lf\t",a_uneven[fp.f_size-1+i]);
        *(r+(2*i+1))=a_even[fp.f_size-1+i]+d_even[fp.f_size-1+i];
        //printf("%lf\t %lf\t",a_even[fp.f_size-1+i]);
       // printf("%lf\t %lf\n",r[2*i],r[2*i+1]);
    }

    free(a_even); free(a_uneven); free(d_uneven); free(d_even);
    s.s=r;
    return s;
}

void DecompressNode::p_conv_f32(const double *x, const double *h, double *r, int nx, int nh)
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







struct filter DecompressNode::filter_definition(char *c) {

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
        printf ("%lf %lf %lf %lf\n", *(ff.f00+n),*(ff.f01+n),*(ff.f10+n),*(ff.f11+n));
    }
    fclose(fp);

    return(ff);
}


int main(int argc, char** argv) 
{

    ros::init(argc, argv, "DecompressNode"); 

    ros::NodeHandle nh; 


    DecompressNode decompressNode(&nh); 


    ros::spin();
    return 0;
} 

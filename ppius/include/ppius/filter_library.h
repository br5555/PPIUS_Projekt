#ifndef UNTITLED_LIBRARY_H
#define UNTITLED_LIBRARY_H

struct filter
{
    char *name_f;
    unsigned short int f_size;
    double *f00, *f01, *f10, *f11;
};

struct koeficjenti{
    unsigned short int c_size;
    double *A, *D;
};
struct signali{
    unsigned short int s_size;
    double *s;
};

struct filter filter_definition(char *c);
void p_conv_f32(const double *x, const double *h, double *r, int nx, int nh);
struct signali idwt(struct koeficjenti c, struct filter fp);
struct koeficjenti dwt(double *signal,struct filter hp,unsigned short int nx);

#endif

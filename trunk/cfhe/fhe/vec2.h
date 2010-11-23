#ifndef FHE_VEC2_H
#define FHE_VEC2_H

typedef struct
{
    double x, y;
} fhe_vec2_t;

void fhe_vec2_init( fhe_vec2_t* v, double x, double y );
void fhe_vec2_init_rot( fhe_vec2_t* v, double th, double l );
void fhe_vec2_copy( fhe_vec2_t* to, const fhe_vec2_t* from );

void fhe_vec2_neg( fhe_vec2_t* r, fhe_vec2_t* v );
void fhe_vec2_add( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u );
void fhe_vec2_sub( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u );
void fhe_vec2_mul_double( fhe_vec2_t* r, fhe_vec2_t* v, double d );
void fhe_vec2_div_double( fhe_vec2_t* r, fhe_vec2_t* v, double d );
void fhe_vec2_mul_vec2( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u );
void fhe_vec2_div_vec2( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u );
void fhe_vec2_rotate( fhe_vec2_t* r, fhe_vec2_t* v, double th );
void fhe_vec2_norm( fhe_vec2_t* r, fhe_vec2_t* v );
void fhe_vec2_lerp( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u, double i );

void fhe_vec2_ineg( fhe_vec2_t* v );
void fhe_vec2_iadd( fhe_vec2_t* v, fhe_vec2_t* u );
void fhe_vec2_isub( fhe_vec2_t* v, fhe_vec2_t* u );
void fhe_vec2_imul_double( fhe_vec2_t* v, double d );
void fhe_vec2_idiv_double( fhe_vec2_t* v, double d );
void fhe_vec2_imul_vec2( fhe_vec2_t* v, fhe_vec2_t* u );
void fhe_vec2_idiv_vec2( fhe_vec2_t* v, fhe_vec2_t* u );
void fhe_vec2_irotate( fhe_vec2_t* v, double th );
void fhe_vec2_inorm( fhe_vec2_t* v );
void fhe_vec2_ilerp( fhe_vec2_t* v, fhe_vec2_t* u, double i );

double fhe_vec2_angle( fhe_vec2_t* v );
double fhe_vec2_length( fhe_vec2_t* v );
double fhe_vec2_dot( fhe_vec2_t* v, fhe_vec2_t* u );

#endif

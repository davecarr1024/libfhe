#ifndef FHE_MAT3_H
#define FHE_MAT3_H

#include <fhe/vec2.h>

typedef double fhe_mat3_t[9];

void fhe_mat3_init( fhe_mat3_t m, 
                    double d0, double d1, double d2,
                    double d3, double d4, double d5,
                    double d6, double d7, double d8 );
void fhe_mat3_init_translation( fhe_mat3_t m, fhe_vec2_t* t );
void fhe_mat3_init_scale( fhe_mat3_t m, fhe_vec2_t* s );
void fhe_mat3_init_rotation( fhe_mat3_t m, double th );
void fhe_mat3_copy( fhe_mat3_t to, const fhe_mat3_t from );
double fhe_mat3_get( fhe_mat3_t m, int i, int j );
void fhe_mat3_set( fhe_mat3_t m, int i, int j, double d );
void fhe_mat3_mul_mat3( fhe_mat3_t r, fhe_mat3_t m1, fhe_mat3_t m2 );
void fhe_mat3_mul_vec2( fhe_vec2_t* r, fhe_mat3_t m, fhe_vec2_t* v );
void fhe_mat3_imul_mat3( fhe_mat3_t m, fhe_mat3_t n );
void fhe_mat3_imul_vec2( fhe_vec2_t* v, fhe_mat3_t m );
double fhe_mat3_det( fhe_mat3_t m );
void fhe_mat3_inverse( fhe_mat3_t i, fhe_mat3_t m );

const fhe_mat3_t FHE_MAT3_ZERO = { 0,0,0, 0,0,0, 0,0,0 };
const fhe_mat3_t FHE_MAT3_I = { 1,0,0, 0,1,0, 0,0,1 };

#endif

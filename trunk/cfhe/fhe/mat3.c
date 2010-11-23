#include <fhe/mat3.h>
#include <fhe/util.h>
#include <math.h>

void fhe_mat3_init( fhe_mat3_t m, 
                    double d0, double d1, double d2,
                    double d3, double d4, double d5,
                    double d6, double d7, double d8 )
{
    m[0] = d0;
    m[1] = d1;
    m[2] = d2;
    m[3] = d3;
    m[4] = d4;
    m[5] = d5;
    m[6] = d6;
    m[7] = d7;
    m[8] = d8;
}

void fhe_mat3_init_translation( fhe_mat3_t m, fhe_vec2_t* t )
{
    FHE_ASSERT( t );
    fhe_mat3_init( m,
                   1, 0, t->x,
                   0, 1, t->y,
                   0, 0, 1 );
}

void fhe_mat3_init_scale( fhe_mat3_t m, fhe_vec2_t* s )
{
    FHE_ASSERT( s );
    fhe_mat3_init( m,
                   s->x, 0, 0,
                   0, s->y, 0,
                   0, 0, 1 );
}

void fhe_mat3_init_rotation( fhe_mat3_t m, double th )
{
    double sa = sin( -th );
    double ca = cos( -th );
    fhe_mat3_init( m,
                   ca, sa, 0,
                   -sa, ca, 0,
                   0, 0, 1 );
}

void fhe_mat3_copy( fhe_mat3_t to, const fhe_mat3_t from )
{
    int i;
    for ( i = 0; i < 9; ++i )
    {
        to[i] = from[i];
    }
}

double fhe_mat3_get( fhe_mat3_t m, int i, int j )
{
    FHE_ASSERT( i >= 0 && i < 3 && j >= 0 && j < 3 );
    return m[i * 3 + j];
}

void fhe_mat3_set( fhe_mat3_t m, int i, int j, double d )
{
    FHE_ASSERT( i >= 0 && i < 3 && j >= 0 && j < 3 );
    m[ i * 3 + j ] = d;
}

void fhe_mat3_mul_mat3( fhe_mat3_t r, fhe_mat3_t m1, fhe_mat3_t m2 )
{
    fhe_mat3_copy( r, FHE_MAT3_ZERO );
    int i, j, k;
    for ( i = 0; i < 3; ++i )
    {
        for ( j = 0; j < 3; ++j )
        {
            for ( k = 0; k < 3; ++k )
            {
                fhe_mat3_set( r, i, j, fhe_mat3_get( r, i, j ) + fhe_mat3_get( m1, i, k ) * fhe_mat3_get( m2, k, j ) );
            }
        }
    }
}

void fhe_mat3_mul_vec2( fhe_vec2_t* r, fhe_mat3_t m, fhe_vec2_t* v )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    fhe_vec2_init( r,
                   m[0] * v->x + m[1] * v->y + m[2],
                   m[3] * v->x + m[4] * v->y + m[5] );
}

double fhe_mat3_det( fhe_mat3_t m )
{
    return m[0] * (m[4] * m[8] - m[7] * m[5]) -
           m[1] * (m[3] * m[8] - m[6] * m[5]) +
           m[2] * (m[3] * m[7] - m[6] * m[4]);
}

void fhe_mat3_inverse( fhe_mat3_t i, fhe_mat3_t m )
{
    double d = fhe_mat3_det( m );
    fhe_mat3_init( i,
                    (m[4] * m[8] - m[5] * m[7]) / d,
                   -(m[1] * m[8] - m[7] * m[2]) / d,
                    (m[1] * m[5] - m[4] * m[2]) / d,
                   -(m[3] * m[8] - m[5] * m[6]) / d,
                    (m[0] * m[8] - m[6] * m[2]) / d,
                   -(m[0] * m[5] - m[3] * m[2]) / d,
                    (m[3] * m[7] - m[6] * m[4]) / d,
                   -(m[0] * m[7] - m[6] * m[1]) / d,
                    (m[0] * m[4] - m[1] * m[3]) / d);
}

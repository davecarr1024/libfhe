#include <fhe/vec2.h>
#include <fhe/util.h>
#include <math.h>

void fhe_vec2_init( fhe_vec2_t* v, double x, double y )
{
    FHE_ASSERT( v );
    v->x = x;
    v->y = y;
}

void fhe_vec2_init_rot( fhe_vec2_t* v, double th, double l )
{
    FHE_ASSERT( v );
    v->x = l * cos( th );
    v->y = l * sin( th );
}

void fhe_vec2_copy( fhe_vec2_t* to, const fhe_vec2_t* from )
{
    FHE_ASSERT( to );
    FHE_ASSERT( from );
    to->x = from->x;
    to->y = from->y;
}

void fhe_vec2_neg( fhe_vec2_t* r, fhe_vec2_t* v )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    r->x = -v->x;
    r->y = -v->y;
}

void fhe_vec2_add( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    r->x = v->x + u->x;
    r->y = v->y + u->y;
}

void fhe_vec2_sub( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    r->x = v->x - u->x;
    r->y = v->y - u->y;
}

void fhe_vec2_mul_double( fhe_vec2_t* r, fhe_vec2_t* v, double d )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    r->x = v->x * d;
    r->y = v->y * d;
}

void fhe_vec2_div_double( fhe_vec2_t* r, fhe_vec2_t* v, double d )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    r->x = v->x / d;
    r->y = v->y / d;
}

void fhe_vec2_mul_vec2( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    r->x = v->x * u->x;
    r->y = v->y * u->y;
}

void fhe_vec2_div_vec2( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    r->x = v->x / u->x;
    r->y = v->y / u->y;
}

void fhe_vec2_rotate( fhe_vec2_t* r, fhe_vec2_t* v, double th )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    fhe_vec2_init_rot( r, th + fhe_vec2_angle( v ), fhe_vec2_length( v ) );
}

void fhe_vec2_norm( fhe_vec2_t* r, fhe_vec2_t* v )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    double l = fhe_vec2_length( v );
    r->x = v->x / l;
    r->y = v->y / l;
}

void fhe_vec2_lerp( fhe_vec2_t* r, fhe_vec2_t* v, fhe_vec2_t* u, double i )
{
    FHE_ASSERT( r );
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    r->x = v->x + ( u->x - v->x ) * i;
    r->y = v->y + ( u->y - v->y ) * i;
}

void fhe_vec2_ineg( fhe_vec2_t* v )
{
    FHE_ASSERT( v );
    v->x = -v->x;
    v->y = -v->y;
}

void fhe_vec2_iadd( fhe_vec2_t* v, fhe_vec2_t* u )
{
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    v->x += u->x;
    v->y += u->y;
}

void fhe_vec2_isub( fhe_vec2_t* v, fhe_vec2_t* u )
{
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    v->x -= u->x;
    v->y -= u->y;
}

void fhe_vec2_imul_double( fhe_vec2_t* v, double d )
{
    FHE_ASSERT( v );
    v->x *= d;
    v->y *= d;
}

void fhe_vec2_idiv_double( fhe_vec2_t* v, double d )
{
    FHE_ASSERT( v );
    v->x /= d;
    v->y /= d;
}

void fhe_vec2_imul_vec2( fhe_vec2_t* v, fhe_vec2_t* u )
{
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    v->x *= u->x;
    v->y *= u->y;
}

void fhe_vec2_idiv_vec2( fhe_vec2_t* v, fhe_vec2_t* u )
{
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    v->x /= u->x;
    v->y /= u->y;
}

void fhe_vec2_irotate( fhe_vec2_t* v, double th )
{
    FHE_ASSERT( v );
    fhe_vec2_init_rot( v, th + fhe_vec2_angle( v ), fhe_vec2_length( v ) );
}

void fhe_vec2_inorm( fhe_vec2_t* v )
{
    FHE_ASSERT( v );
    double l = fhe_vec2_length( v );
    v->x /= l;
    v->y /= l;
}

void fhe_vec2_ilerp( fhe_vec2_t* v, fhe_vec2_t* u, double i )
{
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    v->x += ( u->x - v->x ) * i;
    v->y += ( u->y - v->y ) * i;
}

double fhe_vec2_angle( fhe_vec2_t* v )
{
    FHE_ASSERT( v );
    return atan2( v->y, v->x );
}

double fhe_vec2_length( fhe_vec2_t* v )
{
    FHE_ASSERT( v );
    return sqrt( v->x * v->x + v->y * v->y );
}

double fhe_vec2_dot( fhe_vec2_t* v, fhe_vec2_t* u )
{
    FHE_ASSERT( v );
    FHE_ASSERT( u );
    return v->x * u->x + v->y * u->y;
}


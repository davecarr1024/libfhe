#ifndef FHE_NODE_TYPE_H
#define FHE_NODE_TYPE_H

#include <glib.h>

#if GLIB_MAJOR_VERSION < 2 || GLIB_MINOR_VERSION < 26
#error require glib >= 2.26
#endif

typedef void(*fhe_node_func_t)( void* node, void* data );

typedef struct 
{
    const char* name;
    fhe_node_func_t func;
} fhe_node_type_entry_t;

enum { FHE_NODE_TYPE_MAX_FUNCS = 256 };

typedef struct fhe_node_type_t
{
    const struct fhe_node_type_t* parent;
    fhe_node_type_entry_t funcs[FHE_NODE_TYPE_MAX_FUNCS];
} fhe_node_type_t;

/**
* Add a library to search for node_type classes
*/
void fhe_node_type_add_file( const char* file );

/**
* Add all libraries in the given path for node_type searching
*/
void fhe_node_type_add_path( const char* path );

/**
* Get a node_type definition of the given name
*/
const fhe_node_type_t* fhe_node_type_get( const char* name );

#endif

#ifndef FHE_NODE_TYPE_H
#define FHE_NODE_TYPE_H

#include <glib.h>

/**
* Master enumeration of the various of types of events a node can receive
* To add a new event type, add a value to this enum
*/
typedef enum
{
    FHE_NODE_FUNC_ID_INIT,
    FHE_NODE_FUNC_ID_DESTROY,
    FHE_NODE_FUNC_ID_ON_ATTACH,
    FHE_NODE_FUNC_ID_ON_DETACH,
    FHE_NODE_FUNC_ID_TEST,
    FHE_NODE_FUNC_ID_INVALID,
    FHE_NODE_FUNC_ID_NUM
} fhe_node_func_id_t;

/**
* Function type for node events
* @note node is always a fhe_node_t*
*/
typedef void(*fhe_node_func_t)( void* node, void* data );

/**
* Entry to represent a mapping from fund_id to func for a given node type
*/
typedef struct 
{
    fhe_node_func_id_t id;
    fhe_node_func_t func;
} fhe_node_type_entry_t;

/**
* Represents a node type with parent type and event callbacks
*/
typedef struct fhe_node_type_t
{
    const struct fhe_node_type_t* parent;
    fhe_node_type_entry_t funcs[FHE_NODE_FUNC_ID_NUM];
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

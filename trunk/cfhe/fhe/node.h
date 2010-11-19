#ifndef FHE_NODE_H
#define FHE_NODE_H

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
* Represents a node
*/
typedef struct fhe_node_t
{
    gint refs;
    const fhe_node_type_t* type;
    char* name;
    struct fhe_node_t* parent;
    struct fhe_node_t** children;
    unsigned int n_children;
    void* data;
} fhe_node_t;

/**
* Build a new node
* @param parent of the new node, can be null to build a root node
* @param type the type of the new node
* @param name the name of the new node
* @return the new node
*/
fhe_node_t* fhe_node_init( fhe_node_t* parent, const fhe_node_type_t* type, const char* name );

const fhe_node_type_t* fhe_node_type_get( const char* name );

/**
* Acquire a reference to the given node
*/
void fhe_node_ref( fhe_node_t* node );

/**
* Release a reference to the given node, free the node if its refs are zero
*/
void fhe_node_unref( fhe_node_t* node );

/**
* Add child as a child of node
*/
void fhe_node_add_child( fhe_node_t* node, fhe_node_t* child );

/**
* Removes child from node
* @return success
*/
int fhe_node_remove_child( fhe_node_t* node, fhe_node_t* child );

/**
* Gets a child of node with name name, or null if not found
*/
fhe_node_t* fhe_node_get_child( fhe_node_t* node, const char* name );

/**
* Get the root of the tree node is in
*/
fhe_node_t* fhe_node_get_root( fhe_node_t* node );

/**
* Call an event
* @param node the node to call the event on
* @param id the id of the event
* @param data the argument to the event
*/
void fhe_node_call( fhe_node_t* node, fhe_node_func_id_t id, void* data );

/**
* Publish an event to this subtree
* @param node the root of the subtree to publish to
* @param id the id of the event
* @param data the argument to the event
*/
void fhe_node_publish( fhe_node_t* node, fhe_node_func_id_t id, void* data );

/**
* The basic node type. No parent type and no event callbacks.
*/
const fhe_node_type_t FHE_NODE =
{
    0,
    {
        { FHE_NODE_FUNC_ID_INVALID, 0 }
    }
};

#endif

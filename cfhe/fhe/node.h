#ifndef FHE_NODE_H
#define FHE_NODE_H

#include <fhe/node_type.h>

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
    GHashTable* funcs;
    GHashTable* vars;
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

/**
* Loads a node from a xml file
*/
fhe_node_t* fhe_node_load( const char* filename );

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
void fhe_node_call( fhe_node_t* node, const char* func, void* data );

/**
* Publish an event to this subtree
* @param node the root of the subtree to publish to
* @param id the id of the event
* @param data the argument to the event
*/
void fhe_node_publish( fhe_node_t* node, const char* func, void* data );

void fhe_node_var_set( fhe_node_t* node, const char* name, GVariant* val );
void fhe_node_var_set_boolean( fhe_node_t* node, const char* name, gboolean val );
void fhe_node_var_set_int( fhe_node_t* node, const char* name, int val );
void fhe_node_var_set_double( fhe_node_t* node, const char* name, double val );
void fhe_node_var_set_string( fhe_node_t* node, const char* name, const char* val );

GVariant* fhe_node_var_get( fhe_node_t* node, const char* name );
gboolean fhe_node_var_get_boolean( fhe_node_t* node, const char* name, gboolean def );
int fhe_node_var_get_int( fhe_node_t* node, const char* name, int def );
double fhe_node_var_get_double( fhe_node_t* node, const char* name, double def );
const char* fhe_node_var_get_string( fhe_node_t* node, const char* name, const char* def );

/**
* The basic node type. No parent type and no event callbacks.
*/
const fhe_node_type_t FHE_NODE =
{
    0,
    {
        { NULL, NULL }
    }
};

#endif

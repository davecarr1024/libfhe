#include <fhe/node.h>
#include <fhe/util.h>
#include <stdlib.h>
#include <string.h>

fhe_node_t* fhe_node_init( fhe_node_t* parent, const fhe_node_type_t* type, const char* name )
{
    FHE_ASSERT( type );
    fhe_node_t* node = (fhe_node_t*)malloc( sizeof( fhe_node_t ) );
    node->refs = 1;
    node->type = type;
    node->name = (char*)malloc( sizeof(char) * ( strlen( name ) + 1 ) );
    strcpy( node->name, name );
    node->parent = parent;
    node->children = 0;
    node->n_children = 0;
    fhe_node_call( node, FHE_NODE_FUNC_ID_INIT, NULL );
    if ( node->parent )
    {
        fhe_node_add_child( node->parent, node );
    }
    return node;
}

void fhe_node_ref( fhe_node_t* node )
{
    FHE_ASSERT( node );
    g_atomic_int_inc( &node->refs );
}

void fhe_node_unref( fhe_node_t* node )
{
    FHE_ASSERT( node );
    if ( TRUE != g_atomic_int_dec_and_test( &node->refs ) )
    {
        fhe_node_call( node, FHE_NODE_FUNC_ID_DESTROY, NULL );
        unsigned int i;
        for ( i = 0; i < node->n_children; ++i )
        {
            fhe_node_unref( node->children[i] );
        }
        free( node->children );
        free( node->name );
        free( node );
    }
}

void fhe_node_add_child( fhe_node_t* node, fhe_node_t* child )
{
    FHE_ASSERT( node );
    FHE_ASSERT( child );
    node->n_children++;
    node->children = (fhe_node_t**)realloc( node->children, sizeof( fhe_node_t* ) * node->n_children );
    node->children[node->n_children-1] = child;
    child->parent = node;
    fhe_node_ref( child );
    fhe_node_call( child, FHE_NODE_FUNC_ID_ON_ATTACH, node );
}

int fhe_node_remove_child( fhe_node_t* node, fhe_node_t* child )
{
    FHE_ASSERT( node );
    FHE_ASSERT( child );
    unsigned int i;
    for ( i = 0; i < node->n_children && node->children[i] != child; ++i );
    if ( i < node->n_children )
    {
        fhe_node_t* child = node->children[i];
        fhe_node_call( child, FHE_NODE_FUNC_ID_ON_DETACH, node );
        child->parent = 0;
        fhe_node_unref( child );
        for ( ; i < node->n_children - 1; ++i )
        {
            node->children[i] = node->children[i+1];
        }
        node->n_children--;
        node->children = (fhe_node_t**)realloc( node->children, sizeof( fhe_node_t* ) * node->n_children );
        return 1;
    }
    else
    {
        return 0;
    }
}

fhe_node_t* fhe_node_get_child( fhe_node_t* node, const char* name )
{
    FHE_ASSERT( node );
    unsigned int i;
    for ( i = 0; i < node->n_children; ++i )
    {
        if ( !strcmp( node->children[i]->name, name ) )
        {
            return node->children[i];
        }
    }
    return 0;
}

fhe_node_t* fhe_node_get_root( fhe_node_t* node )
{
    FHE_ASSERT( node );
    fhe_node_t* root;
    for ( root = node; root->parent; root = root->parent );
    return root;
}

void fhe_node_call( fhe_node_t* node, fhe_node_func_id_t id, void* data )
{
    FHE_ASSERT( node );
    const fhe_node_type_t* type;
    const fhe_node_type_entry_t* entry;
    for ( type = node->type; type; type = type->parent )
    {
        for ( entry = type->funcs; entry->id != FHE_NODE_FUNC_ID_INVALID; entry++ )
        {
            if ( entry->id == id )
            {
                entry->func( node, data );
            }
        }
    }
}

void fhe_node_publish( fhe_node_t* node, fhe_node_func_id_t id, void* data )
{
    FHE_ASSERT( node );
    fhe_node_call( node, id, data );
    unsigned int i;
    for ( i = 0; i < node->n_children; ++i )
    {
        fhe_node_publish( node->children[i], id, data );
    }
}

void fhe_node_start_element( GMarkupParseContext* context,
                             const gchar* element_name,
                             const gchar** attribute_names,
                             const gchar** attribute_values,
                             gpointer user_data,
                             GError **error )
{
    fhe_node_t** parent = (fhe_node_t**)user_data;
    
    FHE_ASSERT( parent );

    //don't load two roots
    FHE_ASSERT( !*parent || (*parent)->parent );
    
    if ( !strcmp( element_name, "node" ) )
    {
        const gchar* name = 0;
        const gchar* type = 0;
        
        const gchar **iname, **ival;
        for ( iname = attribute_names, ival = attribute_values; *iname && *ival; ++iname, ++ival )
        {
            if ( !strcmp( *iname, "name" ) )
            {
                name = *ival;
            }
            if ( !strcmp( *iname, "type" ) )
            {
                type = *ival;
            }
        }
        
        FHE_ASSERT_MSG( name, "attr name required in node tag" );
        FHE_ASSERT_MSG( type, "attr type required in node tag" );
                        
        const fhe_node_type_t* node_type = fhe_node_type_get( type );
        FHE_ASSERT_MSG( node_type, "unable to load node_type %s", type );
                        
        fhe_node_t* node = fhe_node_init( *parent, node_type, name );
        FHE_ASSERT_MSG( node, "unable to create node: type %s name %s", type, name );
        
        *parent = node;
    }
    else
    {
        FHE_ASSERT_MSG( 0, "invalid element_name %s", element_name );
    }
                                                 
}

void fhe_node_end_element( GMarkupParseContext* context,
                           const gchar* element_name,
                           gpointer user_data,
                           GError **error )
{
    fhe_node_t** node = (fhe_node_t**)user_data;
    
    if ( !strcmp( element_name, "node" ) )
    {
        FHE_ASSERT( *node );
        if ( (*node)->parent )
        {
            fhe_node_unref( *node );
            *node = (*node)->parent;
        }
    }
}

static const GMarkupParser fhe_node_markup_parser =
{
    fhe_node_start_element,
    fhe_node_end_element,
    NULL,
    NULL,
    NULL
};

fhe_node_t* fhe_node_load( const char* filename )
{
    fhe_node_t* node = 0;
    GMarkupParseContext* context = g_markup_parse_context_new( &fhe_node_markup_parser, 0, &node, NULL );
    FHE_ASSERT( context );
    
    gchar* text;
    gsize text_len;
    
    FHE_ASSERT_MSG( g_file_get_contents( filename, &text, &text_len, NULL ), "unable to read file %s", filename );
    
    FHE_ASSERT_MSG( g_markup_parse_context_parse( context, (const gchar*)text, text_len, NULL ), "unable to parse file %s", filename );
    
    g_free( text );
    
//     g_markup_parser_context_free( context );

    return node;
}

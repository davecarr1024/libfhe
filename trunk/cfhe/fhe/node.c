#include <fhe/node.h>
#include <fhe/util.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

void fhe_node_vars_key_free( char* key )
{
    free( key );
}

void fhe_node_vars_value_free( GVariant* value )
{
    g_variant_unref( value );
}

fhe_node_t* fhe_node_init( fhe_node_t* parent, const fhe_node_type_t* type, const char* name )
{
    FHE_ASSERT( type );
    fhe_node_t* node = (fhe_node_t*)malloc( sizeof( fhe_node_t ) );
    node->refs = 1;
    node->type = type;
    node->name = strdup( name );
    node->parent = 0;
    node->children = 0;
    node->n_children = 0;
    
    node->funcs = g_hash_table_new( g_str_hash, g_str_equal );
    const fhe_node_type_t* itype;
    const fhe_node_type_entry_t* entry;
    for ( itype = node->type; itype; itype = itype->parent )
    {
        for ( entry = itype->funcs; entry->name && entry->func; entry++ )
        {
            GSList* list = g_hash_table_lookup( node->funcs, entry->name );
            list = g_slist_append( list, entry->func );
            g_hash_table_replace( node->funcs, (gpointer)entry->name, list );
        }
    }
    
    node->vars = g_hash_table_new_full( g_str_hash, 
                                        g_str_equal, 
                                        (GDestroyNotify)fhe_node_vars_key_free, 
                                        (GDestroyNotify)fhe_node_vars_value_free );
    
    fhe_node_call( node, "init", NULL );
    if ( parent )
    {
        fhe_node_add_child( parent, node );
    }
    return node;
}

void fhe_node_ref( fhe_node_t* node )
{
    FHE_ASSERT( node );
    g_atomic_int_inc( &node->refs );
}

void fhe_node_func_free( const char* name, GSList* list, gpointer data )
{
    g_slist_free( list );
}

void fhe_node_unref( fhe_node_t* node )
{
    FHE_ASSERT( node );
    if ( g_atomic_int_dec_and_test( &node->refs ) )
    {
        fhe_node_call( node, "destroy", NULL );
        unsigned int i;
        for ( i = 0; i < node->n_children; ++i )
        {
            fhe_node_unref( node->children[i] );
        }
        g_hash_table_unref( node->vars );
        g_hash_table_foreach( node->funcs, (GHFunc)fhe_node_func_free, NULL );
        g_hash_table_unref( node->funcs );
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
    fhe_node_call( child, "attach", node );
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
        fhe_node_call( child, "detach", node );
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

void fhe_node_call( fhe_node_t* node, const char* func_name, void* data )
{
    FHE_ASSERT( node );
    GSList* list;
    for ( list = (GSList*)g_hash_table_lookup( node->funcs, func_name ); list; list = g_slist_next( list ) )
    {
        ((fhe_node_func_t)list->data)( node, data );
    }
}

void fhe_node_publish( fhe_node_t* node, const char* func, void* data )
{
    FHE_ASSERT( node );
    fhe_node_call( node, func, data );
    unsigned int i;
    for ( i = 0; i < node->n_children; ++i )
    {
        fhe_node_publish( node->children[i], func, data );
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
    else if ( !strcmp( element_name, "include" ) )
    {
        const gchar* file = 0;
        
        const gchar **iname, **ival;
        for ( iname = attribute_names, ival = attribute_values; *iname && *ival; ++iname, ++ival )
        {
            if ( !strcmp( *iname, "file" ) )
            {
                file = *ival;
            }
        }
        
        FHE_ASSERT_MSG( file, "attr file required in include tag" );
        
        fhe_node_t* node = fhe_node_load( file );
        FHE_ASSERT_MSG( node, "unable to load node from file %s", file );
        
        fhe_node_add_child( *parent, node );
        
        *parent = node;
    }
    else if ( !strcmp( element_name, "var" ) )
    {
        FHE_ASSERT_MSG( *parent, "var must be under a node tag" );
        
        const gchar* name = 0;
        const gchar* value = 0;
        
        const gchar **iname, **ival;
        for ( iname = attribute_names, ival = attribute_values; *iname && *ival; ++iname, ++ival )
        {
            if ( !strcmp( *iname, "name" ) )
            {
                name = *ival;
            }
            if ( !strcmp( *iname, "value" ) )
            {
                value = *ival;
            }
        }
        
        FHE_ASSERT_MSG( name, "attr name required in node tag" );
        FHE_ASSERT_MSG( value, "attr value required in node tag" );
        
        GVariant* val = g_variant_parse( NULL, value, NULL, NULL, NULL );
        FHE_ASSERT_MSG( val, "unable to parse value from %s", value );
        
        fhe_node_var_set( *parent, name, val );
    }
    else
    {
        FHE_ERROR( "invalid element_name %s", element_name );
    }
                                                 
}

void fhe_node_end_element( GMarkupParseContext* context,
                           const gchar* element_name,
                           gpointer user_data,
                           GError **error )
{
    fhe_node_t** node = (fhe_node_t**)user_data;
    
    if ( !strcmp( element_name, "node" ) || !strcmp( element_name, "include" ) )
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
    
    FHE_ASSERT_MSG( g_file_get_contents( filename, &text, &text_len, NULL ) && text, 
                    "unable to read file %s", filename );
    
    FHE_ASSERT_MSG( g_markup_parse_context_parse( context, (const gchar*)text, text_len, NULL ), 
                    "unable to parse file %s", filename );
    
    g_free( text );
    
    g_markup_parse_context_free( context );

    return node;
}

void fhe_node_var_set( fhe_node_t* node, const char* name, GVariant* val )
{
    FHE_ASSERT( node );
    g_variant_ref( val );
    g_hash_table_replace( node->vars, strdup( name ), val );
}

GVariant* fhe_node_var_get( fhe_node_t* node, const char* name )
{
    FHE_ASSERT( node );
    GVariant* val = g_hash_table_lookup( node->vars, name );
    return val;
}

void fhe_node_var_set_boolean( fhe_node_t* node, const char* name, gboolean val )
{
    fhe_node_var_set( node, name, g_variant_new_boolean( val ) );
}

gboolean fhe_node_var_get_boolean( fhe_node_t* node, const char* name, gboolean def )
{
    GVariant* val = fhe_node_var_get( node, name );
    return val && g_variant_is_of_type( val, G_VARIANT_TYPE_BOOLEAN ) ? g_variant_get_boolean( val ) : def;
}

void fhe_node_var_set_int( fhe_node_t* node, const char* name, int val )
{
    fhe_node_var_set( node, name, g_variant_new_int32( val ) );
}

int fhe_node_var_get_int( fhe_node_t* node, const char* name, int def )
{
    GVariant* val = fhe_node_var_get( node, name );
    return val && g_variant_is_of_type( val, G_VARIANT_TYPE_INT32 ) ? g_variant_get_int32( val ) : def;
}

void fhe_node_var_set_double( fhe_node_t* node, const char* name, double val )
{
    fhe_node_var_set( node, name, g_variant_new_double( val ) );
}

double fhe_node_var_get_double( fhe_node_t* node, const char* name, double def )
{
    GVariant* val = fhe_node_var_get( node, name );
    if ( val )
    {
        if ( g_variant_is_of_type( val, G_VARIANT_TYPE_DOUBLE ) )
        {
            return g_variant_get_double( val );
        }
        else if ( g_variant_is_of_type( val, G_VARIANT_TYPE_INT32 ) )
        {
            return g_variant_get_int32( val );
        }
    }
    return def;
}

void fhe_node_var_set_string( fhe_node_t* node, const char* name, const char* val )
{
    fhe_node_var_set( node, name, g_variant_new_string( val ) );
}

const char* fhe_node_var_get_string( fhe_node_t* node, const char* name, const char* def )
{
    GVariant* val = fhe_node_var_get( node, name );
    if ( val && g_variant_is_of_type( val, G_VARIANT_TYPE_STRING ) )
    {
        return g_variant_dup_string( val, NULL );
    }
    else if ( def )
    {
        return strdup( def );
    }
    else
    {
        return def;
    }
}

#include <fhe/node.h>
#include <fhe/util.h>
#include <gmodule.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

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
        fhe_node_call( node->children[i], FHE_NODE_FUNC_ID_ON_DETACH, node );
        fhe_node_unref( node->children[i] );
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

typedef struct
{
    char* name;
    GModule* mod;
} fhe_node_type_mod_entry_t;

static fhe_node_type_mod_entry_t* fhe_node_type_mods = 0;
static int fhe_node_type_n_mods = 0;

GModule* fhe_node_type_getMod( char* name )
{
    FHE_ASSERT_MSG( g_module_supported(), "gmodule not supported" );
    
    unsigned int i;
    for ( i = 0; i < fhe_node_type_n_mods; ++i )
    {
        if ( !strcmp( fhe_node_type_mods[i].name, name ) )
        {
            return fhe_node_type_mods[i].mod;
        }
    }
    fhe_node_type_n_mods++;
    fhe_node_type_mods = realloc( fhe_node_type_mods, sizeof( fhe_node_type_mod_entry_t ) * fhe_node_type_n_mods );
    GModule* mod = g_module_open( g_module_build_path( ".", (const gchar*)name ), G_MODULE_BIND_LAZY );
    FHE_ASSERT_MSG( mod, "unable to load mod %s", name );
    g_module_make_resident( mod );
    fhe_node_type_mods[fhe_node_type_n_mods].name = malloc( sizeof(char) * strlen( name ) );
    strcpy( fhe_node_type_mods[fhe_node_type_n_mods-1].name, name );
    fhe_node_type_mods[fhe_node_type_n_mods-1].mod = mod;
    return mod;
}

void fhe_node_type_add_file( const char* file )
{
    const char* basename = strrchr( file, '/' );
    if ( NULL == basename )
    {
        basename = file;
    }
    int len = strcspn( basename, G_DIR_SEPARATOR_S );
    char name[len];
    strncpy( name, basename, len );
}

void fhe_node_type_add_path( const char* path )
{
    GDir* dir = g_dir_open( path, 0, NULL );
    FHE_ASSERT_MSG( dir, "unable to open path %s", path );
    const char* file;
    for ( file = g_dir_read_name( dir ); file; file = g_dir_read_name( dir ) )
    {
        const char* ext = strrchr( file, '.' );
        if ( ext && !strcmp( ext + 1, G_MODULE_SUFFIX ) )
        {
            printf( "load %s\n", file );
            fhe_node_type_add_file( file );
        }
        else if ( g_file_test( file, G_FILE_TEST_IS_DIR ) )
        {
            fhe_node_type_add_path( file );
        }
    }
}

const fhe_node_type_t* fhe_node_type_get( const char* _typename )
{
    char typename[1024], modname[1024], name[1024];
    strncpy( typename, _typename, 1024 );
    
    char* tok = strtok( (char*)typename, "." );
    FHE_ASSERT_MSG( tok, "unable to tok modname out of %s", typename );
    strncpy( modname, tok, 1024 );
    
    tok = strtok( NULL, "." );
    FHE_ASSERT_MSG( tok, "unable to tok typename out of %s", typename );
    strncpy( name, tok, 1024 );
    
    GModule* mod = fhe_node_type_getMod( modname );
    FHE_ASSERT_MSG( mod, "unable to load mod %s", modname );
    
    gpointer type = 0;
    g_module_symbol( mod, (const gchar*)name, &type );
    FHE_ASSERT_MSG( type, "error loading symbol %s from mod %s: %s\n", name, modname, g_module_error() );
    return (const fhe_node_type_t*)(type);
}

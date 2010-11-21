#include <fhe/node_type.h>
#include <fhe/util.h>
#include <gmodule.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

GModule** fhe_node_type_mods;
int fhe_node_type_n_mods = 0;
int fhe_node_type_initialized = 0;

void fhe_node_type_add_file( const char* file )
{
    fhe_node_type_n_mods++;
    fhe_node_type_mods = (GModule**)realloc( (void*)fhe_node_type_mods, sizeof( GModule* ) * fhe_node_type_n_mods );
    GModule* mod = g_module_open( file, G_MODULE_BIND_LAZY );
    FHE_ASSERT_MSG( mod, "unable to load mod %s", file );
    g_module_make_resident( mod );
    fhe_node_type_mods[fhe_node_type_n_mods-1] = mod;
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
            fhe_node_type_add_file( file );
        }
        else if ( g_file_test( file, G_FILE_TEST_IS_DIR ) )
        {
            fhe_node_type_add_path( file );
        }
    }
}

void fhe_node_type_init()
{
    if ( !fhe_node_type_initialized )
    {
        FHE_ASSERT_MSG( g_module_supported(), "gmodule not supported" );
        
        fhe_node_type_add_file( NULL );
        fhe_node_type_add_path( "." );
        
        fhe_node_type_initialized = 1;
    }
}

const fhe_node_type_t* fhe_node_type_get( const char* name )
{
    fhe_node_type_init();
    const fhe_node_type_t* type = 0;
    int i;
    for ( i = 0; i < fhe_node_type_n_mods && !type; ++i )
    {
        GModule* mod = fhe_node_type_mods[i];
        
        g_module_symbol( mod, name, (gpointer*)&type );
    }
    FHE_ASSERT_MSG( type, "unable to load node_type %s", name );
    return type;
}

#include <gtest/gtest.h>
extern "C" 
{
    #include <test/common/test_node.h>
}

TEST( node_test, tree )
{
    fhe_node_t* root = fhe_node_init( 0, &FHE_NODE, "root" );
    ASSERT_TRUE( root );
    ASSERT_FALSE( root->parent );
    ASSERT_EQ( 0, root->n_children );
    ASSERT_STREQ( "root", root->name );
    
    fhe_node_t* child = fhe_node_init( root, &FHE_NODE, "child" );
    
    ASSERT_TRUE( child );
    ASSERT_EQ( 0, child->n_children );
    ASSERT_STREQ( "child", child->name );
    ASSERT_EQ( root, child->parent );
    ASSERT_EQ( 1, root->n_children );
    ASSERT_EQ( child, fhe_node_get_child( root, "child" ) );
    ASSERT_EQ( root, fhe_node_get_root( child ) );
    
    ASSERT_TRUE( fhe_node_remove_child( root, child ) );
    
    ASSERT_FALSE( fhe_node_get_child( root, "child" ) );
    
    fhe_node_add_child( root, child );
    
    ASSERT_TRUE( fhe_node_get_child( root, "child" ) );
    
    fhe_node_unref( child );
    fhe_node_unref( root );
}

TEST( node_test, call )
{
    fhe_node_t* node = fhe_node_init( 0, &FHE_TEST_NODE, "node" );
    
    TestData data = {0,0};
    fhe_node_call( node, "test", &data );
    ASSERT_EQ( 3, data.i );
    
    fhe_node_unref( node );
}

TEST( node_test, child_call )
{
    fhe_node_t* node = fhe_node_init( 0, &FHE_TEST_CHILD_NODE, "node" );
    
    TestData data = {0,0};
    fhe_node_call( node, "test", &data );
    ASSERT_EQ( 3, data.i );
    ASSERT_EQ( 4, data.j );
    
    fhe_node_unref( node );
}

TEST( node_test, publish )
{
    fhe_node_t* root = fhe_node_init( 0, &FHE_NODE, "root" );
    fhe_node_t* node = fhe_node_init( root, &FHE_TEST_CHILD_NODE, "node" );
    
    TestData data = {0,0};
    fhe_node_publish( root, "test", &data );
    ASSERT_EQ( 3, data.i );
    ASSERT_EQ( 4, data.j );
    
    fhe_node_unref( node );
    fhe_node_unref( root );
}

TEST( node_test, get_type )
{
    const fhe_node_type_t* type = fhe_node_type_get( "FHE_NODE" );
    ASSERT_TRUE( type );
    ASSERT_FALSE( type->parent );
    ASSERT_FALSE( type->funcs[0].name );
    ASSERT_FALSE( type->funcs[0].func );
}

TEST( node_test, file )
{
    fhe_node_t* node = fhe_node_load( "./test/node_test.xml" );
    ASSERT_TRUE( node );
    
    ASSERT_STREQ( "file_node", node->name );
    
    ASSERT_TRUE( fhe_node_var_get_boolean( node, "b", FALSE ) );
    ASSERT_EQ( 12, fhe_node_var_get_int( node, "i", 0 ) );
    ASSERT_DOUBLE_EQ( 3.14, fhe_node_var_get_double( node, "d", 3.14 ) );
    const char* s = fhe_node_var_get_string( node, "s", NULL );
    ASSERT_TRUE( s );
    ASSERT_STREQ( "what", s );

    TestData data = {0,0};
    fhe_node_publish( node, "test", &data );
    ASSERT_EQ( 3, data.i );
    ASSERT_EQ( 4, data.j );
    
    fhe_node_t* child = fhe_node_get_child( node, "file_child" );
    ASSERT_TRUE( child );
    fhe_node_ref( child );
    
    fhe_node_t* include_node = fhe_node_get_child( node, "include_node" );
    ASSERT_TRUE( include_node );
    fhe_node_ref( include_node );
    
    fhe_node_unref( include_node );
    fhe_node_unref( child );
    fhe_node_unref( node );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}

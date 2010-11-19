#include <gtest/gtest.h>
extern "C" 
{
    #include <fhe/node.h>
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
    
    fhe_node_unref( child );
    fhe_node_unref( root );
}

typedef struct
{
    int i, j;
} TestData;

void fhe_test_node_test( fhe_node_t* node, TestData* data )
{
    data->i = 3;
}

const fhe_node_type_t FHE_TEST_NODE =
{
    &FHE_NODE,
    {
        { FHE_NODE_FUNC_ID_TEST, (fhe_node_func_t)&fhe_test_node_test },
        { FHE_NODE_FUNC_ID_INVALID, 0 }
    }
};

TEST( node_test, call )
{
    fhe_node_t* node = fhe_node_init( 0, &FHE_TEST_NODE, "node" );
    
    TestData data = {0,0};
    fhe_node_call( node, FHE_NODE_FUNC_ID_TEST, &data );
    ASSERT_EQ( 3, data.i );
    
    fhe_node_unref( node );
}

void fhe_test_child_node_test( fhe_node_t* node, TestData* data )
{
    data->j = 4;
}

const fhe_node_type_t FHE_TEST_CHILD_NODE =
{
    &FHE_TEST_NODE,
    {
        { FHE_NODE_FUNC_ID_TEST, (fhe_node_func_t)&fhe_test_child_node_test },
        { FHE_NODE_FUNC_ID_INVALID, 0 }
    }
};

TEST( node_test, child_call )
{
    fhe_node_t* node = fhe_node_init( 0, &FHE_TEST_CHILD_NODE, "node" );
    
    TestData data = {0,0};
    fhe_node_call( node, FHE_NODE_FUNC_ID_TEST, &data );
    ASSERT_EQ( 3, data.i );
    ASSERT_EQ( 4, data.j );
    
    fhe_node_unref( node );
}

TEST( node_test, publish )
{
    fhe_node_t* root = fhe_node_init( 0, &FHE_NODE, "root" );
    fhe_node_t* node = fhe_node_init( root, &FHE_TEST_CHILD_NODE, "node" );
    
    TestData data = {0,0};
    fhe_node_publish( root, FHE_NODE_FUNC_ID_TEST, &data );
    ASSERT_EQ( 3, data.i );
    ASSERT_EQ( 4, data.j );
    
    fhe_node_unref( node );
    fhe_node_unref( root );
}

TEST( node_test, get_type )
{
    const fhe_node_type_t* type = fhe_node_type_get( "fhe.FHE_NODE" );
    ASSERT_TRUE( type );
    ASSERT_FALSE( type->parent );
    ASSERT_EQ( FHE_NODE_FUNC_ID_INVALID, type->funcs[0].id );
    ASSERT_FALSE( type->funcs[0].func );
}

int main( int argc, char** argv )
{
    testing::InitGoogleTest( &argc, argv );
    return RUN_ALL_TESTS();
}

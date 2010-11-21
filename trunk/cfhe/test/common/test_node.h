#ifndef FHE_TEST_NODE_H
#define FHE_TEST_NODE_H

#include <fhe/node.h>

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

#endif

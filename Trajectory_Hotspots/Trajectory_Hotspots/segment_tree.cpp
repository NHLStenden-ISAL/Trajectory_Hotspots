#include "pch.h"
#include "segment_tree.h"

#include "aabb.h"

class Segment_Tree_Node
{


    Segment_Tree_Node* left;
    Segment_Tree_Node* right;

    AABB bounding_box;
};

class Segment_Tree
{
    Segment_Tree_Node root;
};
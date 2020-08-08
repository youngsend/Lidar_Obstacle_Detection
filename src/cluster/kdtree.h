/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
            :	point(arr), id(setId), left(nullptr), right(nullptr)
    {}
};

struct KdTree
{
    Node* root;

    KdTree()
            : root(nullptr)
    {}

    void insertHelper(Node **node, std::vector<float> point, int id, int depth) {
        if (!(*node)) {
            *node = new Node(point, id);
        } else {
            // odd layer, compare y; even layer, compare x
            if (point[depth%2] < (*node)->point[depth%2]) {
                insertHelper(&((*node)->left), point, id, depth + 1);
            } else {
                insertHelper(&((*node)->right), point, id, depth + 1);
            }
        }
    }

    void insert(std::vector<float> point, int id)
    {
        // Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        insertHelper(&root, point, id, 0);
    }

    // ** いらない！
    void searchHelper(Node *node, std::vector<int>& ids, std::vector<float> target, float distanceTol, int depth){
        if (node) {
            // when *node is not nullptr
            int index = depth % 2;
            if (std::fabs(target[index]-node->point[index]) <= distanceTol){
                float distanceX = std::fabs(target[0] - node->point[0]);
                float distanceY = std::fabs(target[1] - node->point[1]);
                if (distanceX * distanceX + distanceY * distanceY <= distanceTol * distanceTol){
                    ids.push_back(node->id);
                }

                // need to search both left and right child because the box region cross both sides
                searchHelper(node->left, ids, target, distanceTol, depth+1);
                searchHelper(node->right, ids, target, distanceTol, depth+1);
            } else {
                // need only search one side
                if (target[index] < node->point[index]) {
                    searchHelper(node->left, ids, target, distanceTol, depth+1);
                } else {
                    searchHelper(node->right, ids, target, distanceTol, depth+1);
                }
            }
        }
    }

// return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root, ids, target, distanceTol, 0);
        return ids;
    }


};





/* \author Aaron Brown */

#include <vector>
#include <cmath>

// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
    PointT point;
    int id;
    Node* left;
    Node* right;

    Node(PointT arr, int setId)
            :	point(std::move(arr)), id(setId), left(nullptr), right(nullptr)
    {}
};

template <typename PointT>
struct KdTree
{
    Node<PointT>* root;

    KdTree() : root(nullptr) {
    }

    static void insertHelper(Node<PointT> **node, PointT point, int id, int depth) {
        if (!(*node)) {
            *node = new Node<PointT>(point, id);
        } else {
            // depth%3==0, compare x; depth%3==1, compare y; depth%3=2, compare z.
            if (point.data[depth%3] < ((*node)->point).data[depth%3]) {
                insertHelper(&((*node)->left), point, id, depth + 1);
            } else {
                insertHelper(&((*node)->right), point, id, depth + 1);
            }
        }
    }

    void insert(PointT point, int id)
    {
        // Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        insertHelper(&root, std::move(point), id, 0);
    }

    static void searchHelper(Node<PointT> *node, std::vector<int>& ids, PointT target,
                             float distanceTol, int depth){
        if (node) {
            // when *node is not nullptr
            int index = depth % 3;
            if (std::fabs(target.data[index]-(node->point).data[index]) <= distanceTol){
                float distanceX = std::fabs(target.x - (node->point).x);
                float distanceY = std::fabs(target.y - (node->point).y);
                float distanceZ = std::fabs(target.z - (node->point).z);
                if (distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ
                    <= distanceTol*distanceTol){
                    ids.push_back(node->id);
                }

                // need to search both left and right child because the box region cross both sides
                searchHelper(node->left, ids, target, distanceTol, depth+1);
                searchHelper(node->right, ids, target, distanceTol, depth+1);
            } else {
                // need only search one side
                if (target.data[index] < (node->point).data[index]) {
                    searchHelper(node->left, ids, target, distanceTol, depth+1);
                } else {
                    searchHelper(node->right, ids, target, distanceTol, depth+1);
                }
            }
        }
    }

// return a list of point ids in the tree that are within distance of target
    std::vector<int> search(PointT target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(root, ids, std::move(target), distanceTol, 0);
        return ids;
    }
};





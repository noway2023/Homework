#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{   
    
    time_t start, stop;
    time(&start);
    switch(splitMethod){
        case SplitMethod::NAIVE:
            root = recursiveBuild(primitives);
            break;
        case SplitMethod::SAH:
            root = SAHBuild(primitives);
            break;
    }
    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::SAHBuild(std::vector<Object*> objects){
    BVHBuildNode* node = new BVHBuildNode();
    Bounds3 bounds;
    for(int i=0;i<objects.size();i++){
        bounds = Union(bounds, objects[i]->getBounds());
        if(objects.size()==1){
            node->bounds = objects[0]->getBounds();
            node->object = objects[0];
            node->left = nullptr;
            node->right = nullptr;
            return node;
        }
        else if(objects.size()==2){
            node->left = recursiveBuild(std::vector{objects[0]});
            node->right = recursiveBuild(std::vector{objects[1]});
            node->bounds = Union(node->left->bounds, node->right->bounds);
            return node;
        }
    // if (objects.size()<=maxPrimsInNode){
    //     Bounds3 mergedBounds;
    //     for(int j=0; j<objects.size(); j++){
    //         mergedBounds = Union(mergedBounds, objects[j]->getBounds());
            
    //         node->objectlist.push_back(objects[j]);
    //     }
    //     node->bounds = mergedBounds;
    //     node->left = nullptr;
    //     node->right = nullptr;
    //     return node;
    // }
        else{
            Bounds3 centroidBounds;
            for (int i = 0; i < objects.size(); ++i)
                centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
            int dim = centroidBounds.maxExtent();
            switch (dim) {
            case 0:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                        f2->getBounds().Centroid().x;
                });
                break;
            case 1:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                        f2->getBounds().Centroid().y;
                });
                break;
            case 2:
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().z <
                        f2->getBounds().Centroid().z;
                });
                break;
            }
            std::vector<double> timespend;
            for(int i=0;i<objects.size();i++){
                auto beginning = objects.begin();
                auto middling = objects.begin() + i;
                auto ending = objects.end();

                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);
                Bounds3 bounds1, bounds2;
                assert(objects.size() == (leftshapes.size() + rightshapes.size()));

                node->left = recursiveBuild(leftshapes);
                node->right = recursiveBuild(rightshapes);
                for(int j=0;j<leftshapes.size();j++){
                    bounds1 = Union(bounds1, leftshapes[0]->getBounds());
                }
                for(int j=0;j<rightshapes.size();j++){
                    bounds2 = Union(bounds2, rightshapes[0]->getBounds());
                }
                double bounds_area = bounds.SurfaceArea();
                double bounds1_area = bounds1.SurfaceArea();
                double bounds2_area = bounds2.SurfaceArea();
                double time = ( bounds1_area /bounds_area * leftshapes.size() )
                        + ( bounds2_area /bounds_area * rightshapes.size() );
                    //+ 0.148f;
                timespend.push_back(time);
            }
            int minindex = std::min_element(timespend.begin(), timespend.end()) - timespend.begin();
            auto beginning = objects.begin();
            auto middling = objects.begin() + minindex;
            auto ending = objects.end();

            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);

            assert(objects.size() == (leftshapes.size() + rightshapes.size()));

            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);

            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
    }
    return node;
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
        if (objects.size() == 1) {
            // Create leaf _BVHBuildNode_
            node->bounds = objects[0]->getBounds();
            node->object = objects[0];
            node->left = nullptr;
            node->right = nullptr;
            return node;
        }
        else if (objects.size() == 2) {
            node->left = recursiveBuild(std::vector{objects[0]});
            node->right = recursiveBuild(std::vector{objects[1]});

            node->bounds = Union(node->left->bounds, node->right->bounds);
            return node;
        }
    // if (objects.size()<=maxPrimsInNode){
    //     Bounds3 mergedBounds;
    //     for(int j=0; j<objects.size(); j++){
    //         mergedBounds = Union(mergedBounds, objects[j]->getBounds());
            
    //         node->objectlist.push_back(objects[j]);
    //     }
    //     node->bounds = mergedBounds;
    //     node->left = nullptr;
    //     node->right = nullptr;
    //     return node;
    // }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
        
    // TODO Traverse the BVH to find intersection
    Intersection result;
    
    std::array<int, 3> dirIsNeg{ray.direction[0]>0, ray.direction[1]>0, ray.direction[2]>0};
    if(!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)){
        return result;
    }
    if(!node->left && !node->right){
        return node->object->getIntersection(ray);

        // double dis= std::numeric_limits<double>::max();
        // for(int i=0; i<node->objectlist.size(); i++){
        //     Intersection temp = node->objectlist[i]->getIntersection(ray);
        //     if(temp.distance<dis){
        //         dis = temp.distance;
        //         result = temp;
        //     }
        // }
        // return result;
    }

    Intersection h1 = getIntersection(node->left, ray);
    Intersection h2 = getIntersection(node->right, ray);
    return h1.distance<h2.distance?h1:h2;
}
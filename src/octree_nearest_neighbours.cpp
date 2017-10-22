#include "octree_nearest_neighbours.h"
#include "utils/Logger.hh"

//! Start-of-the-program initializer for the sequence map.
struct Initializer {
  Initializer() {
    for(unsigned char mask = 0; mask < 256; mask++)
    {
      for(unsigned char index = 0; index < 8; index++)
      {
        char c = 0;
        char *mimap = imap[index];  // maps area index to preference
        for(unsigned char i = 0; i < 8; i++)
        {
          if(( 1 << i ) & mask)   // if ith node exists
            sequence2ci[index][mask][ mimap[i] ] = c++;
          else
            sequence2ci[index][mask][ mimap[i] ] = -1;
        }
      }
      if (mask == UCHAR_MAX) break;
    }
  }
};

namespace{
Initializer init;
}

char sequence2ci[8][256][8] = {};

char amap[8][8] = {
    {0, 1, 2, 4, 3, 5, 6, 7 },
    {1, 0, 3, 5, 2, 4, 6, 7 },
    {2, 0, 3, 6, 1, 4, 5, 7 },
    {3, 1, 2, 7, 0, 5, 4, 6 },
    {4, 5, 6, 0, 7, 1, 2, 3 },
    {5, 4, 7, 1, 6, 0, 3, 2 },
    {6, 4, 7, 2, 5, 0, 3, 1 },
    {7, 5, 6, 3, 4, 1, 2, 0 } };

char imap[8][8] = {
    {0, 1, 2, 4, 3, 5, 6, 7 },
    {1, 0, 4, 2, 5, 3, 6, 7 },
    {1, 4, 0, 2, 5, 6, 3, 7 },
    {4, 1, 2, 0, 6, 5, 7, 3 },
    {3, 5, 6, 7, 0, 1, 2, 4 },
    {5, 3, 7, 6, 1, 0, 4, 2 },
    {5, 7, 3, 6, 1, 4, 0, 2 },
    {7, 5, 6, 3, 4, 1, 2, 0 } };

void childcenter(Point p, Point& cp, char i, int size) {
  switch (i) {
    case 0:  // 000
    cp.x = p.x - size ;
    cp.y = p.y - size ;
    cp.z = p.z - size ;
    break;
    case 1:  // 001
      cp.x = p.x + size ;
      cp.y = p.y - size ;
      cp.z = p.z - size ;
      break;
    case 2:  // 010
      cp.x = p.x - size ;
      cp.y = p.y + size ;
      cp.z = p.z - size ;
      break;
    case 3:  // 011
      cp.x = p.x + size ;
      cp.y = p.y + size ;
      cp.z = p.z - size ;
      break;
    case 4:  // 100
      cp.x = p.x - size ;
      cp.y = p.y - size ;
      cp.z = p.z + size ;
      break;
    case 5:  // 101
      cp.x = p.x + size ;
      cp.y = p.y - size ;
      cp.z = p.z + size ;
      break;
    case 6:  // 110
      cp.x = p.x - size ;
      cp.y = p.y + size ;
      cp.z = p.z + size ;
      break;
    case 7:  // 111
      cp.x = p.x + size ;
      cp.y = p.y + size ;
      cp.z = p.z + size ;
      break;
    default:
      break;
  }
}

// compute which child is closest to the query point
int getClosestChild(const Point& q, const Point& p)
{
  return ((q.x - p.x) >= 0) |
      (((q.y - p.y) >= 0) << 1) |
      (((q.z - p.z) >= 0) << 2);
}

//  void FindClosest(
//          OcTree& tree,
//          OcTreeNode &node,
//          int size,
//          Point pi,
//          NNParams& params)
//  {
//    int child_index = getClosestChild(params.q, pi);
//    LOG_TEST() << "Child: " << child_index << "\n";
//
//    // maps preference to index in children array
////     char *seq2ci = sequence2ci[child_index][node.valid];
//
//    char *mmap = amap[child_index];  // maps preference to area index
//
////     bitunion<T> *children;
////     bitoct::getChildren(node, children);
////     Point cp;
////
////     for (unsigned char i = 0; i < 8; i++)
////     { // in order of preference
////       child_index = mmap[i]; // the area index of the node
////       if (( 1 << child_index ) & node.valid ) {   // if ith node exists
////         childcenter(pi, cp, child_index, size);
////         if ( params.closest_v == 0 ||
////                 std::max(std::max(abs( cp.x - params.q.x ),
////                  abs( cp.y - params.q.y )),
////                  abs( cp.z - params.q.z )) - size
////         > params.closest_v ) {
////           continue;
////         }
////         // find the closest point in leaf seq2ci[i]
////         if (  ( 1 << child_index ) & node.leaf )
////         {   // if ith node is leaf
//////           findClosestInLeaf(&children[seq2ci[i]]);
////         }
////         else
////         { // recurse
////           FindClosest(children[seq2ci[i]].node, size/2, cp, params);
////         }
////       }
////     }
//  }

struct L2Distance
{
  static inline float compute(const Point& p, const Point& q)
  {
    return std::pow(p.x - q.x, 2) +
        std::pow(p.y - q.y, 2) +
        std::pow(p.z - q.z, 2);
  }

  static inline float norm(float x, float y, float z)
  {
    return std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2);
  }

  static inline float sqr(float r)
  {
    return r * r;
  }

  static inline float sqrt(float r)
  {
    return std::sqrt(r);
  }
};

int getKeyDepth(const OcTree& tree, const octomap::point3d& point, const OcTreeKey& key)
{
  for(int depth = tree.getTreeDepth(); depth > 1; --depth)
  {
    if (tree.coordToKey(point, depth) == key)
      return depth;
  }
  return -1;
}

bool contains(OcTree& tree, const Point& query, float sqRadius, const OcTreeKey& o)
{
  // we exploit the symmetry to reduce the test to test
  // whether the farthest corner is inside the search ball.
  auto p = tree.keyToCoord(o);

  auto x = std::abs(query.x - p.x());
  auto y = std::abs(query.y - p.y());
  auto z = std::abs(query.z - p.z());
  // reminder: (x, y, z) - (-e, -e, -e) = (x, y, z) + (e, e, e)
  auto depth = getKeyDepth(tree, p, o);
  auto half_size = tree.getNodeSize(depth) / 2;
  x += half_size;
  y += half_size;
  z += half_size;

  return (L2Distance::norm(x, y, z) < sqRadius);
}



//  template <typename ContainerT>
//  void radiusNeighbors(OcTree& tree, const OcTreeNode& octant, const Point& query, float radius,
//                                                   float sqrRadius, std::vector<uint32_t>& resultIndices,
//                                                   std::vector<float>& distances) const
//  {
//    const ContainerT& points = *data_;
//
//    // if search ball S(q,r) contains octant, simply add point indexes and compute squared distances.
//    if (contains(tree, query, sqrRadius, octant))
//    {
//      uint32_t idx = octant->start;
//      for (uint32_t i = 0; i < octant->size; ++i)
//      {
//        resultIndices.push_back(idx);
//        distances.push_back(L2Distance::compute(query, points[idx]));
//        idx = successors_[idx];
//      }
//
//      return;  // early pruning.
//    }
//
//    if (octant->isLeaf)
//    {
//      uint32_t idx = octant->start;
//      for (uint32_t i = 0; i < octant->size; ++i)
//      {
//        const PointT& p = points[idx];
//        float dist = L2Distance::compute(query, p);
//        if (dist < sqrRadius)
//        {
//          resultIndices.push_back(idx);
//          distances.push_back(dist);
//        }
//        idx = successors_[idx];
//      }
//
//      return;
//    }
//    // check whether child nodes are in range.
////    for (uint32_t c = 0; c < 8; ++c)
////    {
////      if (octant->child[c] == 0) continue;
////      if (!overlaps<L2Distance>(query, radius, sqrRadius, octant->child[c])) continue;
////      radiusNeighbors<L2Distance>(octant->child[c], query, radius, sqrRadius, resultIndices, distances);
////    }
//  }


double getVoxelSquaredSideLen (OcTree& tree, unsigned int tree_depth_arg)
{
  // side length of the voxel cube increases exponentially with the octree depth
  double side_len = tree.getResolution() * static_cast<double>(1 << (tree.getTreeDepth() - tree_depth_arg));
  // squared voxel side length
  return (side_len * side_len);
}

double getVoxelSquaredDiameter (OcTree& tree, unsigned int tree_depth_arg)
{
  // return the squared side length of the voxel cube as a function of the octree depth
  return (getVoxelSquaredSideLen(tree, tree_depth_arg) * 3);
}

float squaredNorm(const Point& p)
{
  return std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2);
}

float pointSquaredDist(const Point& p, const Point& q)
{
  Point x(p.x - q.x, p.y - q.y, p.z - q.z);
  return squaredNorm(x);
}

//  template<typename PointT, typename LeafContainerT, typename BranchContainerT, typename OctreeT> void
//  pcl::octree::OctreePointCloud<PointT, LeafContainerT, BranchContainerT, OctreeT>::genVoxelCenterFromOctreeKey (
//      const OctreeKey & key_arg,
//      unsigned int tree_depth_arg,
//      PointT& point_arg) const
//  {
//    // generate point for voxel center defined by treedepth (bitLen) and key
//    point_arg.x = static_cast<float> ((static_cast <double> (key_arg.x) + 0.5f) * (this->resolution_ * static_cast<double> (1 << (this->octree_depth_ - tree_depth_arg))) + this->min_x_);
//    point_arg.y = static_cast<float> ((static_cast <double> (key_arg.y) + 0.5f) * (this->resolution_ * static_cast<double> (1 << (this->octree_depth_ - tree_depth_arg))) + this->min_y_);
//    point_arg.z = static_cast<float> ((static_cast <double> (key_arg.z) + 0.5f) * (this->resolution_ * static_cast<double> (1 << (this->octree_depth_ - tree_depth_arg))) + this->min_z_);
//  }

void getNeighborsWithinRadiusRecursive (OcTree& tree,
                                        const Point& point, const double radiusSquared, const OcTreeNode* node, const OcTreeKey& key,
                                        unsigned int tree_depth, std::vector<int>& k_indices, std::vector<float>& k_sqr_distances,
                                        unsigned int max_nn)
{
  // get spatial voxel information
  double voxel_squared_diameter =  getVoxelSquaredDiameter(tree, tree_depth);

  // iterate over all children
  for (unsigned char child_idx = 0; child_idx < 8; child_idx++)
  {
    if (!tree.nodeChildExists(node, child_idx))
      continue;

    const OcTreeNode* child_node = tree.getNodeChild(node, child_idx);

    // generate new key for current branch voxel
    key_type t;
    OcTreeKey new_key;
    computeChildKey(child_idx, t, key, new_key);
    //      new_key.x = (key.x << 1) + (!!(child_idx & (1 << 2)));
    //      new_key.y = (key.y << 1) + (!!(child_idx & (1 << 1)));
    //      new_key.z = (key.z << 1) + (!!(child_idx & (1 << 0)));
    //
    //      // generate voxel center point for voxel at key
    //      genVoxelCenterFromOctreeKey (new_key, tree_depth, voxel_center);

    // calculate distance to search point
    auto c = tree.keyToCoord(new_key);
    Point voxel_center(c.x(), c.y(), c.z());
    float squared_dist = pointSquaredDist(voxel_center, point);

    // if distance is smaller than search radius
    const double epsilon = 0.001;
    if (squared_dist + epsilon
        <= voxel_squared_diameter / 4.0 + radiusSquared + sqrt (voxel_squared_diameter * radiusSquared))
    {
      if (tree_depth < tree.getTreeDepth())
      {
        // we have not reached maximum tree depth
        getNeighborsWithinRadiusRecursive(tree, point, radiusSquared, child_node, new_key, tree_depth + 1,
                                          k_indices, k_sqr_distances, max_nn);
        if (max_nn != 0 && k_indices.size () == static_cast<unsigned int> (max_nn))
          return;
      }
      else          // we reached leaf node level
      {
        if (squared_dist <= radiusSquared)
        {
          //            k_indices.push_back ();
          k_sqr_distances.push_back (squared_dist);
        }

        //          size_t i;
        //          const LeafNode* child_leaf = static_cast<const LeafNode*> (child_node);
        //          std::vector<int> decoded_point_vector;

        // decode leaf node into decoded_point_vector
        //          (*child_leaf)->getPointIndices (decoded_point_vector);

        // Linearly iterate over all decoded (unsorted) points
        //          for (i = 0; i < decoded_point_vector.size (); i++)
        {
          //            const Point& candidate_point = this->getPointByIndex (decoded_point_vector[i]);

          // calculate point distance to search point
          //            squared_dist = pointSquaredDist(candidate_point, point);

          // check if a match is found
          //            if (squared_dist > radiusSquared)
          //              continue;
          //
          //            // add point to result vector
          //            k_indices.push_back (decoded_point_vector[i]);
          //            k_sqr_distances.push_back (squared_dist);

          if (max_nn != 0 && k_indices.size () == static_cast<unsigned int> (max_nn))
            return;
        }
      }
    }
  }
}

void radiusSearch (OcTree& tree, const Point &p_q, const double radius, std::vector<int> &k_indices,
                   std::vector<float> &k_sqr_distances, unsigned int max_nn)
{
  assert (isFinite (p_q) && "Invalid (NaN, Inf) point coordinates given to nearestKSearch!");
  OcTreeKey key;
  key.k[0] = key.k[1] = key.k[2] = 0;

  k_indices.clear ();
  k_sqr_distances.clear ();

  getNeighborsWithinRadiusRecursive (tree, p_q, radius * radius, tree.getRoot(), key, 1, k_indices, k_sqr_distances,
                                     max_nn);
}




  #include <octomap/octomap.h>
  #include <pcl/point_types.h>
  #include <pcl/io/pcd_io.h>

  using namespace octomap;
  using Point = pcl::PointXYZ;
  using PointCloud = pcl::PointCloud<Point>;
  struct NNParams {
/** 
   * pointer to the closest point.  size = 4 bytes of 32 bit machines 
   */
  void *closest;

  /** 
   * distance to the closest point. size = 8 bytes 
   */
  double closest_d2;

  // distance to the closest point in voxels
  int closest_v;

  // location of the query point in voxel coordinates
  Point q;

  /** 
   * pointer to the point, size = 4 bytes of 32 bit machines 
   */
  double *p;

  int count;
  int max_count;
};

  // initialized in Boctree.cc, sequence intialized on startup
  extern char amap[8][8];
  extern char imap[8][8];
  extern char sequence2ci[8][256][8];  // maps preference to index in children array for every valid_mask and every case

  
//   template <class P>
//   inline unsigned char childIndex(const T *center, const P *point) {
//     return  (point[0] > center[0] ) | ((point[1] > center[1] ) << 1) | ((point[2] > center[2] ) << 2) ;
//   }

/**
 * Computes the <i>squared</i> Eucledian distance between two points
 * in 3-space
 *
 * @param x1   first input vector
 * @param x2   decond input vecotr
 * @return  Eucledian distance^2 between the two locations
 */
template <class T, class F>
inline T Dist2(const T *x1, const F *x2)
{
  T dx = x2[0] - x1[0];
  T dy = x2[1] - x1[1];
  T dz = x2[2] - x1[2];

  return sqr(dx) + sqr(dy) + sqr(dz);
}

//   //! Leaf node: points in the array
//   inline T* getPoints() const {
//     // absolute pointer
//     //return &(this->points[1].v);
//     // offset pointer
//     return reinterpret_cast<T*>(
//       reinterpret_cast<pointrep*>((char*)this + node.child_pointer) + 1
//     );
//   }
//   
//   //! Leaf node: length in the array
//   inline unsigned int getLength() const {
//     // absolute pointer
//     //return this->points[0].length;
//     // offset pointer
//     return (reinterpret_cast<pointrep*>((char*)this + node.child_pointer))[0].length;
//   }

  /**
   * Given a leaf node, this function looks for the closest point to params.closest
   * in the list of points.
   */
//  inline void findClosestInLeaf(OcTreeNode *node, NNParams& params) const {
//    if (params.count >= params.max_count) return;
//    params.count++;
//
//    T* points = node->getPoints();
//    unsigned int length = node->getLength();
//    for(unsigned int iterator = 0; iterator < length; iterator++ ) {
//      double myd2 = Dist2(params.p, points);
//      if (myd2 < params.closest_d2) {
//        params.closest_d2 = myd2;
//        params.closest = points;
//        if (myd2 <= 0.0001) {
//          params.closest_v = 0; // the search radius in units of voxelSize
//        } else {
//          params.closest_v = sqrt(myd2) * mult + 1; // the search radius in units of voxelSize
//        }
//      }
//      points+=BOctTree<T>::POINTDIM;
//    }
//  }

//   inline void findClosestInLeaf(bitunion<T> *node, int threadNum) const {
//     if (params.count >= params.max_count) return;
//     params.count++;
//     T* points = node->getPoints();
//     unsigned int length = node->getLength();
//     for(unsigned int iterator = 0; iterator < length; iterator++ ) {
//       double myd2 = Dist2(params.p, points); 
//       if (myd2 < params.closest_d2) {
//         params.closest_d2 = myd2;
//         params.closest = points;
//         if (myd2 <= 0.0001) {
//           params.closest_v = 0; // the search radius in units of voxelSize
//         } else {
//           params.closest_v = sqrt(myd2) * mult + 1; // the search radius in units of voxelSize
//         }
//       }
//       points+=BOctTree<T>::POINTDIM;
//     }
//   }
void childcenter(Point p, Point& cp, char i, int size);

void radiusSearch (OcTree& tree, const Point &p_q, const double radius, std::vector<int> &k_indices,
                   std::vector<float> &k_sqr_distances, unsigned int max_nn);

float squaredNorm(const Point& p);
  /**
   * This is the heavy duty search function doing most of
   * the (theoretically unneccesary) work. The tree is recursively searched.
   * Depending on which of the 8 child-voxels is closer to the query point,
   * the children are examined in a special order.
   * This order is defined in map, imap is its inverse and sequence2ci
   * is a speedup structure for faster access to the child indices.
   */
//   void FindClosest(OcTree& tree,
//           OcTreeNode &node,
//           int size,
//           Point pi,
//           NNParams& params);

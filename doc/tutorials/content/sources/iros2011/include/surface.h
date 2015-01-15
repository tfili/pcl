#ifndef SURFACE_H_
#define SURFACE_H_

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>

#include "typedefs.h"


class Mesh
{
public:
  Mesh () : points (new PointCloud) {}
  PointCloudPtr points;
  std::vector<pcl::Vertices> faces;
};

typedef boost::shared_ptr<Mesh> MeshPtr;

PointCloudPtr
smoothPointCloud (const PointCloudPtr & input, double radius, int polynomial_order)
{
  PointCloudPtr output (new PointCloud);
  return (output);
}

SurfaceElementsPtr
computeSurfaceElements (const PointCloudPtr & input, double radius, int polynomial_order)
{
  SurfaceElementsPtr surfels (new SurfaceElements);
  return (surfels);
}

MeshPtr
computeConvexHull (const PointCloudPtr & input)
{
  MeshPtr output (new Mesh);
  return (output);
}


MeshPtr
computeConcaveHull (const PointCloudPtr & input, double alpha)
{
  MeshPtr output (new Mesh);
  return (output);
}

pcl::PolygonMesh::Ptr
greedyTriangulation (const SurfaceElementsPtr & surfels, double radius, double mu, int max_nearest_neighbors, 
                     double max_surface_angle, double min_angle, double max_angle)

{
  pcl::PolygonMesh::Ptr output (new pcl::PolygonMesh);
  return (output);
}


pcl::PolygonMesh::Ptr
marchingCubesTriangulation (const SurfaceElementsPtr & surfels, double leaf_size, double iso_level)
{
  pcl::PolygonMesh::Ptr output (new pcl::PolygonMesh);
  return (output);
}

#endif

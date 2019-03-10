#ifndef UTIL_H
#define UTIL_H
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/TextureMesh.h>
#include <pcl/pcl_base.h>
//#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/search/kdtree.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include "rgbd_mapping/TexturedQuad.h"
#include "rgbd_mapping/TexturedQuadArray.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
float getDepth(
        const cv::Mat & depthImage,
        float x, float y,
        bool smoothing,
        float maxZError = 0.02f,
        bool estWithNeighborsIfNull = false);

pcl::PointXYZ projectDepthTo3D(
        cv::Mat & depthImage,
        float x, float y,
        float cx, float cy,
        float fx, float fy,
        bool smoothing,
        float maxZError = 0.02f);

pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        int searchK,
        float searchRadius,
        const Eigen::Vector3f & viewPoint);

pcl::PolygonMesh::Ptr meshDecimation(const pcl::PolygonMesh::Ptr & mesh, float factor);

void createPolygonIndexes(
        const std::vector<pcl::Vertices> & polygons,
        int cloudSize,
        std::vector<std::set<int> > & neighbors,
        std::vector<std::set<int> > & vertexToPolygons);

std::list<std::list<int> > clusterPolygons(
        const std::vector<std::set<int> > & neighborPolygons,
        int minClusterSize);

void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pCloud, const geometry_msgs::Pose & msg);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        float voxelSize);

pcl::IndicesPtr radiusFiltering(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        float radiusSearch,
        int minNeighborsInRadius);

pcl::IndicesPtr radiusFiltering(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        float radiusSearch,
        int minNeighborsInRadius);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
void appendMesh(
        pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudA,
        std::vector<pcl::Vertices> & polygonsA,
        pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudB,
        std::vector<pcl::Vertices> & polygonsB);
void getCenterOfCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pCloud, Eigen::Vector3f &vCenter);
template<typename pointRGBT>
void denseMeshPostProcessing(
        pcl::PolygonMeshPtr & mesh,
        float meshDecimationFactor,
        int maximumPolygons,
        const typename pcl::PointCloud<pointRGBT>::Ptr & cloud,
        float transferColorRadius,
        bool coloredOutput,
        bool cleanMesh,
        int minClusterSize)
{
    // compute normals for the mesh if not already here
    bool hasNormals = false;
    bool hasColors = false;
    for(unsigned int i=0; i<mesh->cloud.fields.size(); ++i)
    {
        if(mesh->cloud.fields[i].name.compare("normal_x") == 0)
        {
            hasNormals = true;
        }
        else if(mesh->cloud.fields[i].name.compare("rgb") == 0)
        {
            hasColors = true;
        }
    }

    if(maximumPolygons > 0)
    {
        double factor = 1.0-double(maximumPolygons)/double(mesh->polygons.size());
        if(factor > meshDecimationFactor)
        {
            meshDecimationFactor = factor;
        }
    }
    if(meshDecimationFactor > 0.0)
    {
        unsigned int count = mesh->polygons.size();
        mesh = meshDecimation(mesh, (float)meshDecimationFactor);
        hasNormals = false;
        hasColors = false;
    }

    if(cloud.get()!=0 &&
        !hasColors &&
        transferColorRadius >= 0.0)
    {
        // transfer color from point cloud to mesh
        typename pcl::search::KdTree<pointRGBT>::Ptr tree (new pcl::search::KdTree<pointRGBT>(true));
        tree->setInputCloud(cloud);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::fromPCLPointCloud2(mesh->cloud, *coloredCloud);
        std::vector<bool> coloredPts(coloredCloud->size());
        for(unsigned int i=0; i<coloredCloud->size(); ++i)
        {
            std::vector<int> kIndices;
            std::vector<float> kDistances;
            pointRGBT pt;
            pt.x = coloredCloud->at(i).x;
            pt.y = coloredCloud->at(i).y;
            pt.z = coloredCloud->at(i).z;
            if(transferColorRadius > 0.0)
            {
                tree->radiusSearch(pt, transferColorRadius, kIndices, kDistances);
            }
            else
            {
                tree->nearestKSearch(pt, 1, kIndices, kDistances);
            }
            if(kIndices.size())
            {
                //compute average color
                int r=0;
                int g=0;
                int b=0;
                int a=0;
                for(unsigned int j=0; j<kIndices.size(); ++j)
                {
                    r+=(int)cloud->at(kIndices[j]).r;
                    g+=(int)cloud->at(kIndices[j]).g;
                    b+=(int)cloud->at(kIndices[j]).b;
                    a+=(int)cloud->at(kIndices[j]).a;
                }
                coloredCloud->at(i).r = r/kIndices.size();
                coloredCloud->at(i).g = g/kIndices.size();
                coloredCloud->at(i).b = b/kIndices.size();
                coloredCloud->at(i).a = a/kIndices.size();
                coloredPts.at(i) = true;
            }
            else
            {
                //white
                coloredCloud->at(i).r = coloredCloud->at(i).g = coloredCloud->at(i).b = 255;
                coloredPts.at(i) = false;
            }
        }
        pcl::toPCLPointCloud2(*coloredCloud, mesh->cloud);

        // remove polygons with no color
        if(cleanMesh)
        {
            std::vector<pcl::Vertices> filteredPolygons(mesh->polygons.size());
            int oi=0;
            for(unsigned int i=0; i<mesh->polygons.size(); ++i)
            {
                bool coloredPolygon = true;
                for(unsigned int j=0; j<mesh->polygons[i].vertices.size(); ++j)
                {
                    if(!coloredPts.at(mesh->polygons[i].vertices[j]))
                    {
                        coloredPolygon = false;
                        break;
                    }
                }
                if(coloredPolygon)
                {
                    filteredPolygons[oi++] = mesh->polygons[i];
                }
            }
            filteredPolygons.resize(oi);
            mesh->polygons = filteredPolygons;
        }
        hasColors = true;
    }

    if(minClusterSize)
    {
        // filter polygons
        std::vector<std::set<int> > neighbors;
        std::vector<std::set<int> > vertexToPolygons;
        createPolygonIndexes(mesh->polygons,
                mesh->cloud.height*mesh->cloud.width,
                neighbors,
                vertexToPolygons);
        std::list<std::list<int> > clusters = clusterPolygons(
                neighbors,
                minClusterSize<0?0:minClusterSize);

        std::vector<pcl::Vertices> filteredPolygons(mesh->polygons.size());
        if(minClusterSize < 0)
        {
            // only keep the biggest cluster
            std::list<std::list<int> >::iterator biggestClusterIndex = clusters.end();
            unsigned int biggestClusterSize = 0;
            for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
            {
                if(iter->size() > biggestClusterSize)
                {
                    biggestClusterIndex = iter;
                    biggestClusterSize = iter->size();
                }
            }
            if(biggestClusterIndex != clusters.end())
            {
                int oi=0;
                for(std::list<int>::iterator jter=biggestClusterIndex->begin(); jter!=biggestClusterIndex->end(); ++jter)
                {
                    filteredPolygons[oi++] = mesh->polygons.at(*jter);
                }
                filteredPolygons.resize(oi);
            }
        }
        else
        {
            int oi=0;
            for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
            {
                for(std::list<int>::iterator jter=iter->begin(); jter!=iter->end(); ++jter)
                {
                    filteredPolygons[oi++] = mesh->polygons.at(*jter);
                }
            }
            filteredPolygons.resize(oi);
        }

        int before = (int)mesh->polygons.size();
        mesh->polygons = filteredPolygons;
    }

    // compute normals for the mesh if not already here, add also white color if colored output is required
    if(!hasNormals || (!hasColors && coloredOutput))
    {
        // use polygons
        if(hasColors || coloredOutput)
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

            Eigen::Vector3f normal(1,0,0);
            for(unsigned int i=0; i<mesh->polygons.size(); ++i)
            {
                pcl::Vertices & v = mesh->polygons[i];
                if(!hasNormals)
                {
                    //UASSERT(v.vertices.size()>2);
                    /*if(v.vertices.size()>2)
                    {
                        PCL_WARN("util::denseMeshPostProcessing::v.vertices.size:%d", v.vertices.size());
                    }*/
                    Eigen::Vector3f v0(
                            cloud->at(v.vertices[1]).x - cloud->at(v.vertices[0]).x,
                            cloud->at(v.vertices[1]).y - cloud->at(v.vertices[0]).y,
                            cloud->at(v.vertices[1]).z - cloud->at(v.vertices[0]).z);
                    int last = v.vertices.size()-1;
                    Eigen::Vector3f v1(
                            cloud->at(v.vertices[last]).x - cloud->at(v.vertices[0]).x,
                            cloud->at(v.vertices[last]).y - cloud->at(v.vertices[0]).y,
                            cloud->at(v.vertices[last]).z - cloud->at(v.vertices[0]).z);
                    normal = v0.cross(v1);
                    normal.normalize();
                }
                // flat normal (per face)
                for(unsigned int j=0; j<v.vertices.size(); ++j)
                {
                    if(!hasNormals)
                    {
                        cloud->at(v.vertices[j]).normal_x = normal[0];
                        cloud->at(v.vertices[j]).normal_y = normal[1];
                        cloud->at(v.vertices[j]).normal_z = normal[2];
                    }
                    if(!hasColors)
                    {
                        cloud->at(v.vertices[j]).r = 255;
                        cloud->at(v.vertices[j]).g = 255;
                        cloud->at(v.vertices[j]).b = 255;
                    }
                }
            }
            pcl::toPCLPointCloud2 (*cloud, mesh->cloud);
        }
        else
        {
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
            pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

            for(unsigned int i=0; i<mesh->polygons.size(); ++i)
            {
                pcl::Vertices & v = mesh->polygons[i];
                //UASSERT(v.vertices.size()>2);
                /*if(v.vertices.size()>2)
                {
                    PCL_WARN("util::denseMeshPostProcessing::v.vertices.size:%d", v.vertices.size());
                }*/
                Eigen::Vector3f v0(
                        cloud->at(v.vertices[1]).x - cloud->at(v.vertices[0]).x,
                        cloud->at(v.vertices[1]).y - cloud->at(v.vertices[0]).y,
                        cloud->at(v.vertices[1]).z - cloud->at(v.vertices[0]).z);
                int last = v.vertices.size()-1;
                Eigen::Vector3f v1(
                        cloud->at(v.vertices[last]).x - cloud->at(v.vertices[0]).x,
                        cloud->at(v.vertices[last]).y - cloud->at(v.vertices[0]).y,
                        cloud->at(v.vertices[last]).z - cloud->at(v.vertices[0]).z);
                Eigen::Vector3f normal = v0.cross(v1);
                normal.normalize();
                // flat normal (per face)
                for(unsigned int j=0; j<v.vertices.size(); ++j)
                {
                    cloud->at(v.vertices[j]).normal_x = normal[0];
                    cloud->at(v.vertices[j]).normal_y = normal[1];
                    cloud->at(v.vertices[j]).normal_z = normal[2];
                }
            }
            pcl::toPCLPointCloud2 (*cloud, mesh->cloud);
        }
    }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(
        const cv::Mat & imageRgb,
        const cv::Mat & imageDepthIn,
        float fx, float cx, float fy, float cy,
        int decimation,
        float maxDepth,
        float minDepth,
        std::vector<int> * validIndices);

template<class T>
inline bool uIsFinite(const T & value)
{
#if _MSC_VER
    return _finite(value) != 0;
#else
    return std::isfinite(value);
#endif
}
float distanceToPlane(Eigen::Vector3f &p, float a, float b, float c, float d);
void transformInversePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pCloud, const geometry_msgs::Pose & msg);
bool getPointUVCoordinates(pcl::PointXYZ &pt, float fx, float fy, float cx, float cy, int width, int height, pcl::PointXY &uv_coordinates);
Quaterniond QuaternionRot(Vector3d x1, Vector3d y1, Vector3d z1,
                          Vector3d x2, Vector3d y2, Vector3d z2);
void computeTextureCoordsOfQuad(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud,
                                float fx, float fy, float cx, float cy,
                                int width, int height,
                                Eigen::Vector2f &uv_min, Eigen::Vector2f &uv_max);
float distanceFromPointToPlane(Eigen::Vector3f &pt, Eigen::Vector3f* vertices, Eigen::Vector3f &vNormal);
void transformInverseVertices(std::vector<Eigen::Vector3f> &vertices, const geometry_msgs::Pose & pose);
bool getIntersectionOfQuad(Eigen::Vector3f &p1, Eigen::Vector3f &p2, std::vector<Eigen::Vector3f> &vertices, Eigen::Vector3f &interPos);
/**/
void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, const geometry_msgs::Pose & msg);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
        const cv::Mat & imageDepthIn,
        float depthFx, float depthCx, float depthFy, float depthCy,
        int decimation,
        float maxDepth,
        float minDepth,
        std::vector<int> * validIndices);
void filteringCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & pclCloud, pcl::IndicesPtr & indices, float voxelSize,
                    float noiseFilterRadius, int noiseFilterMinNeighbors);
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        float voxelSize);
pcl::IndicesPtr radiusFiltering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        float radiusSearch,
        int minNeighborsInRadius);
pcl::IndicesPtr radiusFiltering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
        float radiusSearch,
        int minNeighborsInRadius);
float getIntersectionOfPlane(Eigen::Vector3f &p1, Eigen::Vector3f &p2, float* coefficients, Eigen::Vector3f &interPos);
//void getOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, std::vector<Eigen::Vector3f> &vertices, Eigen::Vector3f &vHorizen);
void downSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_input,
                  float leaf_size,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
void computingBoundingBox(std::vector<Eigen::Vector3f> &vertices_input,
                          Eigen::Vector3f &vDir,
                          std::vector<Eigen::Vector3f> &vertices);
bool intersectPlane(Eigen::Vector3f &vOrigin, Eigen::Vector3f &vDir, Eigen::Vector4f &planeCoeffs);
float getTop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud);
#endif // UTIL_H

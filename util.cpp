#include "util.h"
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
//#include <pcl18/surface/texture_mapping.h>
#include <pcl/features/integral_image_normal.h>

#ifndef DISABLE_VTK
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#endif

/*#if PCL_VERSION_COMPARE(<, 1, 8, 0)
#include "pcl18/surface/organized_fast_mesh.h"
#else*/
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/impl/marching_cubes.hpp>
#include <pcl/surface/impl/organized_fast_mesh.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
/*
created at 12/15/2017
*/
float getDepth(
        cv::Mat & depthImage,
        float x, float y,
        bool smoothing,
        float maxZError,
        bool estWithNeighborsIfNull)
{
    //UASSERT(!depthImage.empty());
    //UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);
    int u = int(x+0.5f);
    int v = int(y+0.5f);
    if(u == depthImage.cols && x<float(depthImage.cols))
    {
        u = depthImage.cols - 1;
    }
    if(v == depthImage.rows && y<float(depthImage.rows))
    {
        v = depthImage.rows - 1;
    }

    if(!(u >=0 && u<depthImage.cols && v >=0 && v<depthImage.rows))
    {
        return 0;
    }

    bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

    // Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
    // https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
    // Window weights:
    //  | 1 | 2 | 1 |
    //  | 2 | 4 | 2 |
    //  | 1 | 2 | 1 |
    int u_start = std::max(u-1, 0);
    int v_start = std::max(v-1, 0);
    int u_end = std::min(u+1, depthImage.cols-1);
    int v_end = std::min(v+1, depthImage.rows-1);

    float depth = 0.0f;
    if(isInMM)
    {
        if(depthImage.at<unsigned short>(v,u) > 0 &&
           depthImage.at<unsigned short>(v,u) < std::numeric_limits<unsigned short>::max())
        {
            depth = float(depthImage.at<unsigned short>(v,u))*0.001f;
        }
    }
    else
    {
        depth = depthImage.at<float>(v,u);
    }

    if((depth==0.0f || !uIsFinite(depth)) && estWithNeighborsIfNull)
    {
        // all cells no2 must be under the zError to be accepted
        float tmp = 0.0f;
        int count = 0;
        for(int uu = u_start; uu <= u_end; ++uu)
        {
            for(int vv = v_start; vv <= v_end; ++vv)
            {
                if((uu == u && vv!=v) || (uu != u && vv==v))
                {
                    float d = 0.0f;
                    if(isInMM)
                    {
                        if(depthImage.at<unsigned short>(vv,uu) > 0 &&
                           depthImage.at<unsigned short>(vv,uu) < std::numeric_limits<unsigned short>::max())
                        {
                            d = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
                        }
                    }
                    else
                    {
                        d = depthImage.at<float>(vv,uu);
                    }
                    if(d!=0.0f && uIsFinite(d))
                    {
                        if(tmp == 0.0f)
                        {
                            tmp = d;
                            ++count;
                        }
                        else if(fabs(d - tmp/float(count)) < maxZError)
                        {
                            tmp += d;
                            ++count;
                        }
                    }
                }
            }
        }
        if(count > 1)
        {
            depth = tmp/float(count);
        }
    }

    if(depth!=0.0f && uIsFinite(depth))
    {
        if(smoothing)
        {
            float sumWeights = 0.0f;
            float sumDepths = 0.0f;
            for(int uu = u_start; uu <= u_end; ++uu)
            {
                for(int vv = v_start; vv <= v_end; ++vv)
                {
                    if(!(uu == u && vv == v))
                    {
                        float d = 0.0f;
                        if(isInMM)
                        {
                            if(depthImage.at<unsigned short>(vv,uu) > 0 &&
                               depthImage.at<unsigned short>(vv,uu) < std::numeric_limits<unsigned short>::max())
                            {
                                d = float(depthImage.at<unsigned short>(vv,uu))*0.001f;
                            }
                        }
                        else
                        {
                            d = depthImage.at<float>(vv,uu);
                        }

                        // ignore if not valid or depth difference is too high
                        if(d != 0.0f && uIsFinite(d) && fabs(d - depth) < maxZError)
                        {
                            if(uu == u || vv == v)
                            {
                                sumWeights+=2.0f;
                                d*=2.0f;
                            }
                            else
                            {
                                sumWeights+=1.0f;
                            }
                            sumDepths += d;
                        }
                    }
                }
            }
            // set window weight to center point
            depth *= 4.0f;
            sumWeights += 4.0f;

            // mean
            depth = (depth+sumDepths)/sumWeights;
        }
    }
    else
    {
        depth = 0;
    }
    return depth;
}
/*
created at 12/16/2017
*/
pcl::PointXYZ projectDepthTo3D(
        cv::Mat & depthImage,
        float x, float y,
        float cx, float cy,
        float fx, float fy,
        bool smoothing,
        float maxZError)
{
    //UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);
    pcl::PointXYZ pt;

    float depth = getDepth(depthImage, x, y, smoothing, maxZError, false);
    if(depth > 0.0f)
    {
        // Use correct principal point from calibration
        cx = cx > 0.0f ? cx : float(depthImage.cols/2) - 0.5f; //cameraInfo.K.at(2)
        cy = cy > 0.0f ? cy : float(depthImage.rows/2) - 0.5f; //cameraInfo.K.at(5)

        // Fill in XYZ
        pt.x = (x - cx) * depth / fx;
        pt.y = (y - cy) * depth / fy;
        pt.z = depth;
    }
    else
    {
        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
    }
    return pt;
}
/*
created at 12/16/2017
*/
pcl::PointCloud<pcl::Normal>::Ptr computeNormals(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        int searchK,
        float searchRadius,
        const Eigen::Vector3f & viewPoint)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    if(indices->size())
    {
        tree->setInputCloud(cloud, indices);
    }
    else
    {
        tree->setInputCloud (cloud);
    }

    // Normal estimation*
#ifdef PCL_OMP
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> n;
#else
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
#endif
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    n.setInputCloud (cloud);
    // Commented: Keep the output normals size the same as the input cloud
    //if(indices->size())
    //{
    //	n.setIndices(indices);
    //}
    n.setSearchMethod (tree);
    n.setKSearch (searchK);
    n.setRadiusSearch(searchRadius);
    n.setViewPoint(viewPoint[0], viewPoint[1], viewPoint[2]);
    n.compute (*normals);

    return normals;
}
/*
created at 12/16/2017
*/
pcl::PolygonMesh::Ptr meshDecimation(const pcl::PolygonMesh::Ptr & mesh, float factor)
{
    pcl::PolygonMesh::Ptr output(new pcl::PolygonMesh);
#ifndef DISABLE_VTK
    pcl::MeshQuadricDecimationVTK mqd;
    mqd.setTargetReductionFactor(factor);
    mqd.setInputMesh(mesh);
    mqd.process (*output);
#else
    UWARN("RTAB-Map is not built with VTK module so mesh decimation cannot be used!");
    *output = *mesh;
#endif
    return output;
}
/*
created at 12/16/2017
*/
void createPolygonIndexes(
        const std::vector<pcl::Vertices> & polygons,
        int cloudSize,
        std::vector<std::set<int> > & neighbors,
        std::vector<std::set<int> > & vertexToPolygons)
{
    vertexToPolygons = std::vector<std::set<int> >(cloudSize);
    neighbors = std::vector<std::set<int> >(polygons.size());

    for(unsigned int i=0; i<polygons.size(); ++i)
    {
        std::set<int> vertices(polygons[i].vertices.begin(), polygons[i].vertices.end());

        for(unsigned int j=0; j<polygons[i].vertices.size(); ++j)
        {
            int v = polygons[i].vertices.at(j);
            for(std::set<int>::iterator iter=vertexToPolygons[v].begin(); iter!=vertexToPolygons[v].end(); ++iter)
            {
                int numSharedVertices = 0;
                for(unsigned int k=0; k<polygons.at(*iter).vertices.size() && numSharedVertices<2; ++k)
                {
                    if(vertices.find(polygons.at(*iter).vertices.at(k)) != vertices.end())
                    {
                        ++numSharedVertices;
                    }
                }
                if(numSharedVertices >= 2)
                {
                    neighbors[*iter].insert(i);
                    neighbors[i].insert(*iter);
                }
            }
            vertexToPolygons[v].insert(i);
        }
    }
}
/*
created at 12/16/2017
*/
std::list<std::list<int> > clusterPolygons(
        const std::vector<std::set<int> > & neighborPolygons,
        int minClusterSize)
{
    std::set<int> polygonsChecked;

    std::list<std::list<int> > clusters;

    for(unsigned int i=0; i<neighborPolygons.size(); ++i)
    {
        if(polygonsChecked.find(i) == polygonsChecked.end())
        {
            std::list<int> currentCluster;
            currentCluster.push_back(i);
            polygonsChecked.insert(i);

            for(std::list<int>::iterator iter=currentCluster.begin(); iter!=currentCluster.end(); ++iter)
            {
                // get neighbor polygons
                std::set<int> neighbors = neighborPolygons[*iter];
                for(std::set<int>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
                {
                    if(polygonsChecked.insert(*jter).second)
                    {
                        currentCluster.push_back(*jter);
                    }
                }
            }
            if((int)currentCluster.size() > minClusterSize)
            {
                clusters.push_back(currentCluster);
            }
        }
    }
    return clusters;
}
/*
created at 12/16/2017
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(
        const cv::Mat & imageRgb,
        const cv::Mat & imageDepthIn,
        float fx, float cx, float fy, float cy,
        int decimation,
        float maxDepth,
        float minDepth,
        std::vector<int> * validIndices)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    cv::Mat imageDepth = imageDepthIn;
    bool mono;
    PCL_WARN("start-channels:%d", imageRgb.channels());
    if(imageRgb.channels() == 3) // BGR
    {
        mono = false;
    }
    else if(imageRgb.channels() == 1) // Mono
    {
        mono = true;
    }
    else
    {
        return cloud;
    }
    //cloud.header = cameraInfo.header;
    cloud->height = imageDepth.rows/decimation;
    cloud->width  = imageDepth.cols/decimation;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    if(validIndices)
    {
        validIndices->resize(cloud->size());
    }

    float rgbToDepthFactorX = float(imageRgb.cols) / float(imageDepth.cols);
    float rgbToDepthFactorY = float(imageRgb.rows) / float(imageDepth.rows);
    float depthFx = fx / rgbToDepthFactorX;//model.fx()
    float depthFy = fy / rgbToDepthFactorY;//model.fy()
    float depthCx = cx / rgbToDepthFactorX;//model.cx()
    float depthCy = cy / rgbToDepthFactorY;//model.cy()
    ROS_INFO("depthFx:%f, depthFy:%f, depthCx:%f, depthCy:%f", depthFx, depthFy, depthCx, depthCy);
    int oi = 0;
    for(int h = 0; h < imageDepth.rows && h/decimation < (int)cloud->height; h+=decimation)
    {
        for(int w = 0; w < imageDepth.cols && w/decimation < (int)cloud->width; w+=decimation)
        {
            pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

            int x = int(w*rgbToDepthFactorX);
            int y = int(h*rgbToDepthFactorY);
            //UASSERT(x >=0 && x<imageRgb.cols && y >=0 && y<imageRgb.rows);
            if(!mono)
            {
                const unsigned char * bgr = imageRgb.ptr<unsigned char>(y,x);
                pt.b = bgr[0];
                pt.g = bgr[1];
                pt.r = bgr[2];
            }
            else
            {
                unsigned char v = imageRgb.at<unsigned char>(y,x);
                pt.b = v;
                pt.g = v;
                pt.r = v;
            }

            pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w, h, depthCx, depthCy, depthFx, depthFy, false);
            if (pcl::isFinite(ptXYZ) && ptXYZ.z >= minDepth && ptXYZ.z <= maxDepth)
            {
                pt.x = ptXYZ.x;
                pt.y = ptXYZ.y;
                pt.z = ptXYZ.z;
                if (validIndices)
                {
                    validIndices->at(oi) = (h / decimation)*cloud->width + (w / decimation);
                }
                ++oi;
            }
            else
            {
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    if(validIndices)
    {
        validIndices->resize(oi);
    }
    ROS_WARN("oi count:%d", oi);
    return cloud;
}
/*
04/11/2018
*/
void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & pCloud, const geometry_msgs::Pose & msg)
{
    Eigen::Affine3d tfPose;
    tf::poseMsgToEigen(msg, tfPose);
    tf::Transform tfTransform;
    tf::transformEigenToTF(tfPose, tfTransform);
    pcl_ros::transformPointCloud((const pcl::PointCloud<pcl::PointXYZ>)*pCloud, *pCloud, tfTransform);
}
/*
04/11/2018
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
        const cv::Mat & imageDepthIn,
        float depthFx, float depthCx, float depthFy, float depthCy,
        int decimation,
        float maxDepth,
        float minDepth,
        std::vector<int> * validIndices)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cv::Mat imageDepth = imageDepthIn;
    cloud->height = imageDepth.rows/decimation;
    cloud->width  = imageDepth.cols/decimation;
    cloud->is_dense = false;
    cloud->resize(cloud->height * cloud->width);
    if(validIndices)
    {
        validIndices->resize(cloud->size());
    }
    int oi = 0;
    for(int h = 0; h < imageDepth.rows && h/decimation < (int)cloud->height; h+=decimation)
    {
        for(int w = 0; w < imageDepth.cols && w/decimation < (int)cloud->width; w+=decimation)
        {
            pcl::PointXYZ & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

            pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w, h, depthCx, depthCy, depthFx, depthFy, false);
            if (pcl::isFinite(ptXYZ) && ptXYZ.z >= minDepth && ptXYZ.z <= maxDepth)
            {
                pt.x = ptXYZ.x;
                pt.y = ptXYZ.y;
                pt.z = ptXYZ.z;
                if (validIndices)
                {
                    validIndices->at(oi) = (h / decimation)*cloud->width + (w / decimation);
                }
                ++oi;
            }
            else
            {
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }
    if(validIndices)
    {
        validIndices->resize(oi);
    }
    return cloud;
}
/*
04/11/2018
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr removeNaNFromPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *output, indices);
    return output;
}
/*
04/11/2018
*/
pcl::IndicesPtr radiusFiltering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        float radiusSearch,
        int minNeighborsInRadius)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>(false));

    if(indices->size())
    {
        pcl::IndicesPtr output(new std::vector<int>(indices->size()));
        int oi = 0; // output iterator
        tree->setInputCloud(cloud, indices);
        for(unsigned int i=0; i<indices->size(); ++i)
        {
            std::vector<int> kIndices;
            std::vector<float> kDistances;
            int k = tree->radiusSearch(cloud->at(indices->at(i)), radiusSearch, kIndices, kDistances);
            if(k > minNeighborsInRadius)
            {
                output->at(oi++) = indices->at(i);
            }
        }
        output->resize(oi);
        return output;
    }
    else
    {
        pcl::IndicesPtr output(new std::vector<int>(cloud->size()));
        int oi = 0; // output iterator
        tree->setInputCloud(cloud);
        for(unsigned int i=0; i<cloud->size(); ++i)
        {
            std::vector<int> kIndices;
            std::vector<float> kDistances;
            int k = tree->radiusSearch(cloud->at(i), radiusSearch, kIndices, kDistances);
            if(k > minNeighborsInRadius)
            {
                output->at(oi++) = i;
            }
        }
        output->resize(oi);
        return output;
    }
}
/*
04/11/2018
*/
pcl::IndicesPtr radiusFiltering(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
        float radiusSearch,
        int minNeighborsInRadius)
{
    pcl::IndicesPtr indices(new std::vector<int>);
    return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
}
/*
04/11/2018
*/
void filteringCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & pclCloud, pcl::IndicesPtr & indices, float voxelSize,
                    float noiseFilterRadius, int noiseFilterMinNeighbors)
{
    if(indices->size() && voxelSize > 0.0)
    {
        pclCloud = voxelize(pclCloud, indices, voxelSize);
    }
    // Do radius filtering after voxel filtering ( a lot faster)
    if(pclCloud->size() && noiseFilterRadius > 0.0 && noiseFilterMinNeighbors > 0)
    {
        if(pclCloud->is_dense)
        {
            indices = radiusFiltering(pclCloud, noiseFilterRadius, noiseFilterMinNeighbors);
        }
        else
        {
            indices = radiusFiltering(pclCloud, indices, noiseFilterRadius, noiseFilterMinNeighbors);
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*pclCloud, *indices, *tmp);
        pclCloud = tmp;
        pclCloud = removeNaNFromPointCloud(pclCloud);
    }
}
/*
04/11/2018
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr voxelize(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        float voxelSize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    filter.setInputCloud(cloud);
    if(indices->size())
    {
        filter.setIndices(indices);
    }
    filter.filter(*output);
    return output;
}
/*
created at 12/16/2017
*/
void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pCloud, const geometry_msgs::Pose & msg)
{
    Eigen::Affine3d tfPose;
    tf::poseMsgToEigen(msg, tfPose);
    tf::Transform tfTransform;
    tf::transformEigenToTF(tfPose, tfTransform);
    pcl_ros::transformPointCloud((const pcl::PointCloud<pcl::PointXYZRGB>)*pCloud, *pCloud, tfTransform);
}
/*
created at 12/19/2017
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelize(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        float voxelSize)
{
    //UASSERT(voxelSize > 0.0f);
    //UASSERT_MSG((cloud->is_dense && cloud->size()) || (!cloud->is_dense && indices->size()),
            //uFormat("Cloud size=%d indices=%d is_dense=%s", (int)cloud->size(), (int)indices->size(), cloud->is_dense?"true":"false").c_str());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> filter;
    filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    filter.setInputCloud(cloud);
    if(indices->size())
    {
        filter.setIndices(indices);
    }
    filter.filter(*output);
    return output;
}
/*
created at 12/19/2017
*/
pcl::IndicesPtr radiusFiltering(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        float radiusSearch,
        int minNeighborsInRadius)
{
    pcl::IndicesPtr indices(new std::vector<int>);
    return radiusFiltering(cloud, indices, radiusSearch, minNeighborsInRadius);
}
/*
created at 12/19/2017
*/
pcl::IndicesPtr radiusFiltering(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
        const pcl::IndicesPtr & indices,
        float radiusSearch,
        int minNeighborsInRadius)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>(false));

    if(indices->size())
    {
        pcl::IndicesPtr output(new std::vector<int>(indices->size()));
        int oi = 0; // output iterator
        tree->setInputCloud(cloud, indices);
        for(unsigned int i=0; i<indices->size(); ++i)
        {
            std::vector<int> kIndices;
            std::vector<float> kDistances;
            int k = tree->radiusSearch(cloud->at(indices->at(i)), radiusSearch, kIndices, kDistances);
            if(k > minNeighborsInRadius)
            {
                output->at(oi++) = indices->at(i);
            }
        }
        output->resize(oi);
        return output;
    }
    else
    {
        pcl::IndicesPtr output(new std::vector<int>(cloud->size()));
        int oi = 0; // output iterator
        tree->setInputCloud(cloud);
        for(unsigned int i=0; i<cloud->size(); ++i)
        {
            std::vector<int> kIndices;
            std::vector<float> kDistances;
            int k = tree->radiusSearch(cloud->at(i), radiusSearch, kIndices, kDistances);
            if(k > minNeighborsInRadius)
            {
                output->at(oi++) = i;
            }
        }
        output->resize(oi);
        return output;
    }
}
/*
created at 12/19/2017
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeNaNFromPointCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *output, indices);
    return output;
}
/*
created at 01/16/2018
*/
void appendMesh(
        pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudA,
        std::vector<pcl::Vertices> & polygonsA,
        pcl::PointCloud<pcl::PointXYZRGBNormal> & cloudB,
        std::vector<pcl::Vertices> & polygonsB)
{
    /*UWARN("appendMesh00");
    UDEBUG("cloudA=%d polygonsA=%d cloudB=%d polygonsB=%d", (int)cloudA.size(), (int)polygonsA.size(), (int)cloudB.size(), (int)polygonsB.size());
    UASSERT(!cloudA.isOrganized() && !cloudB.isOrganized());*/

    int sizeA = (int)cloudA.size();
    cloudA += cloudB;

    int sizePolygonsA = (int)polygonsA.size();
    polygonsA.resize(sizePolygonsA+polygonsB.size());

    for(unsigned int i=0; i<polygonsB.size(); ++i)
    {
        pcl::Vertices vertices = polygonsB[i];
        for(unsigned int j=0; j<vertices.vertices.size(); ++j)
        {
            vertices.vertices[j] += sizeA;
        }
        polygonsA[i+sizePolygonsA] = vertices;
    }
}
/*
created at 01/17/2018
*/
void getCenterOfCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pCloud, Eigen::Vector3f &vCenter)
{
    vCenter[0] = vCenter[1] = vCenter[2] = 0.0f;
    for(int i=0; i < pCloud->points.size(); ++i)
    {
       vCenter[0] = (i*vCenter[0] + pCloud->points[i].x)/(i+1);
       vCenter[1] = (i*vCenter[1] + pCloud->points[i].y)/(i+1);
       vCenter[2] = (i*vCenter[2] + pCloud->points[i].z)/(i+1);
    }
}
/*
created at 01/20/2018
*/
float distanceToPlane(Eigen::Vector3f &p, float a, float b, float c, float d)
{
    Eigen::Vector3f vNormal(a,b,c);
    d = d/vNormal.norm();
    vNormal.normalize();
    float distance = vNormal.dot(p) + d;
    distance = fabs(distance);
    return distance;
}
/*
created at 01/20/2018
*/
void transformInversePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pCloud, const geometry_msgs::Pose & msg)
{
    Eigen::Affine3d tfPose;
    tf::poseMsgToEigen(msg, tfPose);
    tf::Transform tfTransform;
    tf::transformEigenToTF(tfPose, tfTransform);
    pcl_ros::transformPointCloud((const pcl::PointCloud<pcl::PointXYZRGB>)*pCloud, *pCloud, tfTransform.inverse());
}
/*
created at 01/29/2018
*/
bool getPointUVCoordinates(pcl::PointXYZ &pt, float fx, float fy, float cx, float cy,
                           int width, int height,
                           pcl::PointXY &uv_coordinates)
{
    uv_coordinates.x = fx*pt.x/pt.z +cx;
    uv_coordinates.y = fy*pt.x/pt.z +cy;
    if(uv_coordinates.x >= 0.0f && uv_coordinates.x < width && uv_coordinates.y >= 0.0f && uv_coordinates.y < height)
        return true;//point was visible by the camera
    return false;
}
/*
01/29/2018
*/
Quaterniond QuaternionRot(Vector3d x1, Vector3d y1, Vector3d z1,
                          Vector3d x2, Vector3d y2, Vector3d z2) {

    Matrix3d M = x1*x2.transpose() + y1*y2.transpose() + z1*z2.transpose();

    Matrix4d N;
    N << M(0,0)+M(1,1)+M(2,2)   ,M(1,2)-M(2,1)          , M(2,0)-M(0,2)         , M(0,1)-M(1,0),
         M(1,2)-M(2,1)          ,M(0,0)-M(1,1)-M(2,2)   , M(0,1)+M(1,0)         , M(2,0)+M(0,2),
         M(2,0)-M(0,2)          ,M(0,1)+M(1,0)          ,-M(0,0)+M(1,1)-M(2,2)  , M(1,2)+M(2,1),
         M(0,1)-M(1,0)          ,M(2,0)+M(0,2)          , M(1,2)+M(2,1)         ,-M(0,0)-M(1,1)+M(2,2);

    EigenSolver<Matrix4d> N_es(N);
    Vector4d::Index maxIndex;
    N_es.eigenvalues().real().maxCoeff(&maxIndex);

    Vector4d ev_max = N_es.eigenvectors().col(maxIndex).real();

    Quaterniond quat(ev_max(0), ev_max(1), ev_max(2), ev_max(3));
    quat.normalize();

    return quat;
}
/*
created at 02/01/2018
*/
void computeTextureCoordsOfQuad(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud,
                                float fx, float fy, float cx, float cy,
                                int width, int height,
                                Eigen::Vector2f &uv_min, Eigen::Vector2f &uv_max)
{
    uv_min[0] = FLT_MAX;
    uv_min[1] = FLT_MAX;
    uv_max[0] = FLT_MIN;
    uv_max[1] = FLT_MIN;
    for(int i=0; i < plane_cloud->points.size(); ++i)
    {
        pcl::PointXYZ pt;
        pt.x = plane_cloud->points[i].x;
        pt.y = plane_cloud->points[i].y;
        pt.z = plane_cloud->points[i].z;
        pcl::PointXY uv_coord;
        if(getPointUVCoordinates(pt, fx, fy, cx, cy, width, height, uv_coord))
        {
            if(uv_min[0] > uv_coord.x)
                uv_min[0] = uv_coord.x;
            if(uv_min[1] > uv_coord.y)
                uv_min[1] = uv_coord.y;
            if(uv_max[0] < uv_coord.x)
                uv_max[0] = uv_coord.x;
            if(uv_max[1] < uv_coord.y)
                uv_max[1] = uv_coord.y;
        }
    }
}
/*
created at 02/09/2018
*/
float distanceFromPointToPlane(Eigen::Vector3f &pt, Eigen::Vector3f* vertices, Eigen::Vector3f &vNormal)
{
    Eigen::Hyperplane<float,3> plane_ok = Eigen::Hyperplane<float,3>::Through( vertices[0], vertices[1], vertices[2] );
    vNormal[0] = plane_ok.coeffs()[0];
    vNormal[1] = plane_ok.coeffs()[1];
    vNormal[2] = plane_ok.coeffs()[2];
    //vNormal(plane_ok.coeffs()[0], plane_ok.coeffs()[1], plane_ok.coeffs()[2]);
    //float d = plane_ok.coeffs()[3]/vNormal.norm();
    //vNormal.normalize();
    //float dist = vNormal.dot(vCenter) + d;
    float norm = vNormal.norm();
    float dist = (vNormal[0]*pt[0] + vNormal[1]*pt[1] + vNormal[2]*pt[2] + plane_ok.coeffs()[3])/norm;
    vNormal.normalize();
    return dist;
}
/*
created at 03/16/2018
*/
void transformInverseVertices(std::vector<Eigen::Vector3f> &vertices, const geometry_msgs::Pose & pose)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for(int i=0; i < vertices.size(); ++i)
    {
        pcl::PointXYZRGB p;
        p.x = vertices[i][0];
        p.y = vertices[i][1];
        p.z = vertices[i][2];
        cloud->points.push_back(p);
    }
    transformInversePointCloud(cloud, pose);
    vertices.clear();
    for(int i=0; i < cloud->points.size(); ++i)
    {
        vertices.push_back(cloud->points[i].getVector3fMap());
    }
}
/*04/06/2018*/
bool getIntersectionOfQuad(Eigen::Vector3f &p1, Eigen::Vector3f &p2,
                           std::vector<Eigen::Vector3f> &vertices, Eigen::Vector3f &interPos)
{
    ParametrizedLine<float,3> pline = ParametrizedLine<float,3>::Through(p1,p2);
    Eigen::Hyperplane<float,3> focalPlane = Eigen::Hyperplane<float,3>::Through(vertices[0], vertices[1], vertices[2]);
    double intersection = pline.intersection(focalPlane);
    interPos = intersection*((p2-p1).normalized()) + p1;
    if(fabs(intersection) > (p2 - p1).norm())
        return false;
    std::vector<Eigen::Vector3f> vts;
    vts.resize(5);
    for(int i=0; i < 4; ++i)
    {
        vts[i] = vertices[i];
    }
    vts[4] = vertices[0];
    for(int i=0; i < 4; ++i)
    {
        if((interPos-vts[i]).dot((vts[i+1]-vts[i])) < 0.0f)
            return false;
    }
    return true;
}

 float getIntersectionOfPlane(Eigen::Vector3f &p1, Eigen::Vector3f &p2, float* coefficients, Eigen::Vector3f &interPos)
 {
     ParametrizedLine<float,3> pline = ParametrizedLine<float,3>::Through(p1,p2);
     Eigen::Hyperplane<float,3> plane;
     plane.coeffs()[0] = coefficients[0];
     plane.coeffs()[1] = coefficients[1];
     plane.coeffs()[2] = coefficients[2];
     plane.coeffs()[3] = coefficients[3];
     double intersection = pline.intersection( plane);
     interPos = intersection*((p2-p1).normalized()) + p1;
     return intersection;
 }

 /*void getOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, std::vector<Eigen::Vector3f> &vertices, Eigen::Vector3f &vHorizen)
 {
     pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
     feature_extractor.setInputCloud (cloud_xyz);
     feature_extractor.compute ();
     pcl::PointXYZ min_point_OBB;
     pcl::PointXYZ max_point_OBB;
     pcl::PointXYZ position_OBB;
     Eigen::Matrix3f rotational_matrix_OBB;
     feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

     Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
     Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
     Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
     Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
     Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
     Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
     Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
     Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

     Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
     p1 = rotational_matrix_OBB * p1 + position;
     p2 = rotational_matrix_OBB * p2 + position;
     p3 = rotational_matrix_OBB * p3 + position;
     p4 = rotational_matrix_OBB * p4 + position;
     p5 = rotational_matrix_OBB * p5 + position;
     p6 = rotational_matrix_OBB * p6 + position;
     p7 = rotational_matrix_OBB * p7 + position;
     p8 = rotational_matrix_OBB * p8 + position;

     /*pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
     pCloudOBB->points.push_back(pt1);
     pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
     pCloudOBB->points.push_back(pt2);
     pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
     pCloudOBB->points.push_back(pt3);
     pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
     pCloudOBB->points.push_back(pt4);
     pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
     pCloudOBB->points.push_back(pt5);
     pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
     pCloudOBB->points.push_back(pt6);
     pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
     pCloudOBB->points.push_back(pt7);
     pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));
     pCloudOBB->points.push_back(pt8);*/

     /*Eigen::Vector3f vecs[3];
     vecs[0] = p2 - p1;
     vecs[1] = p4 - p1;
     vecs[2] = p5 - p1;
     bool bLoop = true;
     while(bLoop)
     {
         bLoop = false;
         for(int i=0; i < 2; ++i)
         {
             if(vecs[i].norm() > vecs[i+1].norm())
             {
                 Eigen::Vector3f v = vecs[i];
                 vecs[i] = vecs[i+1];
                 vecs[i+1] = v;
                 bLoop = true;
             }
         }
     }
     //vertices.resize(8);
     vertices[0] = p1;
     vertices[1] = vertices[0] + vecs[1];
     vertices[2] = vertices[1] + vecs[2];
     vertices[3] = vertices[0] + vecs[2];

     vertices[4] = vertices[0] + vecs[0];
     vertices[5] = vertices[1] + vecs[0];
     vertices[6] = vertices[2] + vecs[0];
     vertices[7] = vertices[3] + vecs[0];
     vHorizen = vecs[0];
  }*/

 void downSampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_input,
                   float leaf_size,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
 {
     // Create the filtering object: downsample the dataset using a leaf size of 1cm
     pcl::VoxelGrid<pcl::PointXYZRGB> sor;
     sor.setInputCloud (cloud_input);
     sor.setLeafSize (leaf_size, leaf_size, leaf_size);
     sor.filter (*cloud_out);
 }

 void computingBoundingBox(std::vector<Eigen::Vector3f> &vertices_input,
                           Eigen::Vector3f &vDir,
                           std::vector<Eigen::Vector3f> &vertices)
 {
     Eigen::Vector3f mass_center(0.0f, 0.0f, 0.0f);
     for(int i=0; i < vertices_input.size(); ++i)
     {
        mass_center = (i*mass_center + vertices_input[i])/(i+1);
     }
     vDir.normalize();
     Eigen::Vector3f vUp(0.0f, 0.0f, 1.0f);
     Eigen::Vector3f vDown(0.0f, 0.0f, -1.0f);
     Eigen::Vector3f vForward = vDir;
     Eigen::Vector3f vBackward = -vForward;
     Eigen::Vector3f vLeft = vForward.cross(vUp);
     Eigen::Vector3f vRight = -vLeft;
     vLeft.normalize();
     vRight.normalize();
     Eigen::Vector3f vertex, vec;
     float dist_up, dist_down, dist_forward, dist_backward, dist_left, dist_right;
     dist_up = dist_down = dist_forward = dist_backward = dist_left = dist_right = FLT_MIN;
     for(int i=0; i < vertices_input.size(); ++i)
     {
       vec = vertices_input[i] - mass_center;
       dist_forward = MAX(vec.dot(vForward), dist_forward);
       dist_backward = MAX(vec.dot(vBackward), dist_backward);
       dist_up = MAX(vec.dot(vUp), dist_up);
       dist_down = MAX(vec.dot(vDown), dist_down);
       dist_left = MAX(vec.dot(vLeft), dist_left);
       dist_right = MAX(vec.dot(vRight), dist_right);
     }
     vertices.resize(8);
     vertices[0] = mass_center + dist_left * vLeft + dist_down * vDown + dist_forward * vForward;
     vertices[1] = mass_center + dist_right * vRight + dist_down * vDown + dist_forward * vForward;
     vertices[2] = mass_center + dist_right * vRight + dist_up * vUp + dist_forward * vForward;
     vertices[3] = mass_center + dist_left * vLeft + dist_up * vUp + dist_forward * vForward;

     vertices[4] = mass_center + dist_left * vLeft + dist_down * vDown + dist_backward * vBackward;
     vertices[5] = mass_center + dist_right * vRight + dist_down * vDown + dist_backward * vBackward;
     vertices[6] = mass_center + dist_right * vRight + dist_up * vUp + dist_backward * vBackward;
     vertices[7] = mass_center + dist_left * vLeft + dist_up * vUp + dist_backward * vBackward;
 }

 bool intersectPlane(Eigen::Vector3f &vOrigin, Eigen::Vector3f &vDir, Eigen::Vector4f &planeCoeffs)
 {
     ParametrizedLine<float,3> pline = ParametrizedLine<float,3>::Through(vOrigin, vDir);
     Eigen::Hyperplane<float,3> plane;
     plane.coeffs()[0] = planeCoeffs[0];
     plane.coeffs()[1] = planeCoeffs[1];
     plane.coeffs()[2] = planeCoeffs[2];
     plane.coeffs()[3] = planeCoeffs[3];
     Eigen::Vector3f interPos = pline.intersectionPoint(plane);
     Eigen::Vector3f vec = interPos - vOrigin;
     if(vDir.dot(vec) > 0.0f)
         return true;
     else
         return false;
 }

 float distanceFromPointToLine(Eigen::Vector3f &vOrigin, Eigen::Vector3f &vTarg, Eigen::Vector3f &p)
 {
     ParametrizedLine<float,3> pline = ParametrizedLine<float,3>::Through(vOrigin, vTarg);
     return fabs(pline.distance(p));
 }

 float getTop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud)
 {
     float z = FLT_MIN;
     for(int i=0; i < pCloud->points.size(); ++i)
     {
         z = MAX(z,pCloud->points[i].z);
     }
     return z;
 }

 float getBottom(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud)
 {
     float z = FLT_MAX;
     for(int i=0; i < pCloud->points.size(); ++i)
     {
         z = MIN(z,pCloud->points[i].z);
     }
     return z;
 }

 void filteringDepthMap(cv::Mat &depthMat)
 {
     for(int y=0; y < depthMat.rows; ++y)
     {
         for(int x=0; x < depthMat.cols; ++x)
         {
             //if(depthMat.at<unsigned short>(y,x) > 255)
             if(depthMat.at<unsigned short>(y,x) == std::numeric_limits<unsigned short>::max())
             {
                 depthMat.at<unsigned short>(y,x) = 0;
             }
         }
     }
 }

 void fillHoles(cv::Mat &depthMat, int minDepth, int maxSize)
{
    cv::Mat tmp = depthMat.clone();
    int width = tmp.cols;
    int height = tmp.rows;
    for(int y=1; y < height-1; ++y)
    {
        for(int x=1; x < width-1; ++x)
        {
            if(tmp.at<unsigned short>(y,x)>minDepth)
                continue;
            //extract black holes
            std::vector<cv::Point> black_points;
            std::vector<cv::Point> buf;
            cv::Point p;
            p.x = x;
            p.y = y;
            black_points.push_back(p);
            buf.push_back(p);
            while(!black_points.empty())
            {
                int s = black_points.size();
                p = black_points[s-1];
                black_points.pop_back();
                cv::Point pts[4];
                for(int i=0; i < 4; ++i)
                {
                   pts[i] = p;
                }
                pts[0].x += 1;
                pts[1].y += 1;
                pts[2].x -= 1;
                pts[3].y -= 1;
                for(int i=0; i < 4; ++i)
                {
                    if(pts[i].x >= 0 && pts[i].x < width)
                    {
                        if(pts[i].y >= 0 && pts[i].y < height)
                        {
                            if(tmp.at<unsigned short>(pts[i].y, pts[i].x)<=minDepth)
                            {
                                black_points.push_back(pts[i]);
                                tmp.at<unsigned short>(pts[i].y, pts[i].x) = minDepth + 1;
                                buf.push_back(pts[i]);
                            }
                        }
                    }
                }
            }
            if(buf.size() > maxSize)
                continue;
            //filling holes
            while(!buf.empty())
            {
                std::vector<cv::Point> boundary;
                std::vector<cv::Point> edge;
                for(int i=0; i < buf.size(); ++i)
                {
                    cv::Point pts[4];
                    for(int j=0; j < 4; ++j)
                    {
                        pts[j] = buf[i];
                    }
                    pts[0].x++;
                    pts[1].y--;
                    pts[2].x--;
                    pts[3].y++;
                    for(int j=0; j < 4; ++j)
                    {
                        if(depthMat.at<unsigned short>(pts[j].y, pts[j].x) > minDepth)
                        {
                            boundary.push_back(buf[i]);
                            edge.push_back(pts[j]);
                            buf.erase(buf.begin()+i);
                            //depthMat.at<unsigned short>(buf[i].y, buf[i].x) = depthMat.at<unsigned short>(pts[j].y, pts[j].x);
                            break;
                        }
                    }
                }
                for(int j=0;j < boundary.size(); ++j)
                {
                    depthMat.at<unsigned short>(boundary[j].y,boundary[j].x) = depthMat.at<unsigned short>(edge[j].y,edge[j].x);
                }
            }
            /*while(!buf.empty())
            {
                for(int i=0; i < buf.size(); ++i)
                {
                   cv::Point points[4];
                   for(int j=0; j < 4; ++j)
                   {
                       points[j] = buf[i];
                   }
                   points[0].x += 1;
                   points[1].y += 1;
                   points[2].x -= 1;
                   points[3].y -= 1;
                   bool flg = false;
                   for(int j=0; j < 4; ++j)
                   {
                       if(depthMat.at<unsigned short>(points[j].y, points[j].x) > minDepth)
                       {
                           depthMat.at<unsigned short>(buf[i].y, buf[i].x) = depthMat.at<unsigned short>(points[j].y, points[j].x);
                           buf.erase(buf.begin() + i);
                           flg = true;
                           break;
                       }
                   }
                }
            }*/
        }
    }
}

 void refineDepthMap(cv::Mat &depthMat, int filterSize, int nloops)
 {
    IplConvKernel *convKernel = cvCreateStructuringElementEx(filterSize, filterSize, (filterSize - 1) / 2, (filterSize - 1) / 2, CV_SHAPE_RECT, 0);
    int width = depthMat.cols;
    int height = depthMat.rows;
    IplImage img1 = depthMat;
    IplImage *img2 = cvCreateImage(cvSize(width,height),16,1);
    IplImage *img3 = cvCreateImage(cvSize(width,height),16,1);
    cvErode(&img1, img2, convKernel, nloops);
    cvDilate(img2, img3, convKernel, nloops);
    depthMat = cv::cvarrToMat(img3);
 }

 void inpaintingDepth(cv::Mat &depthMap)
 {
     const unsigned char noDepth = 0;
     cv::Mat tmp;
     //cv::inpaint(depthMap, (depthMap==noDepth), tmp, 5.0, cv::INPAINT_TELEA);
     //tmp.copyTo(depthMap, (depthMap==noDepth));
     IplImage img;
     //cv::inpaint(img, img, img, 5, 0);
 }

 void blendImages(cv::Mat &targ, cv::Mat &src, cv::Mat &depthMat, int tx, int ty, int tz)
 {
     for(int y=0; y<src.rows; ++y)
     {
         for(int x=0; x < src.cols; ++x)
         {
             if(src.at<cv::Vec3b>(y,x)[1]==0)
                 continue;
             if(depthMat.at<unsigned short>(y+ty,x+tx) < tz && depthMat.at<unsigned short>(y+ty,x+tx) > 0)
                 continue;
             targ.at<cv::Vec3b>(y+ty,x+tx)[0] = src.at<cv::Vec3b>(y,x)[0];
             targ.at<cv::Vec3b>(y+ty,x+tx)[1] = src.at<cv::Vec3b>(y,x)[1];
             targ.at<cv::Vec3b>(y+ty,x+tx)[2] = src.at<cv::Vec3b>(y,x)[2];
         }
     }
 }

 void bilateralFilter(cv::Mat &src, int kernel_length, cv::Mat &dst)
 {
     int width = src.cols;
     int height = src.rows;
     cv::Mat input(cvSize(width,height),CV_32FC1);
     cv::Mat output(cvSize(width,height),CV_32FC1);
     for(int y=0; y < height; ++y)
     {
         for(int x=0; x < width; ++x)
         {
             input.at<float>(y,x) = src.at<unsigned short>(y,x);
         }
     }
     cv::bilateralFilter(input, output, kernel_length, kernel_length*2, kernel_length/2);
     for(int y=0; y < height; ++y)
     {
         for(int x = 0; x < width; ++x)
         {
             dst.at<unsigned short>(y,x) = output.at<float>(y,x);
         }
     }
 }

 void smoothing(cv::Mat &src, int kernel_length, int mode, cv::Mat &dst)
 {
     switch(mode)
     {
     case 0:
         //Homogeneous blur:
         cv::blur(src, dst, cvSize(kernel_length, kernel_length), cvPoint(-1,-1));
         break;
     case 1:
         //Gaussian blur:
         cv::GaussianBlur(src, dst, cvSize(kernel_length, kernel_length), 0, 0);
         break;
     case 2:
         //Median blur:
         cv::medianBlur(src, dst, kernel_length);
         break;
     case 3:
         //Bilateral blur:
         bilateralFilter(src, kernel_length, dst);
         break;
     }
 }

void offset_image(cv::Mat &inputMat, int x_offset, int y_offset, cv::Mat &outMat)
{
    for(int y=y_offset; y < inputMat.rows; ++y)
    {
        for(int x = x_offset; x < inputMat.cols; ++x)
        {
            outMat.at<unsigned short>(y, x) = inputMat.at<unsigned short>(y-y_offset,x-x_offset);
        }
    }
}

#include <ros/ros.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/package.h>
#include <pcl/filters/voxel_grid.h>
double DistanceThreshold;
std::string pcd_model_name;
double lafe_size;

pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input(new pcl::PointCloud<pcl::PointXYZ>());
int outkey = 0;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *scene_input);
    outkey = 1;
}
// 对remaining_cloud进行降采样
void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud, 
                          float leaf_size) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(input_cloud);
    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size); // 设置体素大小
    voxel_grid_filter.filter(*output_cloud); // 执行降采样
    std::cout << "Original point cloud size: " << input_cloud->points.size() 
          << " -> Downsampled point cloud size: " << output_cloud->points.size() << std::endl;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_saver");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/camera/depth/color/points", 1, cloud_cb);

    // 等待点云数据
    ros::Rate loop_rate(10);
    // 参数读取
    node.param<double>("DistanceThreshold", DistanceThreshold, 0.011);
    node.param<std::string>("save_pcd_name", pcd_model_name, "current_model");
    node.param<double>("lafe_size", lafe_size, 0.001);

    while (ros::ok() && outkey == 0) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    // 预处理点云
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*scene_input, *scene_input, indices);

    // 平面分割
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    // 中间平面的厚度
    seg.setDistanceThreshold(DistanceThreshold);
    seg.setInputCloud(scene_input);
    seg.segment(*inliers, *coefficients);

    // 提取非平面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(scene_input);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filtered_cloud);

    // 分割两侧点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloudA(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloudB(new pcl::PointCloud<pcl::PointXYZ>());
    
    for (const auto& point : filtered_cloud->points) {
        float distance = coefficients->values[0] * point.x + 
                        coefficients->values[1] * point.y + 
                        coefficients->values[2] * point.z + 
                        coefficients->values[3];
        if (distance == 0.0) continue;
        if (distance < -0.01) remaining_cloudA->push_back(point);
        else if (distance > 0.01) remaining_cloudB->push_back(point);
    }

    // 选择点数较多的部分
    auto remaining_cloud = (remaining_cloudA->size() > remaining_cloudB->size()) ? 
                          remaining_cloudA : remaining_cloudB;

    // 对点云进行降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud_down(new pcl::PointCloud<pcl::PointXYZ>()); // 初始化指针
    downsamplePointCloud(remaining_cloud,remaining_cloud_down,lafe_size);

    // 保存点云
    std::string save_path = ros::package::getPath("3dposedetection") + "/model_pcd/"+pcd_model_name+".pcd";
    pcl::io::savePCDFileASCII(save_path, *remaining_cloud_down);
    std::cout << "Point cloud saved to: " << save_path << std::endl;

    // 可视化保存的点云
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色为黑色

    // 显示保存的点云（蓝色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> remaining_cloud_color(remaining_cloud_down, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(remaining_cloud_down, remaining_cloud_color, "remaining_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "remaining_cloud");

    // 添加坐标系
    viewer->addCoordinateSystem(0.1); // 添加坐标系，长度为0.1

    // 启动可视化窗口
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    return 0;
}

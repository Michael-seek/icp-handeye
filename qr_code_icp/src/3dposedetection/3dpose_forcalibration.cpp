#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/opencv.hpp>
#include <opencv2/flann/flann.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <cmath>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sys/time.h>
#include <pcl/registration/super4pcs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <gr/utils/shared.h>
#include "demo-utils.h"
#include <ros/package.h>
// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_inputttt(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_input_second(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointclouds(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz_kdtree(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr final_model(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_xyz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr ori_object_xyz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_xyz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_sparse_xyz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_aligned_xyz_icped(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr object_kd_search_xyz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_rgb(new pcl::PointCloud<pcl::PointXYZRGBA>());
pcl::PointCloud<pcl::PointNormal>::Ptr scene_with_normals(new pcl::PointCloud<pcl::PointNormal>());

using namespace gr;
using namespace cv;
using namespace std;

int outkey = 0;

// 点云的回调函数
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *scene_input);
  outkey = 1;
  // pcl::io::savePCDFileASCII("output_pcd.pcd", *scene_input);
}
// 找出最小值
float findmin(vector<float> vec)
{
  float min = 999;
  for (auto v : vec)
  {
    if (min > v)
      min = v;
  }
  return min;
}
// 找出值的索引
int getPositionOfmin(vector<float> vec, float min)
{
  auto distance = find(vec.begin(), vec.end(), min);
  return distance - vec.begin();
}

int main(int argc, char **argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "test_real");

  // 订阅base_link的位置话题
  ros::NodeHandle node;

  pcl::visualization::PCLVisualizer visu("Alignment - Super4PCS");

  // 接受点云的订阅者
  ros::Subscriber sub = node.subscribe("/camera/depth/color/points", 1, cloud_cb);

  ros::Rate loop_rate(100);
  // 等待点云的消息
  while (ros::ok() and outkey == 0)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  // 定义所需tf、距离、标志位
  tf::TransformListener listener;
  tf::StampedTransform transform_aru_tf22, transform_aru_tf44, transform_aru_tf888, transform_aru_tf963, transform_aru_tf;
  float distance22, distance888, distance44, distance963, finaldistance;
  int key22, key888, key44, key963;
  // 四个二维码的tf
  std::vector<tf::StampedTransform> transform_aru_tf_vector;
  // 四个二维码里相机的距离
  std::vector<float> distances;
  // 二维码到物块中心，所需旋转角度的倍数的容器(根据不同的标签，从当前的二维码的tf获取到物块中心的tf，以此作为先验，便于点云icp配准)
  std::vector<int> multiples_vec;
  // 二维码的tf名字
  std::vector<std::string> strVec;
  // 标志为，判断是否监听到tf
  key22 = 0;
  key888 = 0;
  key44 = 0;
  key963 = 0;

  double deltatime;
  // 获取初始时间
  ros::Time start_time = ros::Time::now();
  ROS_INFO("Start time: %f", start_time.toSec());

  // 5s内判断，是否监听到四个二维码的tf
  while (node.ok() and (key22 == 0 or key888 == 0 or key44 == 0 or key963 == 0) and (deltatime < 5))
  {
    if (key22 == 0)
    {
      try
      {
        listener.lookupTransform("/aruco_marker_frame22", "/camera_color_optical_frame",
                                 ros::Time(0), transform_aru_tf22);
        cout << "aruco_marker_frame22 exist" << endl;
        key22 = 1;
        distance22 = transform_aru_tf22.getOrigin().x() * transform_aru_tf22.getOrigin().x() + transform_aru_tf22.getOrigin().y() * transform_aru_tf22.getOrigin().y() + transform_aru_tf22.getOrigin().z() * transform_aru_tf22.getOrigin().z();
        // 传入tf
        transform_aru_tf_vector.push_back(transform_aru_tf22);
        // 传入距离
        distances.push_back(distance22);
        // 传入tf的名字
        strVec.push_back("aruco_marker_frame22");
        // 传入需要的旋转倍数
        multiples_vec.push_back(2);
      }
      catch (tf::TransformException ex)
      {
        cout << "aruco_marker_frame22 don't exist" << endl;
      }
    }
    if (key44 == 0)
    {
      try
      {
        listener.lookupTransform("/aruco_marker_frame44", "/camera_color_optical_frame",
                                 ros::Time(0), transform_aru_tf44);
        cout << "aruco_marker_frame44 exist" << endl;
        key44 = 1;
        distance44 = transform_aru_tf44.getOrigin().x() * transform_aru_tf44.getOrigin().x() + transform_aru_tf44.getOrigin().y() * transform_aru_tf44.getOrigin().y() + transform_aru_tf44.getOrigin().z() * transform_aru_tf44.getOrigin().z();
        transform_aru_tf_vector.push_back(transform_aru_tf44);
        distances.push_back(distance44);
        strVec.push_back("aruco_marker_frame44");
        multiples_vec.push_back(1);
      }
      catch (tf::TransformException ex)
      {
        cout << "aruco_marker_frame44 don't exist" << endl;
      }
    }
    if (key888 == 0)
    {
      try
      {
        listener.lookupTransform("/aruco_marker_frame888", "/camera_color_optical_frame",
                                 ros::Time(0), transform_aru_tf888);
        cout << "aruco_marker_frame888 exist" << endl;
        key888 = 1;
        distance888 = transform_aru_tf888.getOrigin().x() * transform_aru_tf888.getOrigin().x() + transform_aru_tf888.getOrigin().y() * transform_aru_tf888.getOrigin().y() + transform_aru_tf888.getOrigin().z() * transform_aru_tf888.getOrigin().z();
        transform_aru_tf_vector.push_back(transform_aru_tf888);
        distances.push_back(distance888);
        strVec.push_back("aruco_marker_frame888");
        multiples_vec.push_back(0);
      }
      catch (tf::TransformException ex)
      {
        cout << "aruco_marker_frame888 don't exist" << endl;
      }
    }
    if (key963 == 0)
    {
      try
      {
        listener.lookupTransform("/aruco_marker_frame963", "/camera_color_optical_frame",
                                 ros::Time(0), transform_aru_tf963);
        cout << "aruco_marker_frame963 exist" << endl;
        key963 = 1;
        distance963 = transform_aru_tf963.getOrigin().x() * transform_aru_tf963.getOrigin().x() + transform_aru_tf963.getOrigin().y() * transform_aru_tf963.getOrigin().y() + transform_aru_tf963.getOrigin().z() * transform_aru_tf963.getOrigin().z();
        transform_aru_tf_vector.push_back(transform_aru_tf963);
        distances.push_back(distance963);
        strVec.push_back("aruco_marker_frame963");
        multiples_vec.push_back(3);
      }
      catch (tf::TransformException ex)
      {
        cout << "aruco_marker_frame963 don't exist" << endl;
      }
    }
    // 获取结束时间
    ros::Time end_time = ros::Time::now();
    // 计算时间差
    ros::Duration duration = end_time - start_time;
    deltatime = duration.toSec();  // 将时间差转换为秒
    ROS_INFO("delta_time: %f seconds", deltatime);
    ros::spinOnce();
  }
  //找到距离最近的一个点，使用这个点计算出物块中心点处的位置和姿态
  float minNumber = findmin(distances);
  int position = getPositionOfmin(distances, minNumber);
  transform_aru_tf = transform_aru_tf_vector[position];
  int multiples = multiples_vec[position];
  //打印使用的二维码信息
  pcl::console::print_info("size: %d  use: %s  \n", strVec.size(), strVec[position].c_str());
  // pcl::console::print_info("multiples,%d   \n", multiples);

  // 创建tf的广播器
  static tf::TransformBroadcaster br;
  //最终结果的旋转值
  Eigen::Quaternionf q_r_final;
  //最终结果的平移
  float transx, transy, transz;
  //是否展示分割的结果
  int ifshowresult = 0;
  //是否展示最终的匹配结果
  int ifshowfinalresult = 1;
  // 点云的变量
  PointCloudT::Ptr object(new PointCloudT);
  PointCloudT::Ptr object_kd_search(new PointCloudT);
  PointCloudT::Ptr scene(new PointCloudT);
  PointCloudT::Ptr object_aligned(new PointCloudT);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPLY(new pcl::PointCloud<pcl::PointXYZ>);
  //清除无效的点云
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*scene_input, *scene_input, indices);
  //赋值
  scene_input_second = scene_input;
  // std::cout << "4..." << std::endl;
  //定义分割点云等变量
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloudA(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloudB(new pcl::PointCloud<pcl::PointXYZ>());
  // 创建一个pcl::ModelCoefficients类型的智能指针，用于存储模型的系数。
  // pcl::ModelCoefficients是一个结构体，用于存储拟合模型的参数，例如平面方程的系数。
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // 创建一个pcl::PointIndices类型的智能指针，用于存储内点的索引。
  // pcl::PointIndices是一个结构体，包含一个点索引的vector，这些点被用来拟合模型。
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // 声明一个pcl::SACSegmentation类的实例，专门用于处理pcl::PointXYZ类型的点云数据。
  // pcl::SACSegmentation是PCL中用于分割点云的类，它实现了基于随机采样一致性（RANSAC）的模型拟合和点云分割。
  // pcl::PointXYZ是一个结构体，表示一个包含x、y、z坐标的3D点。
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // 设置SACSegmentation对象seg的优化系数标志为true。
  // 当这个标志被设置为true时，SACSegmentation算法在找到模型的初始拟合后，
  // 会尝试进一步优化模型的系数，以获得更准确的拟合结果。
  seg.setOptimizeCoefficients(true);
  // 设置SACSegmentation对象seg的模型类型为平面（SACMODEL_PLANE）。
  // 这意味着seg将尝试在点云中找到一个最佳的平面模型。
  seg.setModelType(pcl::SACMODEL_PLANE);
  // 设置SACSegmentation对象seg的方法类型为RANSAC（SAC_RANSAC）。
  // RANSAC是一种鲁棒的模型拟合方法，用于从包含异常值的数据中估计模型参数。
  seg.setMethodType(pcl::SAC_RANSAC);
  // 设置距离的阈值，多少范围的点会被认为是分割模型内的点，就是标定块中中间平面的厚度，这个参数可能需要手动调整获取最佳的效果
  seg.setDistanceThreshold(0.011);
  // 执行点云分割操作，并将结果存储在inliers和coefficients智能指针指向的对象中。
  // inliers将包含分割后模型的内点索引，coefficients将包含模型的系数。  
  seg.setInputCloud(scene_input);
  seg.segment(*inliers, *coefficients);
  // 提出分割后平面外的点，并将结果存储在filtered_cloud中。setNegative设为true是平面外的点，false则相反。
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(scene_input_second);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*filtered_cloud);
  //打印分割后点的数量
  pcl::console::print_info("filtered_cloud size, %d  \n", filtered_cloud->points.size());
  //展示分割后的点云
  if (ifshowresult)
  {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());

    viewer->addPointCloud<pcl::PointXYZ>(scene_input_second, "samples cloudori2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "samples cloudori2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "samples cloudori2");

    viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud, "samples cloudori");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples cloudori");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "samples cloudori");

    viewer->addCoordinateSystem(0.1);
    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  // 根据点云到平面的距离，分割平面两册的点云，把多数点云设为标点块的点云
  int countA = 0;
  int countB = 0;
  for (size_t i = 0; i < filtered_cloud->points.size(); ++i)
  {
    // 跳过原点（0,0,0），因为原点到任何平面的距离都是0。
    if (filtered_cloud->points[i].x * filtered_cloud->points[i].x + filtered_cloud->points[i].y * filtered_cloud->points[i].y + filtered_cloud->points[i].z * filtered_cloud->points[i].z == 0.0)
      continue;
    // 计算点到平面的距离，如果距离小于0（即点在平面的负半空间），则添加到remaining_cloudA。
    if (coefficients->values[0] * filtered_cloud->points[i].x + coefficients->values[1] * filtered_cloud->points[i].y + coefficients->values[2] * filtered_cloud->points[i].z + coefficients->values[3] < 0)
      remaining_cloudA->points.push_back(filtered_cloud->points[i]);
    // 如果距离大于0（即点在平面的正半空间），则添加到remaining_cloudB。
    else if (coefficients->values[0] * filtered_cloud->points[i].x + coefficients->values[1] * filtered_cloud->points[i].y + coefficients->values[2] * filtered_cloud->points[i].z + coefficients->values[3] > 0)
      remaining_cloudB->points.push_back(filtered_cloud->points[i]);
    // 统计距离平面一定距离（大于0.01单位）的点的数量。
    if (coefficients->values[0] * filtered_cloud->points[i].x + coefficients->values[1] * filtered_cloud->points[i].y + coefficients->values[2] * filtered_cloud->points[i].z + coefficients->values[3] < -0.01)
      countA = countA + 1;
    else if (coefficients->values[0] * filtered_cloud->points[i].x + coefficients->values[1] * filtered_cloud->points[i].y + coefficients->values[2] * filtered_cloud->points[i].z + coefficients->values[3] > 0.01)
      countB = countB + 1;
  }
  // 根据统计结果，选择点数较多的那部分作为最终的remaining_cloud。
  if (countA > countB)
    remaining_cloud = remaining_cloudA;
  else
    remaining_cloud = remaining_cloudB;
  // 展示结果
  if (ifshowresult)
  {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->removeAllPointClouds();
    viewer->addPointCloud<pcl::PointXYZ>(scene_input_second, "samples cloudori2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "samples cloudori2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "samples cloudori2");

    viewer->addPointCloud<pcl::PointXYZ>(remaining_cloud, "samples cloudori");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "samples cloudori");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "samples cloudori");

    while (!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  // 设置点云属性并添加到PCL可视化对象中，指定点云颜色为白色。
  remaining_cloud->width = 1;
  remaining_cloud->height = remaining_cloud->points.size();
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloud_color_object(remaining_cloud, 255.0, 255.0, 255.0);
  //添加环境点云到可视化工具中
  visu.addPointCloud<pcl::PointXYZ>(remaining_cloud, PointCloud_color_object, "object_aligned_sparse_xyz");

  // 获取包的路径
  string package_path = ros::package::getPath("3dposedetection");
  // 读取文件中的标定块点云
  string sr = package_path +"/model_pcd/test_model.pcd";

  pcl::console::print_highlight("Loading model point clouds...\n");
  if (pcl::io::loadPCDFile<PointNT>(sr, *object) < 0)
  {
    pcl::console::print_error("Error loading object/scene file!\n");
    return (-1);
  }
  pcl::console::print_highlight("Loading model success...\n");
  if (1)
  {
    // Print results
    printf("\n");

    //******************************************************ICP algorithms*********************************************************//
    // 设置ICP算法的最大迭代次数为500。
    int iterations = 500;
    // 创建一个ICP算法的实例，指定点云的类型为pcl::PointXYZ。
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // 复制object点云到ori_object_xyz点云，以便在ICP算法中使用原始点云。
    pcl::copyPointCloud(*object, *ori_object_xyz);
    // 创建一个VoxelGrid滤波器的实例，用于下采样点云。
    pcl::VoxelGrid<pcl::PointXYZ> grid_search;
    // 设置VoxelGrid滤波器的叶尺寸为0.01f，这决定了点云采样的精细度。
    const float leaf_kd_search = 0.01f;
    grid_search.setLeafSize(leaf_kd_search, leaf_kd_search, leaf_kd_search);
    // 设置VoxelGrid滤波器的输入点云为ori_object_xyz。
    grid_search.setInputCloud(ori_object_xyz);
    // 使用VoxelGrid滤波器对ori_object_xyz点云进行下采样，并将结果存储在object_aligned_sparse_xyz点云中。
    grid_search.filter(*object_aligned_sparse_xyz);

    // ###############################################################
    // 如果要看过滤过的点云，则取消注释下面的部分
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloud_color_object(object_aligned_xyz, 255.0, 255.0, 255.0);
    // visu.addPointCloud<pcl::PointXYZ>(object_aligned_xyz, PointCloud_color_object, "object_aligned_sparse_xyz");
    // ###############################################################

    //以下部分进行旋转平移，利用二维码获取了标定块底部的中心的位置，记录在transform_aru中，用于点云的配准的先验

    Eigen::Matrix4f transform_rotation_humanpredefined = Eigen::Matrix4f::Identity(); // 这个应该是二维码相对于物体坐标系的变换,先转动
    Eigen::AngleAxisf rollAngle(0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(0, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(0.78539816339745, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3f mat3_humanpredefined = q.matrix();
    transform_rotation_humanpredefined.block(0, 0, 3, 3) = mat3_humanpredefined;

    Eigen::Matrix4f transform_trans_humanpredefined = Eigen::Matrix4f::Identity();
    transform_trans_humanpredefined(0, 3) = -0.065; // 应该是负的，这个应该是二维码相对于物体坐标系的变换,再移动
    transform_trans_humanpredefined(1, 3) = 0.02;

    Eigen::Matrix4f transform_rotation_calibration_final = Eigen::Matrix4f::Identity(); // 就不同二维码（贴了四周，各四个），需要再进行一次绕y的旋转
    Eigen::AngleAxisf rollAngle_final(0, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle_final(1.5707963267949 * multiples, Eigen::Vector3f::UnitY()); // 1.5707963267949*multiples
    Eigen::AngleAxisf yawAngle_final(0, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q_final = yawAngle_final * pitchAngle_final * rollAngle_final;
    Eigen::Matrix3f mat3_humanpredefined_final = q_final.matrix();
    transform_rotation_calibration_final.block(0, 0, 3, 3) = mat3_humanpredefined_final;

    Eigen::Matrix3f mat3 = Eigen::Quaternionf(transform_aru_tf.getRotation().w(), transform_aru_tf.getRotation().x(), transform_aru_tf.getRotation().y(), transform_aru_tf.getRotation().z()).toRotationMatrix();
    Eigen::Matrix4f transform_aru = Eigen::Matrix4f::Identity();

    transform_aru.block(0, 0, 3, 3) = mat3;
    transform_aru(0, 3) = transform_aru_tf.getOrigin().x();
    transform_aru(1, 3) = transform_aru_tf.getOrigin().y();
    transform_aru(2, 3) = transform_aru_tf.getOrigin().z();
    transform_aru = transform_rotation_calibration_final * transform_trans_humanpredefined * transform_rotation_humanpredefined * transform_aru;

    // 计算之前得到的变换矩阵transform_aru的逆矩阵。
    Eigen::Matrix4f transform_aru_inv = transform_aru.inverse();
    // 应用变换矩阵把读取文件的点云ori_object_xyz进行变换，得到点云object_xyz。
    pcl::transformPointCloud(*ori_object_xyz, *object_xyz, transform_aru_inv);
    // 创建一个仿射变换对象listeningresult，用于在可视化中表示变换。
    Eigen::Affine3f listeningresult(transform_aru_inv);
    // 在PCLVisualizer对象visu中添加一个坐标系，以展示listeningresult表示的变换。
    visu.addCoordinateSystem(0.2, listeningresult);

    // pcl::io::savePCDFileASCII ("/home/michael/calibration_ws/src/icp-handeye/qr_code_icp/src/3dposedetection/test_model.pcd", *remaining_cloud);

    //设置最大迭代次数
    icp.setMaximumIterations(iterations);
    // 设置ICP算法的输入源点云为remaining_cloud，这通常是环境中的点云。
    icp.setInputSource(remaining_cloud);
    // 设置ICP算法的目标点云为object_xyz，这是已知模型或参考点云。
    icp.setInputTarget(object_xyz);
    pcl::console::print_highlight("Start Aligning ...\n");
    // 执行ICP算法，将remaining_cloud与object_xyz进行对齐，结果存储在remaining_cloud中。
    icp.align(*remaining_cloud); // 理论上应该是环境点云
    pcl::console::print_highlight("Stop Aligning...\n");
    
    //获取最终的配准结果并打印
    Eigen::Matrix4f transformation2 = icp.getFinalTransformation();
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation2(0, 0), transformation2(0, 1), transformation2(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation2(1, 0), transformation2(1, 1), transformation2(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation2(2, 0), transformation2(2, 1), transformation2(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation2(0, 3), transformation2(1, 3), transformation2(2, 3));
    pcl::console::print_info("\n");

    pcl::console::print_info("remaining_cloud size, %d , %d \n", remaining_cloud->points.size(), object_xyz->points.size());

    Eigen::Matrix4f transformationfinal;
    //获取最终的标定块基于相机的位置
    transformationfinal = transformation2.inverse() * transform_aru_inv; // 因为之前ＩＣＰ写成了环境匹配模型，而不是模型匹配环境，所以要逆
    //打印结果，并保存最终结果
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformationfinal(0, 0), transformationfinal(0, 1), transformationfinal(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformationfinal(1, 0), transformationfinal(1, 1), transformationfinal(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformationfinal(2, 0), transformationfinal(2, 1), transformationfinal(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformationfinal(0, 3), transformationfinal(1, 3), transformationfinal(2, 3));
    pcl::console::print_info("\n");
    transx = transformationfinal(0, 3);
    transy = transformationfinal(1, 3);
    transz = transformationfinal(2, 3);

    Eigen::Matrix3f rotation_final;
    rotation_final << transformationfinal(0, 0), transformationfinal(0, 1), transformationfinal(0, 2),
        transformationfinal(1, 0), transformationfinal(1, 1), transformationfinal(1, 2),
        transformationfinal(2, 0), transformationfinal(2, 1), transformationfinal(2, 2);

    q_r_final = rotation_final;
    // 使用最终变换矩阵变换原始模型点云，得到配准后的点云。
    pcl::transformPointCloud(*ori_object_xyz, *final_model, transformationfinal);

    if (ifshowfinalresult)
    {
      cout << "Close PCL windowns to publish tf" << endl;
      //在原点处添加一个坐标系
      visu.addCoordinateSystem(0.2);
      //设置模型点云的颜色为蓝色
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> PointCloud_color(final_model, 0.0, 0.0, 255.0);
      //显示模型点云
      visu.addPointCloud<pcl::PointXYZ>(final_model, PointCloud_color, "object_aligned");
      // 设置点云"object_aligned"中每个点在可视化中的点大小为10。
      visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "object_aligned");
      visu.spin();
    }
  }
  //此时点击pcl展示窗口的x即可发布tf
  cout << "sending transforms" << endl;
  while (ros::ok())
  {
    //  初始化tf数据
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(transx, transy, transz));
    transform.setRotation(tf::Quaternion(q_r_final.x(), q_r_final.y(), q_r_final.z(), q_r_final.w()));

    // 广播base_link与base_laser坐标系之间的tf数据
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_optical_frame", "object"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tracking_origin", "tracking_marker")); // tracking_origin其对应就是camera_color_optical_frame
  }
  return (0);
}

// PRecon.cpp: 定义应用程序的入口点。
//

#include "PRecon.h"
#include "PoissonRecon/PoissonRecon.h"
#include "PoissonRecon/SurfaceTrim.h"
#include "PoissonRecon/Geometry.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/ply_io.h"
#include "pcl/features/normal_3d_omp.h"
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>

using namespace std;

void savePLY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne, string filename) {
	int elementNum = cloud->points.size();
	std::ofstream out(filename);
	out << "ply" << std::endl;
	out << "format ascii 1.0" << std::endl;
	out << "comment File generated" << std::endl;
	out << "element vertex " << elementNum << std::endl;
	out << "property float x" << std::endl;
	out << "property float y" << std::endl;
	out << "property float z" << std::endl;
	out << "property uchar red" << std::endl;
	out << "property uchar green" << std::endl;
	out << "property uchar blue" << std::endl;
	out << "property float nx" << std::endl;
	out << "property float ny" << std::endl;
	out << "property float nz" << std::endl;
	out << "end_header" << std::endl;

	//out << setprecision(6);
	for (int i = 0; i < elementNum; i++)
	{
		out << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " ";
		out << cloud->points[i].r << " " << cloud->points[i].g << " " << cloud->points[i].b << " ";
		out << ne[i].x << " " << ne[i].y << " " << ne[i].z << std::endl;
	}

	out.close();
}

int main()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPLYFile("D://test.ply", *cloud) < 0) {
		PCL_ERROR("点云不存在！");
		return -1;
	}

	cout << "->加载点云个数：" << cloud->points.size() << endl;
	//--------------------------- 法线估计 ---------------------------
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;		//创建法线估计对象
	ne.setInputCloud(cloud);							//设置法线估计输入点云
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());	//创建一个空的kdtree
	ne.setSearchMethod(tree);													//将空kdtree传递给法线估计对象 ne
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//法向量计算结果
	ne.setKSearch(10);			//设置K近邻的个数
	//ne.setRadiusSearch(0.05);	//设置半径邻域的大小，两种方式二选一
	ne.setViewPoint(0, 0, 0);	//设置视点向量，默认0向量(0,0,0)，没有方向
	ne.compute(*normals);		//执行法线估计，并将结果保存到normals中

	//pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	savePLY(cloud, ne, "D://tmp-n.ply");
	CoredVectorMeshData< PlyColorAndValueVertex < double > > mesh;
	callPoissonRecon<double, 2>(cloud, ne, mesh, 10);
	callSurfaceTrim<PlyColorAndValueVertex < double >>(mesh);
	return 0;
}

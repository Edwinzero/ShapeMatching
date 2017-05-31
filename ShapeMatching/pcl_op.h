#pragma once
#ifndef _PCL_OP_H
#define _PCL_OP_H

void CreatePCLCloud(std::vector<Eigen::Vector4f> &points, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	cloud->points.resize(points.size());
	for (int i = 0; i < points.size(); i++) {
		cloud->points[i].getArray4fMap() = points[i];
	}
}
void CreatePCLCloud(std::vector<Eigen::Vector4f> &normals, pcl::PointCloud<pcl::Normal>::Ptr cloud) {
	cloud->points.resize(normals.size());
	for (int i = 0; i < normals.size(); i++) {
		cloud->points[i].getNormalVector4fMap() = normals[i];
	}
}

void PCLfpfhEstimation(pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features, vector<Eigen::Vector4f> &points, vector<Eigen::Vector4f> &normals) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
	// Provide the original point cloud (without normals)
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	CreatePCLCloud(points, cloud);
	fpfh_estimation.setInputCloud(cloud);
	// Provide the point cloud with normals
	pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::Normal>);
	CreatePCLCloud(normals, cloud_with_normals);
	fpfh_estimation.setInputNormals(cloud_with_normals);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);		// way too slow, change to gpu
	fpfh_estimation.setSearchMethod(tree);
	fpfh_estimation.setRadiusSearch(0.2);// Actually compute the spin images
	fpfh_estimation.compute(*pfh_features);
}

#endif /*_PCL_OP_H*/
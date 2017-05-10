#pragma once
#ifndef _RGBD_MAPPING_CL_H
#define _RGBD_MAPPING_CL_H
#include <CLutils.h>
#include <cv_op.h>
#include <Eigen_op.h>
#include <Sensor.h>

class Mapping {
public:
	cl::int2			depth_img_size;		// depth map size
	cl::int2			color_img_size;		// color map size
	cl::float4			bound[2];			// volume size - world space

	cl::size2			global_size2;
	cl::size2			local_size2;

	cl::size3			global_size3;
	cl::size3			local_size3;

	// program
	cl::Context			context;
	cl::CommandQueue	cq;
	cl::Program			program;

	// kernel
	cl::Kernel			kn_d2p;
	cl::Kernel			kn_p2w; // view -> world
	cl::Kernel			kn_p2n; // if compute normal from view space will add work to transformation normal to g space

	cl::Kernel			kn_d2c; // depth->color
	cl::Kernel			kn_d2c2;  // map with id buf

	// mem
	cl::Mem				mem_d; // depth map
	cl::Mem				mem_p; // point map - view space
	cl::Mem				mem_n; // normal map
	cl::Mem				mem_w; // point map - world space

	cl::Mem				mem_c; // color map
	cl::Mem				mem_map; // mapped buffer
	cl::Mem				mem_mapID; // mapped pixel coord buffer

public:
	void Create(void) {
		// context
		cl::System::CreateContext(context);

		// queue
		cq.Create(context, CL_QUEUE_PROFILING_ENABLE);

		// mem
		mem_d.CreateBuffer(context, CL_MEM_READ_WRITE, depth_img_size.area() * sizeof(cl_ushort));
		mem_p.CreateBuffer(context, CL_MEM_READ_WRITE, depth_img_size.area() * sizeof(cl_float4));
		mem_n.CreateBuffer(context, CL_MEM_READ_WRITE, depth_img_size.area() * sizeof(cl_float4));
		mem_w.CreateBuffer(context, CL_MEM_READ_WRITE, depth_img_size.area() * sizeof(cl_float4));

		mem_c.CreateBuffer(context, CL_MEM_READ_WRITE, color_img_size.area() * sizeof(cl_uchar4));
		mem_map.CreateBuffer(context, CL_MEM_READ_WRITE, depth_img_size.area() * sizeof(cl_uchar4));
		mem_mapID.CreateBuffer(context, CL_MEM_READ_WRITE, depth_img_size.area() * 2 * sizeof(cl_int2));
	}


	void UpdateShader(const char *shader) {
		cq.Finish();
		program.Release();

		// program
		const char *flags = "-cl-denorms-are-zero -cl-mad-enable -cl-no-signed-zeros -I ./CLkernels ";
		program.Create(context, shader, flags);

		kn_d2p.Create(program, "depth_to_point");
		kn_p2w.Create(program, "point_to_world");
		kn_p2n.Create(program, "point_to_normal");

		kn_d2c.Create(program, "depth_to_color");
		kn_d2c2.Create(program, "depth_to_color2");
	}

	void Clear(void) {
		cq.Finish();
	}

	void DepthToRGBMapping(const cv::Mat &depth_intr, const cv::Mat &depth_extr,
		const cv::Mat &color_intr, const cv::Mat &color_extr,
		const unsigned short *depth, const cv::Mat &color, cv::Mat &res, std::vector<Eigen::Vector4f> &points) {
		// K
		std::vector<float> DK;
		MatToVec(depth_intr, DK);
		std::vector<float> CK;
		MatToVec(color_intr, CK);

		// M
		std::vector<float> M;
		// transpose extr will make 0001 to column 3
		cv::Mat trans = color_extr * depth_extr.inv();
		std::cout << trans << std::endl;
		MatToVec(trans.t(), M);

		// color
		cq.WriteBuffer(mem_d, CL_TRUE, 0, depth_img_size.area() * sizeof(cl_ushort), depth);
		std::vector<uchar> colorbuf(color.cols*color.rows*4);
		if (color.isContinuous()) {
			for (int y = 0; y < color.rows; y++) {
				for (int x = 0; x < color.cols; x++) {
					int id = 4*(y*color.cols + x);
					colorbuf[id] = color.at<cv::Vec3b>(y, x)[0];
					colorbuf[id + 1] = color.at<cv::Vec3b>(y, x)[1];
					colorbuf[id + 2] = color.at<cv::Vec3b>(y, x)[2];
					colorbuf[id + 3] = 0;
				}
			}
		}
#if 0 
		// test colorbuf
		cv::Mat test(cv::Size(1920, 1080), CV_8UC3, cv::Scalar(0));
		for (int y = 0; y < test.rows; y++) {
			for (int x = 0; x < test.cols; x++) {
				int id = 3*(y*test.cols + x);
				test.at<cv::Vec3b>(y, x)[0] = colorbuf[id];
				test.at<cv::Vec3b>(y, x)[1] = colorbuf[id + 1];
				test.at<cv::Vec3b>(y, x)[2] = colorbuf[id + 2];
			}
		}
		ImgShow("testdf asdf ", test, 960, 540);
#endif

		cq.WriteBuffer(mem_c, CL_TRUE, colorbuf);

		// rgbd mapping
		cq.NDRangeKernel2((kn_d2p << mem_d, mem_p, depth_img_size, DK), global_size2, local_size2);
		cq.NDRangeKernel2((kn_d2c << mem_p, mem_c, mem_map, depth_img_size, color_img_size, DK, CK, M), cl::size2(2048, 2048), local_size2);
		cq.Finish();
		std::vector<uchar> mapbuf(depth_img_size.area()*4);
		points.resize(depth_img_size.area());
		cq.ReadBuffer(mem_w, CL_TRUE, points);
		cq.ReadBuffer(mem_map, CL_TRUE, mapbuf);

		res = cv::Mat(cv::Size(512, 424), CV_8UC3, cv::Scalar(0, 0, 0));
		for (int y = 0; y < res.rows; y++) {
			for (int x = 0; x < res.cols; x++) {
				int id = 4 * (y*res.cols + x);
				res.at<cv::Vec3b>(y, x)[0] = mapbuf[id];     // B 
				res.at<cv::Vec3b>(y, x)[1] = mapbuf[id + 1]; // G
				res.at<cv::Vec3b>(y, x)[2] = mapbuf[id + 2]; // R
			}
		}
	}

	void DepthToRGBMapping(const cv::Mat &depth_intr, const cv::Mat &depth_extr, const cv::Mat &depth_to_color,
		const cv::Mat &color_intr, const cv::Mat &color_extr,
		const unsigned short *depth, const cv::Mat &color, cv::Mat &res, std::vector<Eigen::Vector4f> &points) {
		// K
		std::vector<float> DK;
		MatToVec(depth_intr, DK);
		std::vector<float> CK;
		MatToVec(color_intr, CK);

		// M
		std::vector<float> M;
		// transpose extr will make 0001 to column 3
		cv::Mat trans = depth_to_color;
		std::cout << trans << std::endl;
		MatToVec(trans.t(), M);

		// color
		cq.WriteBuffer(mem_d, CL_TRUE, 0, depth_img_size.area() * sizeof(cl_ushort), depth);
		std::vector<uchar> colorbuf(color.cols*color.rows * 4);
		if (color.isContinuous()) {
			for (int y = 0; y < color.rows; y++) {
				for (int x = 0; x < color.cols; x++) {
					int id = 4 * (y*color.cols + x);
					colorbuf[id] = color.at<cv::Vec3b>(y, x)[0];
					colorbuf[id + 1] = color.at<cv::Vec3b>(y, x)[1];
					colorbuf[id + 2] = color.at<cv::Vec3b>(y, x)[2];
					colorbuf[id + 3] = 0;
				}
			}
		}
#if 0 
		// test colorbuf
		cv::Mat test(cv::Size(1920, 1080), CV_8UC3, cv::Scalar(0));
		for (int y = 0; y < test.rows; y++) {
			for (int x = 0; x < test.cols; x++) {
				int id = 3 * (y*test.cols + x);
				test.at<cv::Vec3b>(y, x)[0] = colorbuf[id];
				test.at<cv::Vec3b>(y, x)[1] = colorbuf[id + 1];
				test.at<cv::Vec3b>(y, x)[2] = colorbuf[id + 2];
			}
		}
		ImgShow("testdf asdf ", test, 960, 540);
#endif

		cq.WriteBuffer(mem_c, CL_TRUE, colorbuf);

		// rgbd mapping
		cq.NDRangeKernel2((kn_d2p << mem_d, mem_p, depth_img_size, DK), global_size2, local_size2);
		cq.NDRangeKernel2((kn_d2c << mem_p, mem_c, mem_map, depth_img_size, color_img_size, DK, CK, M), cl::size2(2048, 2048), local_size2);
		cq.Finish();
		std::vector<uchar> mapbuf(depth_img_size.area() * 4);
		std::vector<int> mapbufID(depth_img_size.area() * 2);
		//points.resize(depth_img_size.area());
		//cq.ReadBuffer(mem_w, CL_TRUE, points);
		cq.ReadBuffer(mem_map, CL_TRUE, mapbuf);
		//cq.ReadBuffer(mem_mapID, CL_TRUE, mapbufID);

		res = cv::Mat(cv::Size(512, 424), CV_8UC3, cv::Scalar(0, 0, 0));
		for (int y = 0; y < res.rows; y++) {
			for (int x = 0; x < res.cols; x++) {
				int id = 4 * (y*res.cols + x);
				res.at<cv::Vec3b>(y, x)[0] = mapbuf[id];     // B 
				res.at<cv::Vec3b>(y, x)[1] = mapbuf[id + 1]; // G
				res.at<cv::Vec3b>(y, x)[2] = mapbuf[id + 2]; // R
			}
		}

		/*
		cv::Mat test = cv::Mat(cv::Size(512, 424), CV_8UC3, cv::Scalar(0, 0, 0));
		for (int i = 0; i < mapbufID.size()/2; i++) {
			int dID = mapbufID[2 * i];
			int cID = mapbufID[2 * i + 1];
			if (dID >= 0 && cID >= 0 && dID < 217088 && cID < 2073600) {
				test.at<cv::Vec3b>(dID) = color.at<cv::Vec3b>(cID);
			}
		}
		ImgShow("test mapbufID", test, 512, 424);
		//*/
	}


	void BackProjectPoints(const cv::Mat &intr, const cv::Mat &extr, const unsigned short *depth,
		std::vector<Eigen::Vector4f> &points, std::vector<Eigen::Vector4f> &normals) {
		// K
		std::vector<float> K;
		MatToVec(intr, K);

		// M
		std::vector<float> M;

#if 0
		double rep[] = {
			1, 0, 0, 0,
			0, 0, 1, 0,
			0, -1, 0, 0,
			0, 0, 0, 1 };
		MatToVec((cv::Mat(4, 4, CV_64F, rep) * extr).t(), M);
#else
		// transpose extr will make 0001 to column 3
		MatToVec(extr.t(), M);
#endif
		// depth
		cq.WriteBuffer(mem_d, CL_TRUE, 0, depth_img_size.area() * sizeof(cl_ushort), depth);

		// backprojection
		cq.NDRangeKernel2((kn_d2p << mem_d, mem_p, depth_img_size, K), global_size2, local_size2);
		cq.NDRangeKernel2((kn_p2w << mem_p, mem_w, depth_img_size, M), global_size2, local_size2);
		cq.NDRangeKernel2((kn_p2n << mem_p, mem_n, depth_img_size), global_size2, local_size2);

		points.resize(depth_img_size.area());
		normals.resize(depth_img_size.area());
		cq.ReadBuffer(mem_w, CL_TRUE, points);
		cq.ReadBuffer(mem_n, CL_TRUE, normals);
	}

	void BackProjectPointsShang(const cv::Mat &intr, const cv::Mat &extr, const unsigned short *depth,
		std::vector<Eigen::Vector4f> &points, std::vector<Eigen::Vector4f> &normals) {
		// K
		std::vector<float> K;
		MatToVec(intr, K);

		// M
		std::vector<float> M;

#if 1
		double rep[] = {
			0, 1, 0, 0,
			-1, 0, 0, 0.69,
			0, 0, 1, 0,
			0, 0, 0, 1 };
		MatToVec((cv::Mat(4, 4, CV_64F, rep)*extr).t(), M);
		//MatToVec((cv::Mat(4, 4, CV_64F, rep)).t(), M);
#else
		// transpose extr will make 0001 to column 3
		MatToVec(extr.t(), M);
#endif
		// depth
		cq.WriteBuffer(mem_d, CL_TRUE, 0, depth_img_size.area() * sizeof(cl_ushort), depth);

		// backprojection
		cq.NDRangeKernel2((kn_d2p << mem_d, mem_p, depth_img_size, K), global_size2, local_size2);
		cq.NDRangeKernel2((kn_p2w << mem_p, mem_w, depth_img_size, M), global_size2, local_size2);
		cq.NDRangeKernel2((kn_p2n << mem_p, mem_n, depth_img_size), global_size2, local_size2);

		points.resize(depth_img_size.area());
		normals.resize(depth_img_size.area());
		cq.ReadBuffer(mem_w, CL_TRUE, points);
		cq.ReadBuffer(mem_n, CL_TRUE, normals);
	}

};
#endif // !_RGBD_MAPPING_CL_H

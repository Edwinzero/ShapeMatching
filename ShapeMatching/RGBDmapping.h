#pragma once
#ifndef _RGBD_MAPPING_CL_H
#define _RGBD_MAPPING_CL_H
#include <CLutils.h>
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

	// mem
	cl::Mem				mem_d; // depth map
	cl::Mem				mem_p; // point map - view space
	cl::Mem				mem_n; // normal map
	cl::Mem				mem_w; // point map - world space

	cl::Mem				mem_c; // color map

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

		mem_c.CreateBuffer(context, CL_MEM_READ_WRITE, color_img_size.area() * sizeof(cl_float4));
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
	}

	void Clear(void) {
		cq.Finish();
	}

	void DepthToRGBMapping(const cv::Mat &depth_intr, const cv::Mat &depth_extr,
		const cv::Mat &color_intr, const cv::Mat &color_extr,
		const unsigned short *depth, const cv::Mat &color) {

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
		cq.NDRangeKernel2((kn_p2n << mem_w, mem_n, depth_img_size), global_size2, local_size2);

		points.resize(depth_img_size.area());
		normals.resize(depth_img_size.area());
		cq.ReadBuffer(mem_w, CL_TRUE, points);
		cq.ReadBuffer(mem_n, CL_TRUE, normals);
	}

};
#endif // !_RGBD_MAPPING_CL_H

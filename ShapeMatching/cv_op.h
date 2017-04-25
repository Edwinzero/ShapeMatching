//
//  cv_op.h
//  Moc
//
//  Created by Wei Li on 6/3/15.
//
#ifndef Moc_cv_op_h
#define Moc_cv_op_h
#include <opencv2\opencv.hpp>

template< class T >
cv::Vec<T,3> operator/(const cv::Vec<T,3> &a, T s) {
	s = 1.0f / s;
	return cv::Vec<T,3>(a.x * s, a.y * s, a.z * s);
}

template< class T >
cv::Point3_<T> operator/( const cv::Point3_<T> &a, T s ) {
	s = 1.0f / s;
	return cv::Point3_<T>( a.x * s, a.y * s, a.z * s );
}

template< class T >
cv::Point3_<T> normalize(const cv::Point3_<T> &a) {
	float mag = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
	float s = 1.0f / mag;
	return cv::Point3_<T>(a.x * s, a.y * s, a.z * s);
}

template< class T >
T magnitude(const cv::Point3_<T> &a) {
	float mag = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
	return mag;
}

template< class T >
cv::Point3_<T> operator*( const cv::Mat &m, const cv::Point3_<T> &v ) {
	cv::Point3_<T> t;
	t.x = m.at<double>( 0, 0 ) * v.x + m.at<double>( 0, 1 ) * v.y + m.at<double>( 0, 2 ) * v.z;
	t.y = m.at<double>( 1, 0 ) * v.x + m.at<double>( 1, 1 ) * v.y + m.at<double>( 1, 2 ) * v.z;
	t.z = m.at<double>( 2, 0 ) * v.x + m.at<double>( 2, 1 ) * v.y + m.at<double>( 2, 2 ) * v.z;

	if ( m.size().width == 4 ) {
		t.x += m.at<double>( 0, 3 );
		t.y += m.at<double>( 1, 3 );
		t.z += m.at<double>( 2, 3 );
	}

	return t;
}

template< class T >
cv::Point3_<T> Sum( const std::vector< cv::Point3_<T> > &v ) {
	cv::Point3_<T> sum( 0, 0, 0 );
	for ( int i = 0; i < v.size(); i++ ) {
		sum += v[ i ];
	}
	return sum;
}

template< class T >
cv::Mat Outer( const cv::Point3_<T> &a, const cv::Point3_<T> &b ) {
	cv::Mat t = cv::Mat_<double>( 3, 3 );

	const T *aa = &a.x;
	const T *bb = &b.x;
	for ( int i = 0; i < 3; i++ ) {
		for ( int j = 0; j < 3; j++ ) {
			t.at<double>( i, j ) = aa[ i ] * bb[ j ];
		}
	}

	return t;
}

template< class T >
T Length( const cv::Point3_<T> &v ) {
	return sqrt( v.x * v.x + v.y * v.y + v.z * v.z );
}

// rotate firstly, translate next
template< class T >
cv::Mat TranslateRotate( const cv::Point3_<T> &t, const cv::Mat &r ) {
	cv::Mat m = cv::Mat_<double>( 4, 4 );

	for ( int i = 0; i < 3; i++ ) {
		for ( int j = 0; j < 3; j++ ) {
			m.at<double>( i, j ) = r.at<double>( i, j );
		}
	}

	m.at<double>( 0, 3 ) = t.x;
	m.at<double>( 1, 3 ) = t.y;
	m.at<double>( 2, 3 ) = t.z;

	m.at<double>( 3, 0 ) = 0;
	m.at<double>( 3, 1 ) = 0;
	m.at<double>( 3, 2 ) = 0;
	m.at<double>( 3, 3 ) = 1;

	return m;
}

cv::Mat ExtractRotate(const cv::Mat &m) {
	cv::Mat r = cv::Mat_<double>(4, 4);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			r.at<double>(i, j) = m.at<double>(i, j);
		}
	}

	r.at<double>(0, 3) = 0;
	r.at<double>(1, 3) = 0;
	r.at<double>(2, 3) = 0;

	r.at<double>(3, 0) = 0;
	r.at<double>(3, 1) = 0;
	r.at<double>(3, 2) = 0;
	r.at<double>(3, 3) = 1;

	return r;
}

template< class T >
cv::Mat QuatToMat( T x, T y, T z, T w ) {
	T x2 = x + x;
	T y2 = y + y;
	T z2 = z + z;

	T xx2 = x * x2;
	T yy2 = y * y2;
	T zz2 = z * z2;

	T xy2 = x * y2;
	T yz2 = y * z2;
	T zx2 = z * x2;

	T wx2 = w * x2;
	T wy2 = w * y2;
	T wz2 = w * z2;

	cv::Mat m = cv::Mat_<double>( 3, 3 );

	m.at<double>( 0, 0 ) = 1 - yy2 - zz2;
	m.at<double>( 0, 1 ) =     xy2 - wz2;
	m.at<double>( 0, 2 ) =     zx2 + wy2;

	m.at<double>( 1, 0 ) =     xy2 + wz2;
	m.at<double>( 1, 1 ) = 1 - zz2 - xx2;
	m.at<double>( 1, 2 ) =     yz2 - wx2;

	m.at<double>( 2, 0 ) =     zx2 - wy2;
	m.at<double>( 2, 1 ) =     yz2 + wx2;
	m.at<double>( 2, 2 ) = 1 - xx2 - yy2;

	return m;
}

template< class T >
void MatToVec( const cv::Mat &m, std::vector<T> &v ) {
	int num = m.size().area();

	v.clear();
	v.resize( num );
	for ( int i = 0; i < num; i++ ) {
		v[ i ] = ((double*)m.ptr())[ i ];
	}
}
void MatToVec(const cv::Mat &m, std::vector<unsigned short> &v) {
	int num = m.size().area();

	v.clear();
	v.resize(num);
	for (int i = 0; i < num; i++) {
		v[i] = ((unsigned short*)m.ptr())[i];
	}
}

template< class T >
T Clamp( T v, T bmin, T bmax ) {
	return ( v <= bmin ? bmin : ( v >= bmax ? bmax : v ) );
}

void ImgShow(const char* name, cv::Mat &img, int width, int height) {
	cv::namedWindow(name, cv::WINDOW_NORMAL);
	cv::imshow(name, img);
	cv::resizeWindow(name, width, height);
	cv::waitKey(1);
}

std::string type2str(int type) {
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

#endif // Moc_cv_op_h
#ifndef __Moca_ACCUM__
#define __Moca_ACCUM__

template< class T >
class Accum {
public:
	int count;
	T sum;

public:
	Accum() {
		Reset();
	}

	void Reset(void) {
		count = 0;
		sum = 0;
	}

	Accum &operator+=(T &v) {
		count++;
		sum += v;
		return *this;
	}

	T Avg(void) const {
		if (count <= 0) {
			return T(0);
		}
		return sum / count;
	}
};

#endif
# GMS: Grid-based Motion Statistics for Fast, Ultra-robust Feature Correspondence

![alt tag](http://mmcheng.net/wp-content/uploads/2017/03/dog_ours.jpg)


Abstract

Incorporating smoothness constraints into feature matching is known to enable ultra-robust matching. However, such formulations are both complex and slow, making them unsuitable for video applications. This paper proposes GMS (Grid-based Motion Statistics), a simple means of encapsulating motion smoothness as the statistical likelihood of a certain number of matches in a region. GMS enables translation of high match numbers into high match quality. This provides a real-time, ultra-robust correspondence system. Evaluation on videos, with low textures, blurs and wide-baselines show GMS consistently out-performs other real-time matchers and can achieve parity with more sophisticated, much slower techniques.

Author

[JiaWang Bian](http://jwbian.net), Wen-Yan Lin, [Yasuyuki Matsushita](http://www-infobiz.ist.osaka-u.ac.jp/user/matsushita/index.html), [Sai-Kit Yeung](http://people.sutd.edu.sg/~saikit/), Tan Dat Nguyen, [Ming-Ming Cheng](http://mmcheng.net)


Publication:

**GMS: Grid-based Motion Statistics for Fast, Ultra-robust Feature Correspondence** JiaWang Bian, Daniel Lin, Yasuyuki Matsushita, Sai-Kit Yeung, Tan Dat Nguyen, Ming-Ming Cheng **IEEE CVPR, 2017** [[Project Page](http://jwbian.net/gms)] [[pdf](http://jwbian.net/Papers/GMS_CVPR17.pdf)] [[Code](https://github.com/JiawangBian/GMS-Feature-Matcher)] [[Video Demo](http://jwbian.net/Demo/gms_matching_demo.mp4)]


Citation 

	If you use the code in your publication, please cite our paper.
	
Video Matching Demo
	
[![IMAGE ALT TEXT HERE](http://jwbian.net/wp-content/uploads/2017/04/matching_demo_chair-e1492913756279.png)](https://youtu.be/3SlBqspLbxI)   [![IMAGE ALT TEXT HERE](http://jwbian.net/wp-content/uploads/2017/04/matching_demo_tum-e1492913770981.png)](https://youtu.be/tjMpgno6k5A)   [![IMAGE ALT TEXT HERE](http://jwbian.net/wp-content/uploads/2017/04/matching_demo_car-e1492913739458.png)](https://youtu.be/TIVWTTQTkeI)

Environment:

	The code can run on Windows, Linux, and Mac.

Requirement:

	1.OpenCV 3.0 or later (for IO and ORB features, necessary)

	2.cudafeatures2d module(for gpu nearest neighbor, optional)

How to run:

	Image pair demo and video demo in demo.cpp.

	(Note:	Please use gpu match when you run video demo.)
	
Tune Parameters:

	In Main.cpp
		1.#define USE_GPU" will need gpu cudafeatures2d module for nearest neighbor match, 
			using cpu match by commenting it.
	
	In GMS.h
		2.	#define LocalTheshold 1			// Chosing threshold has two ways : global and local
				1 for local threshold, 0 for global threshold;
				
		3.	#define TreshFactor 6			// factor for calculating threshold
				The higher, the less matches, vice verse
				
		4. 	GMS::setParameter(int num1, int num2 )
				The grid number = num * num for left and right image.
				num = 20 is a normal option.
				
		5. 	GMS::getInlier(int withRS)
				0 means no ration and no scale version (default)
				1 means with rotatio and scale change. (may better but slower)



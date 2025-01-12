#include "method_.h"

using namespace std;
using namespace cv;

//void* xmlMapReader(void* file)
//{
//	
//	cv::FileStorage fn(file->xmlMapName, cv::FileStorage::READ);
//	fn["Fisheye_Undistort_Map_mapxL"] >> file->mapImg;
//	fn.release(file);
//	return 
//}

void fisheyeCalib_(fisheyeCalibInfo infoStereoCalib)
{
	/****************************************
	***detect lines in calibration images
	****************************************/
	std::string fname = infoStereoCalib.calibPatternFile;	// "D:\\studying\\stereo vision\\research code\\fisheye-stereo-calibrate\\fisheye-library\\MyCalibration_\\fisheyeCalib_\\fisheyeCalib_\\patternsL.xml";
	LineDetection ld;
	//    ld.editAllEdges(ld.loadEdgeXML(fname));

	//    std::vector<std::vector<std::vector<cv::Point2i> > > edges = ld.loadEdgeXML(fname);
	//    ld.saveParameters();
	//    ld.editAllEdges(edges);
	ld.loadImageXML(fname);
	ld.saveParameters();

	ld.processAllImages();
	//
	std::string output = infoStereoCalib.calibLineDetected;
	ld.writeXML(output);


	/****************************************
	***calibration with pre-detected lines
	****************************************/
	Calibration calib;
	std::string filename = infoStereoCalib.calibLineDetected;
	calib.loadData(filename);
	int a_size = 4;
	IncidentVector::initA(a_size);
	//    std::vector<double> a;
	//    a.push_back(5e-3); a.push_back( 6e-4); a.push_back( 7e-5); a.push_back( 8e-6); a.push_back( 9e-7);
	//    a_size = 5;
	//    IncidentVector::setA(a);
	//    int x = atoi(argv[2]), y = atoi(argv[3]), f = atoi(argv[4]);
	//    double x = 953., y = 600., f = 401.;
	//    IncidentVector::setF(f);
	//    IncidentVector::setCenter(cv::Point2d(793,606));
	//    IncidentVector::setF0((int)f);

	std::cout << "Projection Model:\t" << IncidentVector::getProjectionName() << std::endl;
	std::cout << "Center:\t" << IncidentVector::getCenter() << std::endl;
	std::cout << "     f:\t" << IncidentVector::getF() << std::endl;
	for (int i = 0; i < IncidentVector::nparam - 3; ++i) {
		std::cout << "    a" << i << ":\t" << IncidentVector::getA().at(i) << std::endl;
	}

	std::cout << "Orthogonal pairs: " << calib.edges.size() << std::endl;
	long lines = 0;
	long points = 0;
	for (auto &pair : calib.edges) {
		lines += pair.edge[0].size() + pair.edge[1].size();
		for (auto &line : pair.edge[0]) {
			points += line.size();
		}
		for (auto &line : pair.edge[1]) {
			points += line.size();
		}
	}
	std::cout << "Lines: " << lines << std::endl;
	std::cout << "Points: " << points << std::endl;


	// Show an image of all edges
//    cv::Mat img = cv::Mat::zeros(IncidentVector::getImgSize().height, IncidentVector::getImgSize().width, CV_8UC3);
//    cv::Vec3b color[30] = {cv::Vec3b(255,255,255), cv::Vec3b(255,0,0), cv::Vec3b(255,255,0), cv::Vec3b(0,255,0), cv::Vec3b(0,0,255),
//        cv::Vec3b(255,0,255), cv::Vec3b(204,51,51), cv::Vec3b(204,204,51), cv::Vec3b(51,204,51), cv::Vec3b(51,204,204),
//        cv::Vec3b(51,51,204), cv::Vec3b(204,51,204), cv::Vec3b(204,204,204), cv::Vec3b(153,102,102), cv::Vec3b(153,153,102),
//        cv::Vec3b(102,153,102), cv::Vec3b(102,153,153), cv::Vec3b(102,102,153), cv::Vec3b(153,102,153), cv::Vec3b(153,153,153),
//        cv::Vec3b(51,51,204), cv::Vec3b(204,51,204), cv::Vec3b(204,204,204), cv::Vec3b(153,102,102), cv::Vec3b(153,153,102),
//        cv::Vec3b(102,153,102), cv::Vec3b(102,153,153), cv::Vec3b(102,102,153), cv::Vec3b(153,102,153), cv::Vec3b(153,153,153),
//    };
//    cv::namedWindow("lines", CV_WINDOW_NORMAL);
//    int j = 0;
//    for (auto &pair : calib.edges) {
//        for (int i = 0; i < 2; ++i) {
//            for (auto &line : pair.edge[i]) {
//                for (auto &point : line) {
//                    img.at<cv::Vec3b>(int(point->point.y), int(point->point.x)) = color[j%30];
//                }
//            }
//        }
//        cv::imshow("lines", img);
//        cv::waitKey();
//        ++j;
//    }
//        cv::imshow("edges", img);
//        cv::waitKey();
//        img = cv::Mat::zeros(IncidentVector::getImgSize().height, IncidentVector::getImgSize().width, CV_8UC1);
//    cv::imwrite("lines.png", img);

//    if (std::string(argv[7]) == std::string("divide")) {
//        calib.calibrate(true);
//    } else {
//        calib.calibrate(false);
//    }

//    IncidentVector::initA(0);
//    calib.calibrate(false);
//    IncidentVector::initA(1);
//    calib.calibrate(false);
	IncidentVector::initA(a_size);
	calib.calibrate(false);
	//calib.calibrate(true);
		//calib.calibrate2();

	std::string outname = infoStereoCalib.calibFile;
	calib.save(outname);

	//    calib.calibrate(true);
	//    calib.save(std::string("d_")+outname);

	std::cout << "END" << std::endl;

}

void fisheyeUndistort_(std::string filePath_, Size imgSize, cv::Mat mapX,
	cv::Mat mapY, std::vector<cv::Mat>& imgUndistort)
{
	//load all the images in the folder
	String filePath = filePath_ + "\\*.jpg";
	std::vector<String> fileNames;
	glob(filePath, fileNames, false);

	if(!imgUndistort.empty())
	{
		imgUndistort.clear();
	}
	for(int i = 0; i < fileNames.size(); i++)
	{
		Mat imgSrc = imread(fileNames[i]);
		if(imgSrc.size() != imgSize)
		{
			resize(imgSrc, imgSrc, imgSize);
		}
		Mat imgDst;
		remap(imgSrc, imgDst, mapX, mapY, INTER_LINEAR, BORDER_TRANSPARENT);
		imgUndistort.push_back(imgDst);
		imwrite(fileNames[i].substr(0, fileNames[i].length() - 4) + "_undistort.jpg", imgDst);
	}
}

bool ptsDetect_calib(std::vector<cv::Mat> imgsL, std::vector<cv::Mat> imgsR, douVecPt2f& ptsL, douVecPt2f& ptsR,
	douVecPt3f& ptsReal, int corRowNum, int corColNum)
{
	if (imgsL.size() != imgsR.size() || imgsL.size() <= 0)
	{
		cout << "images error" << endl;
		return false;
	}

	if (!ptsL.empty())
	{
		ptsL.clear();
	}

	if (!ptsR.empty())
	{
		ptsR.clear();
	}

	if (!ptsReal.empty())
	{
		ptsReal.clear();
	}

	//
	Size patternSize(corRowNum, corColNum);		//0:the number of inner corners in each row of the chess board
																//1:the number of inner corners in each col of the chess board

	//detect the inner corner in each chess image
	for (int i = 0; i < imgsL.size(); i++)
	{
		Mat img_left = imgsL[i];
		Mat img_right = imgsR[i];

		if (img_left.rows != img_right.rows && img_left.cols != img_right.cols)
		{
			std::cout << "image error" << std::endl;
			return false;
		}

		std::vector<Point2f> cornerPts_left, cornerPts_right;
		bool patternFound_left = findChessboardCorners(img_left, patternSize, cornerPts_left,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		bool patternFound_right = findChessboardCorners(img_right, patternSize, cornerPts_right,
			CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
		if (patternFound_left && patternFound_right)
		{
			Mat imgL_gray, imgR_gray;
			cvtColor(img_left, imgL_gray, COLOR_RGB2GRAY);
			cornerSubPix(imgL_gray, cornerPts_left, Size(3, 3), Size(-1, -1),
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));
			ptsL.push_back(cornerPts_left);
			drawChessboardCorners(img_left, patternSize, cornerPts_left, patternFound_left);

			cvtColor(img_right, imgR_gray, COLOR_RGB2GRAY);
			cornerSubPix(imgR_gray, cornerPts_right, Size(3, 3), Size(-1, -1),
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 1e-6));
			ptsR.push_back(cornerPts_right);
			drawChessboardCorners(img_right, patternSize, cornerPts_right, patternFound_right);
		}
	}

	//two cameras calibration
	Size2f squareSize(100, 100);		//the real size of each grid in the chess board,which is measured manually by ruler

	std::vector<Point3f> tempPts;
	for (int i = 0; i < patternSize.height; i++)
	{
		for (int j = 0; j < patternSize.width; j++)
		{
			Point3f realPt;
			realPt.x = j * squareSize.width;
			realPt.y = i * squareSize.height;
			realPt.z = 0;
			tempPts.push_back(realPt);
		}
	}
	for (int i = 0; i < ptsL.size(); i++)
	{
		ptsReal.push_back(tempPts);
	}

	return true;
}

/**
 * \brief merge the input four images to one big image:\merged
 * \param tl
 * \param tr
 * \param bl
 * \param br
 * \param merged
 */
void merge4(const cv::Mat& tl, const cv::Mat& tr, const cv::Mat& bl, const cv::Mat& br, cv::Mat& merged)
{
	int type = tl.type();
	cv::Size sz = tl.size();
	if (type != tr.type() || type != bl.type() || type != br.type()
		|| sz.width != tr.cols || sz.width != bl.cols || sz.width != br.cols
		|| sz.height != tr.rows || sz.height != bl.rows || sz.height != br.rows)
	{
		cout << "rectify failed." << endl;
		return;
	}

	merged.create(cv::Size(sz.width * 2, sz.height * 2), type);
	tl.copyTo(merged(cv::Rect(0, 0, sz.width, sz.height)));
	tr.copyTo(merged(cv::Rect(sz.width, 0, sz.width, sz.height)));
	bl.copyTo(merged(cv::Rect(0, sz.height, sz.width, sz.height)));
	br.copyTo(merged(cv::Rect(sz.width, sz.height, sz.width, sz.height)));
}

/****************************************
***reprojection and get the rectify remap
****************************************/
void rectify_(calibInfo infoStereoCalib)
{
	std::vector<cv::Mat> imgUndistortL, imgUndistortR;
	cv::Mat mapxL, mapyL;
	cv::Mat mapxR, mapyR;

	{
		Reprojection reproj;
		double f_;

		std::string param = infoStereoCalib.calibFileL;
		//std::cout << "Type parameter file name > ";
		//std::cin >> param;
		reproj.loadPrameters(param);

		// Print parameters
		std::cout << "f: " << IncidentVector::getF() << "\nf0: " << IncidentVector::getF0() << std::endl;
		std::cout << "center: " << IncidentVector::getCenter() << std::endl;
		std::cout << "image size: " << IncidentVector::getImgSize() << std::endl;
		std::cout << "ai: ";
		std::vector<double> a_s = IncidentVector::getA();
		for (std::vector<double>::iterator it = a_s.begin(); it != a_s.end(); it++) {
			std::cout << *it << '\t';
		}
		std::cout << std::endl;

		reproj.theta2radius();
		//    reproj.saveRadius2Theta("Stereographic.dat");

		f_ = IncidentVector::getF();
		reproj.calcMaps(f_, mapxL, mapyL);

		std::string filePathL = infoStereoCalib.calibChessImgPathL;//"D:\\studying\\stereo vision\\research code\\data\\20190719\\camera_jpg_2\\left"
		//fisheyeUndistort_(filePathL, IncidentVector::getImgSize(), mapxL, mapyL, imgUndistortL);
	}
	{
		Reprojection reproj;
		double f_;

		std::string param = infoStereoCalib.calibFileR;// "resCalibR.xml";
		//std::cout << "Type parameter file name > ";
		//std::cin >> param;
		reproj.loadPrameters(param);

		// Print parameters
		std::cout << "f: " << IncidentVector::getF() << "\nf0: " << IncidentVector::getF0() << std::endl;
		std::cout << "center: " << IncidentVector::getCenter() << std::endl;
		std::cout << "image size: " << IncidentVector::getImgSize() << std::endl;
		std::cout << "ai: ";
		std::vector<double> a_s = IncidentVector::getA();
		for (std::vector<double>::iterator it = a_s.begin(); it != a_s.end(); it++) {
			std::cout << *it << '\t';
		}
		std::cout << std::endl;

		reproj.theta2radius();
		//    reproj.saveRadius2Theta("Stereographic.dat");

		f_ = IncidentVector::getF();
		reproj.calcMaps(f_, mapxR, mapyR);

		std::string filePathR = infoStereoCalib.calibChessImgPathR;// "D:\\studying\\stereo vision\\research code\\data\\20190719\\camera_jpg_2\\right";
		//fisheyeUndistort_(filePathR, IncidentVector::getImgSize(), mapxR, mapyR, imgUndistortR);
	}
	FileStorage fn_1(infoStereoCalib.stereoCalib_undistort_mapxL, FileStorage::WRITE);
	fn_1 << "Fisheye_Undistort_Map_mapxL" << mapxL;
	fn_1.release();

	FileStorage fn_2(infoStereoCalib.stereoCalib_undistort_mapyL, FileStorage::WRITE);
	fn_2 << "Fisheye_Undistort_Map_mapyL" << mapyL;
	fn_2.release();

	FileStorage fn_3(infoStereoCalib.stereoCalib_undistort_mapxR, FileStorage::WRITE);
	fn_3 << "Fisheye_Undistort_Map_mapxR" << mapxR;
	fn_3.release();

	FileStorage fn_4(infoStereoCalib.stereoCalib_undistort_mapyR, FileStorage::WRITE);
	fn_4 << "Fisheye_Undistort_Map_mapyR" << mapyR;
	fn_4.release();

	// stereo calibration
	douVecPt2f ptsLeft, ptsRight;
	douVecPt3f ptsReal;
	ptsDetect_calib(imgUndistortL, imgUndistortR, ptsLeft, ptsRight, ptsReal, infoStereoCalib.chessRowNum, infoStereoCalib.chessColNum);

	cv::Size imgSize = IncidentVector::getImgSize();

	cv::Mat K1, K2, D1, D2;
	vector<Mat> Rs_L, Ts_L, Rs_R, Ts_R;
	double rmsL = calibrateCamera(ptsReal, ptsLeft, imgSize, K1, D1, Rs_L, Ts_L);
	double rmsR = calibrateCamera(ptsReal, ptsRight, imgSize, K2, D2, Rs_R, Ts_R);

	cv::Mat matrixR, matrixT;
	cv::Mat E, F, Q;
	int stereoFlag = 0;
	stereoFlag |= cv::CALIB_USE_INTRINSIC_GUESS;
	//stereoFlag |= cv::CALIB_FIX_S1_S2_S3_S4;
	//stereoFlag |= cv::CALIB_ZERO_TANGENT_DIST;
	//stereoFlag |= cv::CALIB_FIX_INTRINSIC;
	//stereoFlag |= cv::CALIB_FIX_INTRINSIC;
	//stereoFlag |= cv::CALIB_FIX_INTRINSIC;
	//stereoFlag |= cv::CALIB_FIX_INTRINSIC;
	//stereoFlag |= cv::CALIB_FIX_INTRINSIC;
	//stereoFlag |= cv::CALIB_FIX_INTRINSIC;
	//stereoFlag |= cv::CALIB_FIX_INTRINSIC;

	double rms = stereoCalibrate(ptsReal, ptsLeft, ptsRight,
		K1, D1, K2, D2,
		IncidentVector::getImgSize(), matrixR, matrixT, E, F, Q, stereoFlag,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-16));

	cout << "˫Ŀ�궨��" << rms << endl;

	cv::Mat R1, R2, P1, P2, Q_;
	double balance = 0.0, fov_scale = 1.0;
	stereoRectify(K1, D1, K2, D2,
		imgSize, matrixR, matrixT, R1, R2, P1, P2, Q_,
		cv::CALIB_ZERO_DISPARITY);//, imgSize, balance, fov_scale

	cv::Mat lmapx, lmapy, rmapx, rmapy;
	//
	initUndistortRectifyMap(K1, D1, R1, P1, imgSize, CV_32F, lmapx, lmapy);
	initUndistortRectifyMap(K2, D2, R2, P2, imgSize, CV_32F, rmapx, rmapy);

	FileStorage fn(infoStereoCalib.stereoCalib, FileStorage::WRITE);
	fn << "ImgSize" << imgSize;
	fn << "StereoCalib_K1" << K1;
	fn << "StereoCalib_D1" << D1;
	fn << "StereoCalib_K2" << K2;
	fn << "StereoCalib_D2" << D2;
	fn << "StereoCalib_R" << matrixR;
	fn << "StereoCalib_T" << matrixT;
	fn << "StereoCalib_E" << E;
	fn << "StereoCalib_F" << F;
	fn << "Rectify_R1" << R1;
	fn << "Rectify_R2" << R2;
	fn << "Rectify_P1" << P1;
	fn << "Rectify_P2" << P2;
	fn << "Rectify_Q" << Q_;
	fn.release();

	FileStorage fn_5(infoStereoCalib.stereoCalib_rectify_mapxL, FileStorage::WRITE);
	fn_5 << "Stereo_Rectify_Map_mapxL" << lmapx;
	fn_5.release();

	FileStorage fn_6(infoStereoCalib.stereoCalib_rectify_mapyL, FileStorage::WRITE);
	fn_6 << "Stereo_Rectify_Map_mapyL" << lmapy;
	fn_6.release();

	FileStorage fn_7(infoStereoCalib.stereoCalib_rectify_mapxR, FileStorage::WRITE);
	fn_7 << "Stereo_Rectify_Map_mapxR" << rmapx;
	fn_7.release();

	FileStorage fn_8(infoStereoCalib.stereoCalib_rectify_mapyR, FileStorage::WRITE);
	fn_8 << "Stereo_Rectify_Map_mapyR" << rmapy;
	fn_8.release();
}

#include<opencv2/opencv.hpp>
#include<iostream>
#include<cstdio>
#include<omp.h>
#include "USart.h"   
using namespace std;
using namespace cv;

#define T_ANGLE_THRE 10
#define T_SIZE_THRE 5

bool ISempty(vector<Point>rhs)                                 //判断数组是否为空
{
	return rhs.empty();
}

void sharpen2D_1(Mat&src, Mat&output)
{
	Mat kernel(3, 3, CV_32F, Scalar(0));
	kernel.at<float>(1, 1) = 5.0f;
	kernel.at<float>(0, 1) = -1.0f;
	kernel.at<float>(2, 1) = -1.0f;
	kernel.at<float>(0, 1) = -1.0f;
	kernel.at<float>(1, 2) = -1.0f;
	filter2D(src, output, src.depth(), kernel);
}
void brightAdjust(Mat &src, Mat &dst, double dContrast, double dBright)
{
	int nVal;
	int rowNumber = dst.rows;
	int colNumber = dst.cols*dst.channels();

	omp_set_num_threads(8);
#pragma omp parallel for

	for (int i = 0; i < rowNumber; i++)
	{
		uchar* dstdata = dst.ptr<uchar>(i);
		uchar* srcdata = src.ptr<uchar>(i);
		for (int j = 0; j < colNumber - 1; j++)
		{
			//nVal = saturate_cast<uchar>((dContrast * srcdata[j]) + dBright); //严重影响效率
			nVal = (int)(dContrast * srcdata[j] + dBright);
			if (nVal > 255) nVal = 255;
			else if (nVal < 0) nVal = 0;
			dstdata[j] = nVal;
		}
	}
}

void getDiffImage(Mat &src1, Mat &src2, Mat &dst, int nThre)
{
	int nVal;
	int rowNumber = src1.rows;
	int colNumber = src1.cols * src1.channels();

	omp_set_num_threads(8);
#pragma omp parallel for

	for (int i = 0; i < rowNumber; i++)
	{
		uchar* srcData1 = src1.ptr<uchar>(i);
		uchar* srcData2 = src2.ptr<uchar>(i);
		uchar* dstData = dst.ptr<uchar>(i);
		for (int j = 0; j < colNumber; j++)
		{
			if (srcData1[j] - srcData2[j]> nThre)
				dstData[j] = 255;
			else
				dstData[j] = 0;
		}
	}
}

vector<RotatedRect> armorDetect(vector<RotatedRect> vEllipse)
{
	vector<RotatedRect> vRlt;
	RotatedRect armor;
	int nL, nW;
	float areaI, areaJ;
	double dAngle;
	vRlt.clear();
	if (vEllipse.size() < 2)
		return vRlt;
	for (unsigned int nI = 0; nI < vEllipse.size() - 1; nI++)
	{
		for (unsigned int nJ = nI + 1; nJ < vEllipse.size(); nJ++)
		{
			//areaI = vEllipse[nI].size.area();
			//areaJ = vEllipse[nJ].size.area();
			dAngle = abs(vEllipse[nI].angle - vEllipse[nJ].angle);
			while (dAngle > 180)
				dAngle -= 180;
			if ((dAngle < T_ANGLE_THRE || 180 - dAngle < T_ANGLE_THRE) && abs(vEllipse[nI].size.height - vEllipse[nJ].size.height) < (vEllipse[nI].size.height + vEllipse[nJ].size.height) / T_SIZE_THRE && abs(vEllipse[nI].size.width - vEllipse[nJ].size.width) < (vEllipse[nI].size.width + vEllipse[nJ].size.width) / T_SIZE_THRE && (abs(vEllipse[nI].center.y - vEllipse[nJ].center.y)<50))
			{
				armor.center.x = (vEllipse[nI].center.x + vEllipse[nJ].center.x) / 2;
				armor.center.y = (vEllipse[nI].center.y + vEllipse[nJ].center.y) / 2;
				armor.angle = (vEllipse[nI].angle + vEllipse[nJ].angle) / 2;
				if (180 - dAngle < T_ANGLE_THRE)
					armor.angle += 90;
				nL = (vEllipse[nI].size.height + vEllipse[nJ].size.height) / 2;
				nW = sqrt((vEllipse[nI].center.x - vEllipse[nJ].center.x) * (vEllipse[nI].center.x - vEllipse[nJ].center.x) + (vEllipse[nI].center.y - vEllipse[nJ].center.y) * (vEllipse[nI].center.y - vEllipse[nJ].center.y));

				if (nL < nW)
				{
					armor.size.height = nL;
					armor.size.width = nW;
				}
				else
				{
					armor.size.height = nW;
					armor.size.width = nL;
				}
				//if ((areaI / areaJ<1.1) || (areaJ / areaI<1.1))
				vRlt.push_back(armor);

			}
		}
	}
	return vRlt;
}

Point2f drawBox(RotatedRect box, Mat &img)
{
	Point2f vertex[4], pointafa;
	box.points(vertex);
	pointafa.x = (vertex[2].x + vertex[0].x) / 2 - 160;
	pointafa.y = (vertex[2].y + vertex[0].y) / 2 - 120;
	for (int i = 0; i < 4; i++)
	{
		line(img, vertex[i % 4], vertex[(i + 1) % 4], Scalar(0, 0, 255), 2, CV_AA);
	}
	//cout << vertex[0] << endl;
	//cout << vertex[2] << endl;

	return pointafa;

}


int main()
{
	CSerialPort mySerialPort;

	if (!mySerialPort.InitPort(4))
	{
		std::cout << "initPort fail !" << std::endl;
	}
	else
	{
		std::cout << "initPort success !" << std::endl;
	}
	if (!mySerialPort.OpenListenThread())
	{
		std::cout << "OpenListenThread fail !" << std::endl;
	}
	else
	{
		std::cout << "OpenListenThread success !" << std::endl;
	}
	int g_MedianBlurValue = 16;
	bool bFlag = true;
	vector<Mat> channels;
	vector<RotatedRect> vEllipse;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<RotatedRect> vRlt;
	RotatedRect box, frembox;
	Point2f vertex[4];
	Mat frame, bImage, gImage, rImage, rawImage, grayImage, rlt, blurimage,redimage;
	Mat binary;
	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	vector<Point>slidfram;
	double time = 0;
	unsigned int frames = 0;
	while (waitKey(30) != 27)
	{
		cap >> frame;
		//resize(frame, frame, Size(320, 240));
		//cvtColor(frame, frame, 6);
		//equalizeHist(frame, frame);
		//cvtColor(frame, frame, 8);
		frame.copyTo(rawImage);
		/*frame.copyTo(redimage);
		cvtColor(redimage, redimage, 40);
		inRange(redimage, Scalar(0, 43, 46), Scalar(10, 255, 255), redimage);
		imshow("redimage", redimage);
		frame.copyTo(blurimage);
		cvtColor(blurimage, blurimage, 40);
		inRange(blurimage, Scalar(125, 43, 46), Scalar(155, 255, 255), blurimage);
		imshow("blurimage", blurimage);*/
		//	frames++;
		double t0 = getTickCount();
		//sharpen2D_1(frame, frame);
		GaussianBlur(binary, binary, Size(17, 17), 0, 0);
		//blur(binary, binary, Size(20, 20), Point(-1, -1));
		brightAdjust(frame, rawImage, 1, -230);//白180晚160
		sharpen2D_1(rawImage, rawImage);
		
		split(rawImage, channels);
		bImage = channels.at(0);
		gImage = channels.at(1);
		rImage = channels.at(2);
		rImage.copyTo(binary);
		getDiffImage(rImage, gImage, binary, 5);//40---80


		
		imshow("Bi", binary);
		imshow("RLT1", rlt);

		vector<vector<Point> >contours, contours1;
		vector<Vec4i>hierarchy, hierarchy1;
		findContours(rlt, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		if (contours.size() == 0)
			continue;
		for (int i = 0; i < contours.size(); i++)
		{
			if (contourArea(contours[i])>50)
				//if ((contourArea(contours)>100 && contours.size()>10) || (contours.size()<10 && contours.size()>10) )
			{
				box = minAreaRect(Mat(contours[i]));
				box.points(vertex);
				for (int i = 0; i < 4; i++)
					line(rlt, vertex[i % 4], vertex[(i + 1) % 4], Scalar(255), 2, CV_AA);


				if (((box.size.height / box.size.width) > 1.0)/* && ((box.size.height / box.size.width) <3.0)*/)
					bFlag = true;
				/*for (int nI = 0; (nI < 5); nI++)
				{
					for (int nJ = 0; (nJ < 5); nJ++)
					{
						if (box.center.y - 2 + nJ > 0 && box.center.y - 2 + nJ < 480 && box.center.x - 2 + nI > 0 && box.center.x - 2 + nI < 640)
						{
							Vec3b sx = frame.at<Vec3b>((int)(box.center.y - 2 + nJ), (int)(box.center.x - 2 + nI));
							if (sx[0] < 200 || sx[1] < 200 || sx[2] < 200)
							{
								bFlag = false;
							}
						}
					}
				}*/
				else bFlag = false;
				if (bFlag)
				{
					vEllipse.push_back(box);
				}
			}


		}
		Point2f point=Point(660,660);
		vector<Point2f> pointafa;
		Point2f pointdata;
		vector<float>datas;
		vRlt = armorDetect(vEllipse);
		int minlong;
		vector<int>woirth;
		int longest = 10000;
		for (unsigned int nI = 0; nI < vRlt.size(); nI++)
			//point=drawBox(vRlt[nI], frame);
		/*{
			Point2f vertex[4];
			vRlt[nI].points(vertex);
			pointdata.x = abs((vertex[2].x - vertex[0].x));
			pointdata.y = abs((vertex[2].y - vertex[0].y));
			if (vRlt.size()>0)
			if (longest > pointdata.y)
			{
				longest = pointdata.y;
				minlong = nI;
			    woirth.push_back(nI);
			}
			}
			if (vRlt.size() > 0) 
			for (int j = 0; j < woirth.size();j++)
			{
				if (vRlt[j].center.y == vRlt[minlong].center.y)
				{
					vRlt[j].points(vertex);
					for (int i = 0; i < 4; i++)
					{
						line(frame, vertex[i % 4], vertex[(i + 1) % 4], Scalar(0, 0, 255), 2, CV_AA);
					}
					point.x = (vertex[2].x + vertex[0].x) / 2 - 160;
					point.y = (vertex[2].y + vertex[0].y) / 2 - 120;
					circle(frame, Point(point.x + 160, point.y + 120), 5, Scalar(0, 255, 255));
				}
			}*/
		
		//	
		////求均值	
		//	{
		//	drawBox(vRlt[nI], frame);
		//	Point2f vertex[4];
		//	vRlt[nI].points(vertex);
		//	pointdata.x = abs((vertex[2].x - vertex[0].x));
		//	pointdata.y = abs((vertex[2].y - vertex[0].y));
		//	if ((pointdata.x / pointdata.y>1.4) && (pointdata.x / pointdata.y<2.3))
		//	pointafa.push_back(Point2f((vertex[2].x + vertex[0].x) / 2, (vertex[2].y + vertex[0].y) / 2));

		//	}
		//	float pllumx=0, pllumy=0;
		//	for (int i = 0; i < pointafa.size(); i++)
		//	{
		//	pllumx += pointafa[i].x;
		//	pllumy += pointafa[i].y;
		//	}
		//	if (pointafa.size()>0)
		//	{
		//	point.x = pllumx / pointafa.size() - 160;
		//	point.y = pllumy / pointafa.size() - 120;
		//	circle(frame, Point(point.x + 160, point.y + 120), 25, Scalar(0, 255, 255),5,4);
		//	}
			//去最小矩形
		{
			Point2f vertex[4];
			vRlt[nI].points(vertex);
			pointdata.x = abs((vertex[2].x - vertex[0].x));
			pointdata.y = abs((vertex[2].y - vertex[0].y));
			if (vRlt.size()>0)
			if (longest > pointdata.x)
			{longest = pointdata.x;

			minlong = nI;
			}
			
			if ((vRlt.size() > 0) && (pointdata.x / pointdata.y > 1.4) && (pointdata.x / pointdata.y < 2.3))
			{
				//vRlt[minlong].points(vertex);
				for (int i = 0; i < 4; i++)
				{
					line(frame, vertex[i % 4], vertex[(i + 1) % 4], Scalar(0, 0, 255), 2, CV_AA);
				}
				point.x = (vertex[2].x + vertex[0].x) / 2 - 160;
				point.y = (vertex[2].y + vertex[0].y) / 2 - 120;
				circle(frame, Point(point.x + 160, point.y + 120), 5, Scalar(0, 255, 255));
			}
		}
		
		/*if (slidfram.size() == 5)
			slidfram.erase(slidfram.begin());
		slidfram.push_back(point);
		int xxx = 0, yyy = 0;
		if (slidfram.size()>0)
		for (int i = 0; i < slidfram.size(); i++)
		{
			xxx += slidfram[i].x;
			yyy += slidfram[i].y;
		}
		xxx = xxx / slidfram.size();
		yyy = yyy / slidfram.size();*/
		time = (getTickCount() - t0) / getTickFrequency();
		cout << time << " fps" << endl;
		cout << "宽是：" << point.x << endl;
		cout << "高是：" << point.y<< endl;
		
		datas.push_back(30);
		datas.push_back(0);
		datas.push_back(time);
		vEllipse.clear();
		vRlt.clear();
		
		imshow("frame", frame);
		//imshow("2", rawImage);
		imshow("RLT", rlt);
		
		if (true == mySerialPort.WriteFloatData(datas))
		{
			cout << "all completed" << endl;
		}

		for (int i = 0; i < mySerialPort.recData_float.size(); i++)
		{
			std::cout << mySerialPort.recData_float[i] << "	,";
		}
		std::vector<float>().swap(mySerialPort.recData_float);//清理内存
		vector<float>().swap(datas);


		if (waitKey(1) == 27)
			break;
		

			}

			return 0;
		}
		
		

		
	
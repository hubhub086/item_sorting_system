#include <iostream>
#include <pthread.h>
#include <time.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

#include "uart.h"

using namespace std;
using namespace cv;

void getContours(Mat imgDil, Mat img);
Point get_center_position(Mat* img_origin, Rect roi);
vector<Point> get_map_pos(Mat* img, vector<Rect> rois);

Mat frame;
Mat gray, hsv_frame, orange_thres, black_thres, RGB_mask;
vector<Mat> hsvSplit;

vector<Rect> location_rois = {{277,5,60,60},
							  {0,0,60,60},
							  {0,0,60,60},
							  {0,0,60,60}};

int fd;
int main_status;  // modified in uart.cpp --> void* receive_thread(void* ptr)

int main()
{
    cout << "###  item_sorting_system  ###" << endl;
    VideoCapture capture(1, CAP_V4L2);
	if (!capture.isOpened())
	{
		cout << "camera not open!" << endl;
		exit(1);
	}
    // camera config
    capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(CAP_PROP_FRAME_WIDTH,640);
    capture.set(CAP_PROP_FRAME_HEIGHT,480);
    capture.set(CAP_PROP_FPS,60);
    capture.set(CAP_PROP_BUFFERSIZE, 1);
    capture.set(CAP_PROP_AUTO_WB, 0);
    capture.set(CAP_PROP_WB_TEMPERATURE, 4000);
    cout << "CAP_PROP_FPS" << capture.get(CAP_PROP_FPS) << endl;
    cout << "CAP_PROP_BUFFERSIZE=" << capture.get(CAP_PROP_BUFFERSIZE) << endl;
    cout << "CAP_PROP_AUTO_WB=" << capture.get(CAP_PROP_AUTO_WB) << endl;
	cout << "CAP_PROP_WB_TEMPERATURE=" << capture.get(CAP_PROP_WB_TEMPERATURE) << endl;
    cout << "CAP_PROP_FRAME_SIZE=(" << capture.get(CAP_PROP_FRAME_WIDTH) << "," << capture.get(CAP_PROP_FRAME_HEIGHT) << ")" << endl;

    //uart init
    fd = uart_init("/dev/ttyTHS1", 115200, 8, 'N', 1);
    uart_receiveThread_touch();
	uart_send(fd, "main init finish\n");

    clock_t start, end;
    while(1)
    {   
        start = clock();
        capture >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        cvtColor(frame, hsv_frame, COLOR_BGR2HSV); 

        ////颜色识别
        split(hsv_frame, hsvSplit);	
        equalizeHist(hsvSplit[2], hsvSplit[2]); 
        merge(hsvSplit, hsv_frame);	
        inRange(hsv_frame, Scalar(11, 43, 46), Scalar(140, 255, 255), orange_thres); //黄色
        inRange(hsv_frame, Scalar(35, 43, 36), Scalar(77, 255, 255), black_thres); //黑色
        // inRange(frame, Scalar(0, 0, 0), Scalar(100, 100, 100), RGB_mask); //黑色
        inRange(frame, Scalar(0, 150, 90), Scalar(20, 255, 180), RGB_mask); //绿色
        // Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
		// morphologyEx(orange_thres, orange_thres, MORPH_OPEN, element);  //开操作
		// morphologyEx(orange_thres, orange_thres, MORPH_CLOSE, element);  //闭操作

        ////形状识别
        // Mat imgBlur;
        // GaussianBlur(gray, imgBlur, Size(3,3), 3, 0);
        // Mat img_threshold;
        // threshold(gray, img_threshold, 0, 255, THRESH_BINARY | THRESH_OTSU);
        // Mat imgCanny;
        // Canny(img_threshold, imgCanny, 25, 75);
        // Mat imgDil;
        // Mat kernel = getStructuringElement(MORPH_RECT, Size(3,3));
        // dilate(imgCanny, imgDil, kernel);
        // getContours(imgDil, frame);

        end = clock();
        imshow("frame", frame);
        imshow("orange", orange_thres);
        imshow("black", black_thres);
        imshow("RGB_mask", RGB_mask);
        // cout << "fps=" << 1/(double(end-start)/CLOCKS_PER_SEC) << endl;
        int key = waitKey(1);
        if (key == 27)
        {
            break;
        }
    }

    return 0;
}

vector<Point> get_map_pos(Mat* img, vector<Rect> rois)
{
	vector<Point> rec_pos;
	for (int i=0; i<rois.size(); i++)
	{
		Point pos = get_center_position(img, rois[i]);
		rec_pos.push_back(pos);
	}
	return rec_pos;
}

Point get_center_position(Mat* img_origin, Rect roi)
{
    Mat img(*img_origin, roi);
    Point center_points = {0, 0}; 
	int count = 0;
    for (int col=0; col < img.cols; col++)
    {
        for (int row=0; row < img.rows; row++)
        {
            int R = img.at<uchar>(row, col);
            if (R < 2)
            {
                center_points.y += row;
                center_points.x += col;
                count++;
            }
        }
    }
	if (count > 0)
    {
        center_points.y = int(center_points.y / count) + roi.y;
        center_points.x = int(center_points.x / count) + roi.x;
        // center_points.push_back(KeyPoint);
    }
    return center_points;
}


void getContours(Mat imgDil,Mat img)
{
	//创建轮廓
	vector<vector<Point>> contours;
	//创建层级轮廓
	//Vec4i:每个向量具有4个整数值
	vector<Vec4i> hierarchy;
	//找到轮廓
	findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	//绘制轮廓
	//绘制所有的，符号为-1,颜色为紫色，厚度为2
	//drawContours(img, contours, -1,Scalar(255, 0, 255),10);
    //过滤噪声点
	for (int i = 0; i < contours.size(); i++)
	{
		//先找到每个轮廓的面积
		int area = contourArea(contours[i]);
		//打印出面积
		cout << area << endl;
		//定义轮廓对象的名称
		string objectType;
		//角点向量，仅存储轮廓中的角点
		vector<vector<Point>> conPoly(contours.size());
		//存储矩形边界框向量
		vector<Rect> boundRect(contours.size());
		//如果面积在可提取范围内，则提取对应轮廓
		if (area > 1000 && area < 4000)
		{
			//找到每个轮廓的周长
			float peri = arcLength(contours[i], true);
			//从轮廓中找到曲线的角点，true表示是否闭合
			//轮廓：contours[i]中包含所有的点
			//而角点：conPoly[i]中仅包含角点
			//如果是矩形，将有4个，三角形则是3个
			approxPolyDP(contours[i], conPoly[i], 0.02 * peri,true);
			//绘制所有的，符号为-1,颜色为紫色，厚度为2
			drawContours(img, conPoly, i, Scalar(255, 0, 255), 2);
			//打印出每个轮廓的角点数
			cout << conPoly[i].size() << endl; 
			//绘制矩形边界框，将含纳每一个独立的形状
			boundRect[i] = boundingRect(conPoly[i]);
			//将边界框打印在原图上
			rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 5);
			//定义每个轮廓的角点数
			int objCor = (int)conPoly[i].size();
			//三角形判断
			if (objCor == 3)
			{
				objectType = "Triangle";
			}
			//矩形判断
			if (objCor == 4)
			{
				//定义长宽比
				float aspectRatio = (float)boundRect[i].width / (float)boundRect[i].height;
				cout << "长宽比"<< aspectRatio << endl;
				//正方形判断
				if (aspectRatio > 0.95 && aspectRatio < 1.05)
				{
					objectType = "Square";
				}
				//不然即是矩形
				else
				{
					objectType = "Rectangle";
				}
			}
			//圆形判断
			if (objCor >4)
			{
				objectType = "Circle";
			}
			//在形状上方写出对应的名称
			putText(img, objectType, { boundRect[i].x,boundRect[i].y - 5 }, FONT_HERSHEY_DUPLEX, 0.75, Scalar(0, 69, 255), 2);
		}
	}
}

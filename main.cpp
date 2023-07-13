#include <iostream>
#include <fstream>
#include <pthread.h>
#include <time.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

#include "uart.h"

using namespace std;
using namespace cv;

void getContours(Mat imgDil, Mat img);
Point get_center_position(Mat* img_origin, Rect roi);
vector<Point> get_map_pos(Mat* img);

Mat frame;
Mat gray, hsv_frame, orange_thres, green_thres, RGB_mask;
Mat img_warp_hsv,img_wrap_BGR;
vector<Mat> hsvSplit;

vector<Rect> location_rois = {{277,5,60,60},
							  {0,0,60,60},
							  {0,0,60,60},
							  {0,0,60,60}};

int fd;
int main_status = 10;  // modified in uart.cpp --> void* receive_thread(void* ptr)
vector<Point2f> dst_points = {Point2f(0.0, 0.0),
							  Point2f(0.0, 500.0),
				              Point2f(400.0, 0.0),
				              Point2f(400.0, 500.0)};
vector<Point2f> map_pos_2f;

// [247, 92;
//  508, 84;
//  258, 415;
//  517, 409]

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
    capture.set(CAP_PROP_AUTO_WB, 1);
    // capture.set(CAP_PROP_WB_TEMPERATURE, 4000);
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
		cout << "### item sorting system  ###" << endl;
        start = clock();
        capture >> frame;
        cvtColor(frame, hsv_frame, COLOR_BGR2HSV); 
		split(hsv_frame, hsvSplit);	
        equalizeHist(hsvSplit[2], hsvSplit[2]); 
        merge(hsvSplit, hsv_frame);	

		cout << main_status << endl;

        ////颜色识别
        if (main_status == 10)
		{
			inRange(hsv_frame, Scalar(35, 43, 36), Scalar(77, 255, 255), green_thres); //绿色
			Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
			morphologyEx(green_thres, green_thres, MORPH_OPEN, element);  //开操作
			morphologyEx(green_thres, green_thres, MORPH_CLOSE, element);  //闭操作
			vector<Point> map_pos = get_map_pos(&green_thres);
			if (map_pos.size() == 4 && main_status == 10)
			{
				map_pos_2f.clear();
				for (int i=0; i<map_pos.size(); i++)
				{
					map_pos_2f.push_back(Point2f(map_pos[i].x, map_pos[i].y));
				}
				cout << map_pos << endl;
			}
			if (map_pos_2f.size() == 4)
			{
				Mat rotation,img_warp;
				rotation=getPerspectiveTransform(map_pos_2f,dst_points);
				warpPerspective(frame, img_warp, rotation, Size(400, 500));
				imshow("imgwrap", img_warp);
			}
			imshow("green", green_thres);
		}
		else if (main_status == 20)
		{
			if (map_pos_2f.size() == 4)
			{
				Mat rotation;
				rotation=getPerspectiveTransform(map_pos_2f,dst_points);
				warpPerspective(hsv_frame, img_warp_hsv, rotation, Size(400, 500));
				warpPerspective(frame, img_wrap_BGR, rotation, Size(400, 500));
				imshow("imgwraphsv", img_warp_hsv);
				imshow("imgwrapBGR", img_wrap_BGR);
			}

			inRange(img_warp_hsv, Scalar(11, 43, 46), Scalar(140, 255, 255), orange_thres); //黄色
			inRange(img_warp_hsv, Scalar(0, 0, 0), Scalar(200, 200, 30), RGB_mask); //黑色
			// inRange(frame, Scalar(90, 150, 0), Scalar(180, 255, 20), RGB_mask); //绿色
			imshow("orange", orange_thres);
			imshow("RGB_mask", RGB_mask);
		}
        // //形状识别
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
        
        cout << "fps=" << 1/(double(end-start)/CLOCKS_PER_SEC) << endl;
        int key = waitKey(1);
        if (key == 27)
        {
			fstream f;
			f.open("../map_pos.txt", ios::out);
			f << map_pos_2f << endl;
			f.close();
            break;
        }
    }

    return 0;
}

vector<Point> get_map_pos(Mat* img)
{
	vector<Point> rec_pos_sort;
	vector<Point> rec_pos;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(green_thres, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	for (int i = 0; i < contours.size(); i++)
	{
		Rect bound = boundingRect(contours[i]);
		Point center = {0,0};
		int count = 0;
		for (int j=0; j<bound.width; j++)
		{
			for (int k=0; k< bound.height; k++)
			{
				if (green_thres.at<uchar>(bound.y+k, bound.x+j) == 255)
				{
					center += {bound.x+j, bound.y+k};
					count++;
				}
			}
		}
		center.x /= count;
		center.y /= count;
		rec_pos.push_back(center);
		circle(frame, center, 3, Scalar(0, 255, 120), -1);
		// cout << "center=" << center << endl;
		// int area = contourArea(contours[i]);
	}
	if (rec_pos.size() == 4)
	{
		vector<int> temp;
		for (int i=0; i<rec_pos.size(); i++)
		{
			int xandy = rec_pos[i].x + rec_pos[i].y;
			temp.push_back(xandy);
		}
		int max_index = max_element(temp.begin(), temp.end()) - temp.begin();
		int min_index = min_element(temp.begin(), temp.end()) - temp.begin();
		rec_pos_sort.push_back(rec_pos[min_index]);
		rec_pos_sort.push_back(rec_pos[max_index]);
		int r1=-1, r2=-1;
		for (int i=0; i<4; i++)
		{
			if (i != min_index && i != max_index)
			{
				if (r1==-1)  r1 = i;
				else  r2 = i;
			}
		}
		if (rec_pos[r1].y < rec_pos[r2].y)
		{
			rec_pos_sort.push_back(rec_pos[r1]);
			rec_pos_sort.push_back(rec_pos[r2]);
		}
		else
		{
			rec_pos_sort.push_back(rec_pos[r2]);
			rec_pos_sort.push_back(rec_pos[r1]);
		}
		Point tempPoint = rec_pos_sort[1];
		rec_pos_sort[1] = rec_pos_sort[3];
		rec_pos_sort[3] = tempPoint;
	}
	return rec_pos_sort;
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
	cout << "find:" << contours.size() << endl;
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

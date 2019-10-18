
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;
double sum(Mat src)
{
   double counterw = 0;
   double counterz = 0;
   Mat_<uchar>::iterator it = src.begin<uchar>();
   Mat_<uchar>::iterator itend = src.end<uchar>();
   for (; it != itend; it++)
   {
      if ((*it) > 0)
         counterw += 1;
      if ((*it) == 0)
         counterz += 1;
   }
   double a = counterw * 1.0 / (counterz + counterw) * 100;
   return a;
}
int main()
{
   Mat huabu(1000, 1000, CV_8UC3, Scalar(0, 0, 0));
   line(huabu, Point(5, 0), Point(5, 950), Scalar(255, 255, 255), 1, LINE_AA);
   line(huabu, Point(5, 950), Point(950, 950), Scalar(255, 255, 255), 1, LINE_AA);
   VideoCapture capture("34.avi");
   int i = 15;
   float last = 0;
   while (1)
   {
      Mat s1;
      capture >> s1;
      if (s1.empty())
      {
         cout << "n=vedio is over";
         return 0;
      }
      Mat s2;
      cvtColor(s1, s2, COLOR_BGR2GRAY);

      //namedWindow("原图1", WINDOW_NORMAL);
      //imshow("原图1", s1);
      //namedWindow("灰度1", WINDOW_NORMAL);
      //imshow("灰度1", s2);
      //--------阈值化

      threshold(s2, s2, 253, 255, THRESH_BINARY);

      namedWindow("阈值1", WINDOW_NORMAL);
      imshow("阈值1", s2);
      double a = sum(s2);
      //double *p = &a;
      //vector<Point>g;
      cout << "图片s1白色占比：" << a << endl;
      //---------
      int h = 100;
      int w = 100;
      line(huabu, Point(i - 10, 950 - last*10), Point(i, 950 -a*10), Scalar(255, 0, 0), 1, LINE_AA);
      i += 10;
      last = a;
      namedWindow("画布");
      imshow("画布", huabu);
      waitKey(500);
   }
   return 0;
}

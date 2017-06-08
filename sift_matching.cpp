////#include "opencl_kernels_features2d.hpp"
//#include<def.hpp>
//#if METHOD_POINT_
////#if ~ORB_
//#include <iostream>
//#include <opencv2/core/opengl.hpp>
//#include "opencv2/opencv.hpp"
//#include "opencv2/core/core.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/cudaarithm.hpp"
//#include "opencv2/core/cuda.hpp"
//#include "opencv2/cudafilters.hpp"
//#include "opencv2/cudawarping.hpp"
//#include "opencv2/core.hpp"
//#include <opencv2/core/utility.hpp>
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/cudaimgproc.hpp"
//#include "opencv2/cudafeatures2d.hpp"
//#include "opencv2/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
//#include <string>
//#include <vector>
//#include <algorithm>
//#include <numeric>
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include <opencv2/core/opengl.hpp>
//#include <opencv2/cudacodec.hpp>
//enum  method_{
//    ORB_,SIFT_,SURF_,BM_
//};

//using namespace std;
//using namespace cv;
//using namespace cv::xfeatures2d;
//using namespace cv::cuda;

//void calCrossPoint(Mat &dst,int x0,int y0, int x1,int y1,int x2,int y2,int x3,int y3)
//{
//    int de=(x1-x0)*(y3-y2)+(y0-y1)*(x3-x2),de2=(y3-y2);
//    const int64 s = getTickCount();
//    Vec2i a;
//    if(de==0)
//        cout<< "parrellel points!"<<endl;
//    else if (de2==0)
//        cout<< "points in the same baseline!"<<endl;
//    else
//    {

//    int d=(y0-y1)*(y3-y2)*x0+y0*(y3-y2)*(x1-x0)+(y1-y0)*(y3-y2)*x2+(x2-x3)*(y1-y0)*y2;
//    a[0]=d/de;
//    a[1]=x2+(x3-x2)*(a[0]-y2)/de2;
//    printf("crosspoint at (%d,%d) ",a[1],a[0]);
//     const double t = (getTickCount() - s) / getTickFrequency();
//    cout << "calculating Time : " << t*1000000  << " us" << endl;
//    Point center = Point (a[1],a[0]);
//    int r = 5;
//    circle (dst,center,r,Scalar(255,0,0));
////	dst.at<Vec3b>(a[1],a[0])[0]=255;
////	dst.at<Vec3b>(a[1],a[0])[1]=0;
////	dst.at<Vec3b>(a[1],a[0])[2]=0;
//    //cout<<"hehe"<<dst.type()<<endl;
//    //cout<<"hehe"<<dst.elemSize1()<<endl;
//    //return a;

//    }
//}


//void houghlines(Mat& src)
//{
////	int max_iters=10,open_close_pos;
////	open_close_pos=12;
////    int n = open_close_pos - max_iters;
////    int an = n > 0 ? n : -n;
////    Mat element = getStructuringElement(0, Size(an*2+1, an*2+1), Point(an, an) );
////    if( n < 0 )
////        morphologyEx(src, src, MORPH_OPEN, element);
////    else
////        morphologyEx(src, src, MORPH_CLOSE, element);
////    imshow("Open/Close",src);
////    cv::pyrDown(src,src);

//    Mat dst, cdst;
//    Canny(src, dst, 50, 200, 3);
//  //  dst=-dst;
//    cv::cvtColor(dst, cdst, COLOR_GRAY2BGR);

//#if 0
//    vector<Vec2f> lines;
//    HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

//    for( size_t i = 0; i < lines.size(); i++ )
//    {
//        float rho = lines[i][0], theta = lines[i][1];
//        Point pt1, pt2;
//        double a = cos(theta), b = sin(theta);
//        double x0 = a*rho, y0 = b*rho;
//        pt1.x = cvRound(x0 + 1000*(-b));
//        pt1.y = cvRound(y0 + 1000*(a));
//        pt2.x = cvRound(x0 - 1000*(-b));
//        pt2.y = cvRound(y0 - 1000*(a));
//        line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
//    }
//#else
//    vector<Vec4i> lines;
//    HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
// //   cdst-=cdst;
//    for( size_t i = 0; i < lines.size(); i++ )
//    {
//        Vec4i l = lines[i];
//        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
//    }
//#endif


//    src=cdst.clone();
//  //  int qq=CV_8U;
//    imshow("source", cdst);
//    //imshow("detected lines", src);

//   // waitKey();
//}

//void orbmatching(Mat &image,Mat &img)
//{
////	houghlines(image);
////	houghlines(img);
////    cv::pyrDown(image,image);

////    cv::pyrDown(img,img);
//    Mat s_img=image.clone();
//    Mat s_img2=img.clone();
//    Mat mask(image.size(), CV_8UC1, cv::Scalar::all(1));
//    Mat mask2(img.size(), CV_8UC1, cv::Scalar::all(1));
//    mask(cv::Range(0, image.rows / 2), cv::Range(0, image.cols / 2)).setTo(cv::Scalar::all(0));

//    mask2(cv::Range(0, image.rows / 2), cv::Range(0, image.cols / 2)).setTo(cv::Scalar::all(0));
//   // Ptr<SIFT> sift = SIFT::create()
//    double start=getTickCount();
//    //calculate left
//    cv::Ptr<cv::ORB> orb = cv::ORB::create(1200, 1.2f, 8, 31, 0,
//                              2, 0, 31, 20);

//                       std::vector<cv::KeyPoint> keypoints;
//                       cv::Mat descriptors;
//                       orb->detectAndCompute(s_img, mask, keypoints, descriptors);
//                       double q=(getTickCount()-start)*1000.0/getTickFrequency();
//                        cout<<q<<" ms "<<endl;


////calculate r.



//                      std::vector<cv::KeyPoint> keypoints_gold;
//                      cv::Mat descriptors_gold;
//                      orb->detectAndCompute(s_img2, mask2, keypoints_gold, descriptors_gold);
//                       q=(getTickCount()-start)*1000.0/getTickFrequency();
//                       cout<<q<<" ms "<<endl;


//                      cv::BFMatcher matcher(cv::NORM_HAMMING);
//                      std::vector<cv::DMatch> matches;
//                      matcher.match(descriptors_gold,descriptors, matches);
//                      double e=(getTickCount()-start)*1000.0/getTickFrequency();
//                      cout<<e<<" ms "<<endl;
//                      Mat img_matches;
//                      drawMatches(img,keypoints_gold,image,keypoints,matches,img_matches);
//                      imshow("matches",img_matches);


//       //   int matchedCount = getMatchedPointsCount(keypoints_gold, keypoints, matches);
//        //  double matchedRatio = static_cast<double>(matchedCount) / keypoints.size();
//}

//void siftmatching(Mat &image,Mat &img)
//{
////    houghlines(image);
////    houghlines(img);

//    Mat s_img=image.clone();
//    Mat s_img2=img.clone();
//    Mat mask(image.size(), CV_8UC1, cv::Scalar::all(1));
//    Mat mask2(img.size(), CV_8UC1, cv::Scalar::all(1));
//    mask(cv::Range(0, image.rows / 2), cv::Range(0, image.cols / 2)).setTo(cv::Scalar::all(0));

//    mask2(cv::Range(0, image.rows / 2), cv::Range(0, image.cols / 2)).setTo(cv::Scalar::all(0));
//    Ptr<SIFT> sift = SIFT::create();
//    double start=getTickCount();
//    //calculate left


//                       std::vector<cv::KeyPoint> keypoints;
//                       cv::Mat descriptors;
//                       sift->detectAndCompute(s_img, mask, keypoints, descriptors);
//                       double q=(getTickCount()-start)*1000.0/getTickFrequency();
//                        cout<<q<<" ms! "<<endl;


////calculate r.


//                      std::vector<cv::KeyPoint> keypoints_gold;
//                      cv::Mat descriptors_gold;
//                      sift->detectAndCompute(s_img2, mask2, keypoints_gold, descriptors_gold);
//                       q=(getTickCount()-start)*1000.0/getTickFrequency();
//                       cout<<q<<" ms! "<<endl;


//                      cv::BFMatcher matcher(cv::NORM_L1);
//                      std::vector<cv::DMatch> matches;
//                      matcher.match(descriptors, descriptors_gold, matches);
//                      double e=(getTickCount()-start)*1000.0/getTickFrequency();
//                      cout<<e<<" ms! "<<endl;
//                      Mat img_matches;
//                      drawMatches(image,keypoints,s_img,keypoints_gold,matches,img_matches);
//                      imshow("matches",img_matches);


//     //    int matchedCount = getMatchedPointsCount(keypoints_gold, keypoints, matches);
//        //  double matchedRatio = static_cast<double>(matchedCount) / keypoints.size();
//}

//void surfmatching(Mat &image,Mat &img)
//{
//        Mat s_img=image.clone();
//        Mat s_img2=img.clone();
//        Mat mask(image.size(), CV_8UC1, cv::Scalar::all(1));
//        Mat mask2(img.size(), CV_8UC1, cv::Scalar::all(1));
//        mask(cv::Range(0, image.rows / 2), cv::Range(0, image.cols / 2)).setTo(cv::Scalar::all(0));
//        mask2(cv::Range(0, image.rows / 2), cv::Range(0, image.cols / 2)).setTo(cv::Scalar::all(0));
//        Ptr<SURF> sift = SURF::create();
//        double start=getTickCount();
//        //calculate left


//                           std::vector<cv::KeyPoint> keypoints;
//                           cv::Mat descriptors;
//                           sift->detectAndCompute(s_img, mask, keypoints, descriptors);
//                           double q=(getTickCount()-start)*1000.0/getTickFrequency();
//                            cout<<q<<" ms "<<endl;


//    //calculate r.


//                          std::vector<cv::KeyPoint> keypoints_gold;
//                          cv::Mat descriptors_gold;
//                          sift->detectAndCompute(s_img2, mask2, keypoints_gold, descriptors_gold);
//                           q=(getTickCount()-start)*1000.0/getTickFrequency();
//                           cout<<q<<" ms "<<endl;


//                          cv::BFMatcher matcher(cv::NORM_L1);
//                          std::vector<cv::DMatch> matches;
//                          matcher.match(descriptors, descriptors_gold, matches);
//                          double e=(getTickCount()-start)*1000.0/getTickFrequency();
//                          cout<<e<<" ms "<<endl;
//                          Mat img_matches;
//                          drawMatches(image,keypoints,s_img,keypoints_gold,matches,img_matches);
//                          imshow("matches",img_matches);
//}

//int main(int argc, char* argv[])
//{

//    const std::string fname="/home/cy/Documents/workspace/QT_Pros/build-SfM-Desktop_Qt_5_5_1_GCC_64bit-Debug/video.avi",rname="/home/cy/Documents/workspace/QT_Pros/build-SfM-Desktop_Qt_5_5_1_GCC_64bit-Debug/videor.avi";

//       Mat l_frame,r_frame;
//       int count=0;
//       Mat img1=l_frame, img2=r_frame;
//       cv::VideoCapture l_reader(fname);
//       cv::VideoCapture r_reader(rname);
//method_ m=method_(ORB_);
//cout<<m<<endl;
//       for (;;)
//       {
//           double start=getTickCount();
//           if (!l_reader.read(l_frame))
//               break;
//           if (!r_reader.read(r_frame))
//                       break;
//           cv::cvtColor(l_frame,l_frame,COLOR_BGRA2GRAY,1);
//          cv::cvtColor(r_frame,r_frame,COLOR_BGRA2GRAY,1);
//          imshow("left",l_frame);
//          imshow("Right",r_frame);
//          if(++count<5)continue;
//          switch(m){
//          case ORB_:
//              orbmatching(l_frame,r_frame);
//              break;
//          case SIFT_:
//              siftmatching(l_frame,r_frame);
//              break;
//          case SURF_:
//              surfmatching(l_frame,r_frame);
//              break;
////          case BM_:
////              blockmatching(l_frame,r_frame);
//          default:
//              break;
//          }
//             double q1=(getTickCount()-start)/getTickFrequency();
//             cout<<"Feature matching takes "<< q1<<"s"<<endl;
//            waitKey();
//           if (cv::waitKey(3) > 0)
//               break;
//       }

//    waitKey(0);
//    return 0;
//}

//#endif

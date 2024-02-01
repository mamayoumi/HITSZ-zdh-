#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/imgproc/imgproc_c.h>

using namespace std;

ros::Publisher vel_pub;

using namespace cv;

enum CameraState
{
    COMPUTER = 0,
    ZED,
    REALSENSE
};
CameraState state =  REALSENSE;


Mat frame_msg;
void rcvCameraCallBack(const sensor_msgs::Image::ConstPtr& img)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    frame_msg = cv_ptr->image;
}


/*
*rgb model input transform to hsi model
*input: input_img_ptr output_img_ptr
*return: none
*attention: input_img should be U8C3 
*           output_img should be U8C3
*           they two should be init befor calling 
*/
void rgb2hsi(Mat* input, Mat* output)
{
	for (int i = 0; i < (*input).rows; i++)
	{
		for (int j = 0; j < (*input).cols; j++)
		{
			double b = (*input).at<Vec3b>(i, j)[0] / 255.f,
				g = (*input).at<Vec3b>(i, j)[1] / 255.f,
				r = (*input).at<Vec3b>(i, j)[2] / 255.f;

			double num = 0.5 * (2 * r - g - b);
			double den = sqrt((r - g) * (r - g) + (r - b) * (g - b));
			if (den == 0)
			{
				(*output).at<Vec3b>(i, j)[0] = 0; 
			}
			else
			{
				(*output).at<Vec3b>(i, j)[0] = b <= g ? (uchar)255 * acos(num / den) / (2 * 3.14) : (uchar)255 * (1 - acos(num / den) / (2 * 3.14));
			}

			float sum = b + g + r;
			if (sum == 0)
			{
				(*output).at<Vec3b>(i, j)[1] = 0;
			}
			else
			{
				(*output).at<Vec3b>(i, j)[1] = (uchar)255 * (1 - 3 * min(min(b, g), r) / sum);
			}

			(*output).at<Vec3b>(i, j)[2] = (uchar)255 * sum / 3;

		}
	}
}


/*
*Trackbar_callback
*input: int track_value void* usrdata
*return none
*no need call
*/
void h_threshold_callback(int pos, void* usrdata)
{
	//cout << "h_min: " << pos << endl;

}

void s_threshold_callback(int pos, void* usrdata)
{
	//cout << "s_min: " << pos << endl;

}

void i_threshold_callback(int pos, void* usrdata)
{
	//cout << "i_min: " << pos << endl;

}

void H_threshold_callback(int pos, void* usrdata)
{
	//cout << "h_max: " << pos << endl;

}

void S_threshold_callback(int pos, void* usrdata)
{
	//cout << "s_max: " << pos << endl;

}

void I_threshold_callback(int pos, void* usrdata)
{
	//cout << "i_max: " << pos << endl;

}



/*
*threshold hsi model to two_value img
*input: h_threshold_min ,h_threshold_max ,s_threshold_min ,s_threshold_max ,i_threshold_min ,i_threshold_max ,input_img_ptr ,output_img_ptr
*return; none
*attention: hsi threshold are produced by trackbar
*			input_img should be hsi model
*			output_img should be U8C1 and zero_init before calling
*/
void threshold(int h_min, int h_max, int s_min, int s_max, int i_min, int i_max, Mat* input, Mat* output)
{
	for (int i = 0; i < (*input).rows; i++)
	{
		for (int j = 0; j < (*input).cols; j++)
		{
			if (
				(*input).at<Vec3b>(i, j)[0] <= (h_max * 255.0 / 360.0) && (*input).at<Vec3b>(i, j)[0] >=(h_min * 255.0 / 360.0)
				&& (*input).at<Vec3b>(i, j)[1] <= s_max && (*input).at<Vec3b>(i, j)[1] >= s_min
				&& (*input).at<Vec3b>(i, j)[2] <= i_max && (*input).at<Vec3b>(i, j)[2] >=i_min

				)
			{
				(*output).at<uchar>(i, j) = 255;
			}
			else
			{
				(*output).at<uchar>(i, j) = 0;
			}


		}
	}
}



/*
*dilate img
*input: input_img ,output_img
*return; none
*attention: input_img better be two_value img or it will dilate img by threshold 127
*			output_img should be U8C1 and zero_init before calling
*			dilate is by eight_neighbour rule
*/
void Dilate(Mat input, Mat output)
{
	for (int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
			if (i == 0 || j == 0 || i == input.rows - 1 || j == input.cols - 1)
			{
				if ((int)input.at<uchar>(i, j) > 127)
				{
					output.at<uchar>(i, j) = 255;
				}
				else
				{
					output.at<uchar>(i, j) = 0;
				}
			}
			else
			{
				if (
					(int)input.at<uchar>(i, j) > 127 ||
					(int)input.at<uchar>(i, j + 1) > 127 ||
					(int)input.at<uchar>(i + 1, j) > 127 ||
					(int)input.at<uchar>(i - 1, j) > 127 ||
					(int)input.at<uchar>(i, j - 1) > 127||
					(int)input.at<uchar>(i-1, j - 1)>127||
					(int)input.at<uchar>(i-1, j + 1)>127||
					(int)input.at<uchar>(i+1, j - 1)>127||
					(int)input.at<uchar>(i+1, j + 1)>127
					)
				{

					output.at<uchar>(i, j) = 255;
				}
				else
				{

					output.at<uchar>(i, j) = 0;
				}
			}
		}
	}
}




/*
*erode img
*input: input_img ,output_img
*return; none
*attention: input_img better be two_value img or it will erode img by threshold 127
*			output_img should be U8C1 and zero_init before calling
*			erode is by four_neighbour rule
*/
void Erode(Mat input, Mat output)
{
	for (int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
			if (i == 0 || j == 0 || i == input.rows - 1 || j == input.cols - 1)
			{
				if ((int)input.at<uchar>(i, j) > 127)
				{
					output.at<uchar>(i, j) = 255;
				}
				else
				{
					output.at<uchar>(i, j) = 0;
				}
			}
			else
			{
				if (
					(int)input.at<uchar>(i, j) > 127 &&
					(int)input.at<uchar>(i, j + 1) > 127 &&
					(int)input.at<uchar>(i + 1, j) > 127 &&
					(int)input.at<uchar>(i - 1, j) > 127 &&
					(int)input.at<uchar>(i, j - 1) > 127
					)
				{

					output.at<uchar>(i, j) = 255;
				}
				else
				{

					output.at<uchar>(i, j) = 0;
				}
			}
		}
	}
}





/*
*track_growing by seed_growing_thinking
*input: input_img_ptr ,output_img_ptr ,growing_dis_threshold
*return; double mid in angle_decision_row
*attention: input_img should be two_value img 
*			output_img should be U8C1 and zero_init before calling
*			first ergodic input_img last tenth  row to find left_begin and right_begin
*			then growing from two begin in two tracks
*			stop at input_img 0.7 rows
*/
double seed_growing(Mat* input, Mat *output, int dis_threshold)
{
	
	int grow_start = 0;
	int left=0;
	int right=640;
	for (int j = (*input).cols / 2; j > 1; j--)
	{
		if (abs((*input).at<uchar>((*input).rows - 10, j) - (*input).at<uchar>((*input).rows - 10, j - 1)) > dis_threshold)
		{
			circle((*output), Point(j, (*input).rows - 10), 1, Scalar(255, 255, 255), 1);
			grow_start = j;
			break;
			//imshow("3", (*input));

		}
	}
	for (int i = (*input).rows - 11; i > 0.7 * (*input).rows; i--)
	{
		if ((*input).at<uchar>(i, grow_start) < 127)
		{
			for (int j = grow_start - 1; j > 0;j--)
			{
				if ((*input).at<uchar>(i, j) > 127)
				{
					grow_start = j;
					if(i==(*input).rows - 20)
					{
						left=j;
					}
					circle((*output), Point(j, i), 1, Scalar(255, 255, 255), 1);
					break;
				}
			}
		}
		else
		{
			for (int j = grow_start + 1; j < 0.5* (*input).cols ; j++)
			{
				if ((*input).at<uchar>(i, j) > 127)
				{
					grow_start = j;
					if(i==(*input).rows - 20)
					{
						left=j;
					}
					circle((*output), Point(j, i), 1, Scalar(255, 255, 255), 1);
					break;
				}
			}
		}
	}

	int grow_start1 = 0;
	for (int j = (*input).cols / 2; j < (*input).cols ; j++)
	{
		if (abs((*input).at<uchar>((*input).rows - 10, j) - (*input).at<uchar>((*input).rows - 10, j + 1)) > dis_threshold)
		{
			circle((*output), Point(j, (*input).rows - 10), 1, Scalar(255, 255, 255), 1);
			grow_start1 = j;
			break;
			//imshow("3", (*input));

		}
	}
	for (int i = (*input).rows - 11; i > 0.7 * (*input).rows; i--)
	{
		if ((*input).at<uchar>(i, grow_start1) < 127)
		{
			for (int j = grow_start1 + 1; j < (*input).cols; j++)
			{
				if ((*input).at<uchar>(i, j) > 127)
				{
					grow_start1 = j;
					if(i==(*input).rows - 20)
					{
						right=j;
					}
					circle((*output), Point(j, i), 1, Scalar(255, 255, 255), 1);
					break;
				}
			}
		}
		else
		{
			for (int j = grow_start1 - 1; j > 0.5 * (*input).cols; j--)
			{
				if ((*input).at<uchar>(i, j) > 127)
				{
					grow_start1 = j;
					if(i==(*input).rows - 20)
					{
						right=j;
					}
					circle((*output), Point(j, i), 1, Scalar(255, 255, 255), 1);
					break;
				}
			}
		}
	}
	return 0.5*left+0.5*right;
	
}




/*
*find the mid distance from bottom
*input: input_img_ptr
*return: the mid distance from bottom
*attention: if mid 200 pixel have 100 white , return
*/
int swp(Mat* input)
{
	for (int i = (*input).rows - 1; i > 0; i--)
	{
		int count = 0;
		for (int j = 0.5*(*input).cols - 100; j < 0.5*(*input).cols + 100; j++)
		{
			if ((*input).at<uchar>(i, j) > 127)
			{
				count = count + 1;
			}
		}
		if (count > 100)
		{
			return i;
		}
	}
	return 0;

}



/*
* find all the counter and choose square between min_threshold and max_threshold counter calculate external rectangle rotation angle
*input: input_img
*return: all the counter which square between min_threshold and max_threshold external rectangle rotation angle
*atteention: input should be right roi May God bless you
*
*/
double findAndDrawBlobs(Mat& image) {
	vector<std::vector<Point>> contours;
	findContours(image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	
	double threshold1 = 5.0;
	double threshold2 = 50.0;
	double angle=60;

	vector<Point> contourPoints;
	vector<pair<double, double>> data;

	for (int i = 0; i < contours.size(); i++)
	{

		double area = contourArea(contours[i]);


		cout << area << endl;
		if (area > threshold1 && area < threshold2)
		{

			contourPoints.insert(contourPoints.end(), contours[i].begin(), contours[i].end());
			Moments m = moments(contours[i]);
			
			data.push_back({ m.m10 / m.m00,m.m01 / m.m00 });
			 
		}
	}


	double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
	int n = data.size();
	if (n !=0)
	{


		for (int i = 0; i < n; i++) {
			sum_x += data[i].first;
			sum_y += data[i].second;
			sum_xy += data[i].first * data[i].second;
			sum_x2 += data[i].first * data[i].first;
		}
	
	double x_mean = sum_x / n;
	double y_mean = sum_y / n;


	double a = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);
	cout << a << endl;



	RotatedRect minRect = minAreaRect(contourPoints);


	 
	double angle= minRect.angle;

	cout << "Rotation angle: " << angle << endl;
	}
	return angle;
}





int main(int argc, char ** argv)
{
    ros::init(argc, argv, "car_demo");
    ros::NodeHandle n;
    ros::Subscriber camera_sub;
	VideoCapture capture;
	 
   int h_min = 272;
   int s_min = 52;
   int i_min = 25;
   int h_max = 360;
   int s_max = 255;
   int i_max = 255;

	//create trackbar to dicede hsi threshold
   namedWindow("thresholdbar");
   Mat bar = Mat::zeros(480, 640, CV_8UC1);
   imshow("thresholdbar", bar);
   createTrackbar("h_min", "thresholdbar", &h_min, 360, h_threshold_callback, 0);
   createTrackbar("h_max", "thresholdbar", &h_max, 360, H_threshold_callback, 0);
   createTrackbar("s_min", "thresholdbar", &s_min, 255, s_threshold_callback, 0);
   createTrackbar("s_max", "thresholdbar", &s_max, 255, S_threshold_callback, 0);
   createTrackbar("i_min", "thresholdbar", &i_min, 255, i_threshold_callback, 0);
   createTrackbar("i_max", "thresholdbar", &i_max, 255, I_threshold_callback, 0);
  

   
   bool turn = false; // whethear decide the final direction
   bool left = false; // final direction is left or not
	double zeta=0;  //angle 
	double M=0;     //mid 
    bool at=false;   // some important mark to decide wheather fast run after fianl decision


    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
   
    if(state == COMPUTER)
    {
        capture.open(0);     
        if (!capture.isOpened())
        {
            printf("电脑摄像头没有正常打开\n");
            return 0;
        }
        waitKey(1000);
    }
    else if(state == ZED)
    {
        capture.open(2);     
        if (!capture.isOpened())
        {
            printf("ZED摄像头没有正常打开\n");
            return 0;
        }
        waitKey(1000);
    }
    else if(state == REALSENSE)
    {
        camera_sub = n.subscribe("/camera/color/image_raw",1,rcvCameraCallBack);
		waitKey(1000);
	}

	Mat frame;//当前帧图片
    ros::Rate loop_rate(10); // 设置循环频率为10Hz
    geometry_msgs::Twist vel_msg;
    int width=capture.get(CAP_PROP_FRAME_WIDTH);
    int height=capture.get(CAP_PROP_FRAME_HEIGHT);

	//main while 
    while (ros::ok())
    {

        if(state == COMPUTER)
        {
            capture.read(frame);
            if (frame.empty())
            {
                printf("没有获取到电脑图像\n");
                continue;
            }
        }
        else if(state == ZED)
        {
            capture.read(frame);
            if (frame.empty())
            {
                printf("没有获取到ZED图像\n");
                continue;
            }
            frame = frame(cv::Rect(0,0,frame.cols/2,frame.rows));//截取zed的左目图片
        }
        else if(state == REALSENSE)
        {
            if(frame_msg.cols == 0)
            {
                printf("没有获取到realsense图像\n");
                ros::spinOnce();
                continue;
            }
			frame = frame_msg;
        }


	
       
				
			resize(frame, frame, Size(640, 480), 0, 0, INTER_LINEAR);  // resize standard size
			imwrite("/home/eaibot/dip_ws/src/exp4/src/1.jpg",frame);   // save img to check camera 
			
			
			Mat img=Mat::zeros(frame.rows , frame.cols , CV_8UC1);  //init track img
			
			Mat pro_img1 = Mat::zeros(frame.rows, frame.cols, CV_8UC3);  //init hsi img

			rgb2hsi(&frame, &pro_img1);

			Mat pro_img2 = Mat::zeros(frame.rows, frame.cols, CV_8UC1);  //init hsi_threshold img

			threshold(h_min, h_max, s_min, s_max, i_min, i_max, &pro_img1, &pro_img2);


			if (turn == false)   //when final decision is not made
			{
			M=seed_growing(&pro_img2, &img, 100);  //produce track
			zeta=-0.001*(M-320);

			vel_msg.linear.x = 0.5;
			
			if(zeta>0.2)
			{
			zeta=0.2;
			
			}
			if(zeta<-0.2)
			{
			zeta=-0.2;
			}
		
			cout<<zeta<<endl;  // easy P control
        	vel_msg.angular.z = zeta;
        
        	vel_pub.publish(vel_msg);
			
				if (swp(&pro_img2)>320)   //mid is close prepare for final dicede
				{
				
					  
					  imwrite("/home/eaibot/dip_ws/src/exp4/src/1.jpg",frame);  // save img for checking
					 
				
				//dilate six times to large feture
				Mat clo = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
				for (int i = 0; i < 3; i++)
				{
					Dilate(pro_img2, clo);
					Dilate(clo, pro_img2);

				}
			    // find roi bottom
				int end = 0;
				for (int i = pro_img2.rows - 1; i > 0; i--)
				{
					if (pro_img2.at<uchar>(i, 0.5 * pro_img2.cols) > 127)
					{
						end = i;
						break;
					}
				}
				if (end < 200)
				{
					end = 200;
				}
				//提取中部roi 
				Rect rect = Rect(0.5 * pro_img2.cols - 100, end - 200, 200, 200);
				Mat ROI = pro_img2(rect);
				Mat clo2 = Mat::zeros(ROI.rows, ROI.cols, CV_8UC1);
				//dilate six times to large feture
				for (int i = 0; i < 3; i++)
				{
					Dilate(ROI, clo2);
					Dilate(clo2, ROI);

				}
				//删除顶端
				int start = 0;
				for (int i = 0; i < ROI.rows; i++)
				{
					int count = 0;
					for (int j = 0; j < ROI.cols; j++)
					{
						if (ROI.at<uchar>(i, j) > 127)
						{
							count++;

						}

					}
					if (count > 120)
					{
						start = i;
						break;
					}
				}
				//再提取roi
				Rect rect2 = Rect(0, start, 200, 200 - start);
				Mat roi = ROI(rect2);


				//dilate six times to large feture
				Mat clo3 = Mat::zeros(roi.rows, roi.cols, CV_8UC1);
				for (int i = 0; i < 3; i++)
				{
					Dilate(roi, clo3);
					Dilate(clo3, roi);

				}
				//提取left roi
				int begin = 0;
				for (int j = 3; j < roi.cols - 3; j++)
				{
					int cnt = 0;
					for (int i = 0; i < roi.rows; i++)
					{
						if (roi.at<uchar>(i, j) > 127)
						{
							cnt++;
						}
					}
					if (cnt < 0.9 * roi.rows)
					{
						begin = j;
						break;
					}
				}
				//提取right roi
				int stop = 0;
				for (int j = roi.cols - 3; j > 3; j--)
				{
					int cnt = 0;
					for (int i = 0; i < roi.rows; i++)
					{
						if (roi.at<uchar>(i, j) > 127)
						{
							cnt++;
						}
					}
					if (cnt < 0.9 * roi.rows)
					{
						stop = j;
						break;
					}
				}

				//最终匹配
				Rect rect1 = Rect(0.5 * pro_img2.cols - 100 + begin - 20, end - 200 + start, stop - begin + 40, 200 - start);
				ROI = frame(rect1);
				  imwrite("/home/eaibot/dip_ws/src/exp4/src/ROI.jpg",ROI); 
				Mat ROII = Mat::zeros(ROI.rows, ROI.cols, CV_8UC3);

				rgb2hsi(&ROI, &ROII);
				roi = Mat::zeros(ROI.rows, ROI.cols, CV_8UC1);
				threshold(166, 289, 42, 255, 0, 255, &ROII, &roi);
				imwrite("/home/eaibot/dip_ws/src/exp4/src/roi.jpg",roi); 
				imshow("1", roi);
				double ang=findAndDrawBlobs(roi);
				cout<<ang<<endl;
					 //根据外接矩形旋转角度判断左右
					if( abs(ang)>20&&abs(ang)<70)
					{
					turn=true;
					cout<<"right"<<endl;
					
					}
					else
					{
					turn=true;
					left=true;
					cout<<"left"<<endl;
					}
					
					
				}
	  
			}	
			
			if(turn&&left)
			{
			int l=0;
			int r=640;
			bool lf=false;
			bool rf=false;
			for(int j=320;j>0;j--)
			{
			if(pro_img2.at<uchar>(449,j)>127)
			{
			l=j;
			lf=true;
			break;
			}
			}
			for(int j=320;j<640;j++)
			{
			if(pro_img2.at<uchar>(449,j)>127)
			{
			r=j;
			rf=true;
			break;
			}
			}
			//缓速旋转到正方向
			if(!lf&&!rf&&!at)
			{
			vel_msg.linear.x = 0.1;
			double zeta=0.3;
			vel_msg.angular.z = zeta;
			
        		vel_pub.publish(vel_msg);
        		
			}
			else{
			at=true;
			double zeta=-0.001*(l+r-640);  //p control
			vel_msg.linear.x = 0.5;
			
			vel_msg.angular.z = zeta;
			
        		vel_pub.publish(vel_msg);
			}
        		
			}
			
			
			if(turn&&!left)
			{
			int l=0;
			int r=640;
			bool lf=false;
			bool rf=false;
			for(int j=320;j>0;j--)
			{
			if(pro_img2.at<uchar>(449,j)>127)
			{
			l=j;
			lf=true;
			break;
			}
			}
			for(int j=320;j<640;j++)
			{
			if(pro_img2.at<uchar>(449,j)>127)
			{
			r=j;
			rf=true;
			break;
			}
			}
			//缓速旋转到正方向
			if(!lf&&!rf&&!at)
			{
			vel_msg.linear.x = 0.1;
			double zeta=-0.3;
			vel_msg.angular.z = zeta;
			
        		vel_pub.publish(vel_msg);
        		
			}
			else{
			at=true;
			double zeta=-0.001*(l+r-640);
			vel_msg.linear.x = 0.5;
			
			vel_msg.angular.z = zeta;
			
        		vel_pub.publish(vel_msg);
			}
        		
			}
			

			imshow("after", pro_img2);
			imshow("seed", img);
			char c = waitKey(1);

        ros::spinOnce(); // 处理回调函数
        waitKey(1);
        loop_rate.sleep(); // 控制循环速率
    }

}

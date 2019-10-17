
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

int main(int argc, char *argv[])
{
	std::cout << "create or detect[c or d]:";
	char cod;
	std::cin >> cod;
	if (cod == 'c')
	{
		cv::Mat a;
		cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
		cv::aruco::drawMarker(dict, 20, 500, a);
		cv::imshow("a", a);
		cv::waitKey(0);
		cv::imwrite("7X7_100_20_500x500.jpg", a);
		return 0;
	}
	else if (cod == 'd')
	{
		cv::Mat a;
		cv::Mat rvec;
		cv::Mat tvec;
		cv::Mat cameraMatrix, distCoeffs;
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		std::vector<cv::Vec3d> rvecs, tvecs;
		cameraMatrix = (cv::Mat_<double>(3, 3) << 1441.792886, 0.000000, 314.125541,
						0.000000, 1441.180901, 236.418704,
						0.000000, 0.000000, 1.000000);
		distCoeffs = (cv::Mat_<double>(5, 1) << -0.653561, 0.997471, 0.005317, 0.003278, 0.000000);
		// cameraMatrix = (cv::Mat_<double>(3, 3) << 1128.048344, 0, 339.421769, 0, 1127.052190, 236.535242, 0, 0, 1);
		// distCoeffs = (cv::Mat_<double>(5, 1) << -0.568429, 0.514592, -0.000126, 0.000500, 0.00000);
		cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
		cv::VideoCapture cap(1);
		while (cap.grab())
		{
			cap >> a;
			cv::aruco::detectMarkers(a, dict, corners, ids);
			if (ids.size() > 0)
			{
				std::string depth;
				std::stringstream ss;

				cv::aruco::drawDetectedMarkers(a, corners, ids);
				std::vector<cv::Point3f> world_cor;
				world_cor.push_back(cv::Point3f(-0.066, 0.066, 0));
				world_cor.push_back(cv::Point3f(0.066, 0.066, 0));
				world_cor.push_back(cv::Point3f(0.066, -0.066, 0));
				world_cor.push_back(cv::Point3f(-0.066, -0.066, 0));
				cv::aruco::estimatePoseSingleMarkers(corners, 0.132, cameraMatrix, distCoeffs, rvecs, tvecs);
				
				for (int i = 0; i < ids.size(); i++)
				{
					cv::aruco::drawAxis(a, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.066);
					cv::solvePnP(world_cor, corners[i], cameraMatrix, distCoeffs, rvecs[i], tvecs[i],false, cv::SOLVEPNP_IPPE_SQUARE);
					double theta_x, theta_y, theta_z;
					cv::Mat rvecM0;
					Eigen::Matrix3d rvecM(3, 3);
					cv::Rodrigues(rvecs[i], rvecM0);
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++)
						{
							rvecM(i, j) = rvecM0.at<double>(i, j);
						}
					double theta = std::atan2(rvecM(2, 1), rvecM(2, 2));
					// theta_x = std::atan2(rvecM(2, 1), rvecM(2, 2));
					// if(theta_x < 0) //求出三个旋转角//
					// theta_y = std::atan2(-rvecM(2, 0),
					// 					  sqrt(rvecM(2, 1) * rvecM(2, 1) + rvecM(2, 2) * rvecM(2, 2)));
					// theta_z = std::atan2(rvecM(1, 0), rvecM(0, 0));
					theta_x = 3.14 - std::atan2(rvecM(2, 1), rvecM(2, 2)); //求出三个旋转角//
					theta_y = -std::atan2(-rvecM(2, 0),
										  sqrt(rvecM(2, 1) * rvecM(2, 1) + rvecM(2, 2) * rvecM(2, 2)));
					theta_z = -std::atan2(rvecM(1, 0), rvecM(0, 0));
					Eigen::Matrix3d m;
					m = Eigen::AngleAxisd(theta_x, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(theta_y, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(theta_z, Eigen::Vector3d::UnitZ());
					Eigen::Vector3d n(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
					Eigen::Vector3d l = m * n;
					ss.clear();
					ss << l[2];
					ss >> depth;
					std::cout << depth << std::endl;
					depth = depth.substr(0, 5);
					cv::putText(a, "depth: " + depth + "m", corners[i][0], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255, 255, 255));
					ss.clear();
					ss << l[1];
					ss >> depth;
					std::cout << depth << std::endl;
					depth = depth.substr(0, 5);
					cv::putText(a, "y: " + depth + "m", corners[i][1], cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255, 255, 255));
					ss.clear();
					ss << l[0];
					ss >> depth;
					std::cout << depth << std::endl;
					depth = depth.substr(0, 5);
					cv::putText(a, "x: " + depth + "m", corners[i][2], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 255));
				}
				if (ids.size() >= 2)
				{
					cv::Point2f center1((corners[0][0].x + corners[0][1].x) / 2, (corners[0][1].y + corners[0][2].y) / 2);
					cv::Point2f center2((corners[1][0].x + corners[1][1].x) / 2, (corners[1][1].y + corners[1][2].y) / 2);
					cv::line(a, center1, center2, cv::Scalar(100, 100, 100));
				}
			}
			cv::imshow("a", a);
			cv::waitKey(3);
		}

		return 0;
	}
}

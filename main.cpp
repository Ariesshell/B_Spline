
#include <opencv.hpp>
#include <string.h>
#include <vector>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace cv;

void GenerateNodeVector(int n, int k, vector<float>& nodeVector, bool uniform = false)
{
	if (uniform)
	{
		for (size_t i = 0; i < n+k+1; i++)
		{
			nodeVector.push_back(i*(1.0f) / (n + k));
		}
	}
	else
	{
		int piecewise = n - k - 1;
		if (piecewise == 1)
		{
			nodeVector.assign(n + 2, 0.0f);
			for (int i = n; i < n + k + 1; i++) {
				nodeVector.push_back(1.f);
			}
		}
		else
		{
			nodeVector.assign(k + 1, 0.0f);
			int flag = 0;
			float value = 0.0f;
			while (flag != piecewise) {
				value += 1.0f / piecewise;
				nodeVector.push_back(value);
				flag++;
			}
			for (int i = 0; i < k + 1; i++)
			{
				nodeVector.push_back(1.f);
			}
		}
	}

}

float BaseFunction(int idx, int k, float u, vector<float>& nodeVector)
{
	float Nik_u;
	if (k == 0) {
		if ((u >= nodeVector[idx]) && (u < nodeVector[idx + 1]))
		{
			Nik_u = 1.0f;
		}
		else
		{
			Nik_u = 0.0f;
		}
	}
	else {
		float Length1 = nodeVector[idx + k] - nodeVector[idx];
		float Length2 = nodeVector[idx + k + 1] - nodeVector[idx + 1];
		float alpha;
		float beta;
		if (Length1 == 0.0f)
		{
			alpha = 0.0f;
		}
		else
		{
			alpha = (u - nodeVector[idx]) / Length1;

		}
		if (Length2 == 0.0f)
		{
			beta = 0.0f;
		}
		else
		{
			beta = (nodeVector[idx + k + 1] - u) / Length2;
		}
		Nik_u = alpha * BaseFunction(idx, k - 1, u, nodeVector)
			+ beta * BaseFunction(idx + 1, k - 1, u, nodeVector);
	}
	return Nik_u;
}

int GenerateSplineCurve(int num, int k, vector<Point>& ctrlPoints, vector<Point>& bSplineCurve)
{
	int count = ctrlPoints.size();
	vector<float> nodeVector;
	GenerateNodeVector(count, k, nodeVector);

	float u = .0f / num;
	for (size_t i = 0; i < num; i++)
	{
		Point interPoint = { static_cast<long>(0), static_cast<long>(0) };
		for (int j = 0; j < count; j++)
		{
			float base = BaseFunction(j, k, u, nodeVector);
			interPoint.x += base * ctrlPoints[j].x;
			interPoint.y += base * ctrlPoints[j].y;
		}
		bSplineCurve.push_back(interPoint);
		u += 1.0f / num;
	}

	return 0;
}

int GenerateControlPoints(int num, vector<Point>& cPoints, int k, bool closed = false){
	const int width = 600;
	const int height = 600;
	Point p;
	srand(static_cast<unsigned>(time(0)));
	for (size_t i = 0; i < num; i++)
	{
		int x = static_cast<int>(rand()) / (static_cast<int>(RAND_MAX / width));
		int y = static_cast<int>(rand()) / (static_cast<int>(RAND_MAX / height));
		p = Point(x, y);
		cPoints.push_back(p);
	}
	if (closed)
	{
		if (num < k)
		{
			return -1;
		}
		for (size_t i = 0; i < k; i++)
		{
			cPoints.push_back(cPoints[i]);
		}
		
	}
	return 0;
}

int main() {
	const int pointNum = 10;
	const int k = 6;
	const int interpNum = 140;
	vector<Point> cPoints;
	GenerateControlPoints(pointNum, cPoints, k);
    cv::Mat cvimage = cv::Mat(600, 600, CV_8UC3);
	for (size_t i = 0; i < cPoints.size(); i++)
	{
		cv::circle(cvimage, cPoints[i], 3.5, cv::Scalar(200, 0, 0));
		cv::putText(cvimage, std::to_string(i), cPoints[i], cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
		//cv::imwrite(path, cvimage);
	}
	vector<Point> splinePoints;
	GenerateSplineCurve(interpNum, k, cPoints, splinePoints);

	for (size_t i = 0; i < splinePoints.size(); i++)
	{ 
		cv::circle(cvimage, splinePoints[i], 5, cv::Scalar(0, 0, 255));
		cv::putText(cvimage, std::to_string(i), splinePoints[i], cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(23, 126, 0), 1, cv::LINE_AA);
	}
	vector<vector<Point>> contour;
	contour.push_back(splinePoints);
	//cv::drawContours( mage, contour, -1, Scalar(0, 255, 155),1);
	imshow("spline", cvimage);
	waitKey(0);
	return 0;
}
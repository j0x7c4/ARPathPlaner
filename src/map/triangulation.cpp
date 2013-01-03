#include <cv.h>
#include <highgui.h>
#include "PPMap.h"
using namespace std;
using namespace cv;

ppMap ppmap;

void onMouse(int event,int x,int y,int flag,void* param){
	if ( event == CV_EVENT_LBUTTONUP ) {
		printf("%d %d\n",x,y);
	}
}

int main(int argc,char** argv[])
{
	ppPoint points[12] = { ppPoint(200,10), ppPoint(550,300), ppPoint(200,550), ppPoint(10,200) ,
													ppPoint(190,200),ppPoint(220,200), ppPoint(220,230),ppPoint(186,210),
												ppPoint(240,200),ppPoint(270,200), ppPoint(270,230),ppPoint(236,210)};

	vector<ppPoint> border(points,points+4);
	vector<vector<ppPoint>> obstacles;
	obstacles.push_back(vector<ppPoint>(points+4,points+8));
	obstacles.push_back(vector<ppPoint>(points+8,points+12));
	ppmap.createBorder(border);
	ppmap.createObstacles(obstacles);
	ppmap.init();
	ppmap.createMap();
	Mat image = ppmap.getImage();
	imshow("Delaunay",image);
	waitKey(0);

	cvDestroyWindow("Delaunay");

	return 0;
}
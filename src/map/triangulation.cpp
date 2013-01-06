#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "PPMap.h"

using namespace std;
using namespace cv;

ppMap ppmap(800,600);


int main(int argc,char** argv)
{
	ppPoint points[12] = { ppPoint(200,10), ppPoint(550,300), ppPoint(200,550), ppPoint(10,200) ,
													ppPoint(190,200),ppPoint(220,200), ppPoint(220,230),ppPoint(186,210),
												ppPoint(240,200),ppPoint(270,200), ppPoint(270,230),ppPoint(236,210)};

	vector<ppPoint> border(points,points+4);
	vector<vector<ppPoint> > obstacles;
	obstacles.push_back(vector<ppPoint>(points+4,points+8));
	obstacles.push_back(vector<ppPoint>(points+8,points+12));
	ppmap.createBorder(border);
	ppmap.createObstacles(obstacles);
	ppmap.init();
	ppmap.createMap();
	//test all vertex of blocks is clockwise
	for ( int i=0 ; i<ppmap.blocks.size(); i++ ) {
		if ( !isInRegion(ppmap.blocks[i].points,ppmap.blocks[i].center) ) {
			printf("%d NOOOO!\n",i);
			printf("%d,%d\n",ppmap.blocks[i].center.x,ppmap.blocks[i].center.y);
			for ( int j=0 ; j<ppmap.blocks[i].points.size() ; j++ ) {
				printf("%d,%d\n",ppmap.blocks[i].points[j].x,ppmap.blocks[i].points[j].y);
			}
		}
	}
	Mat image = ppmap.getImage();
  imshow("Delaunay",image);
	waitKey(0);
	return 0;
}
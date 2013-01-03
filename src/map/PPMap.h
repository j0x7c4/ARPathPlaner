#include <vector>
#include <cv.h>
#include <math.h>
using namespace std;
using namespace cv;
#define VIDEO_WIDTH 600
#define VIDEO_HEIGHT 600

class ppPoint{
public:
	int tag,x,y;
	ppPoint(){
		x=y=tag=0;
	}
	ppPoint(int tx,int ty, int ttag=0) { 
		x=tx;y=ty;
		tag = ttag;
	}
	bool operator == (const ppPoint& p)const {
		return p.x==x && p.y==y;
	}
	bool operator == (ppPoint& p)const {
		return p.x==x && p.y==y;
	}
};
class ppEdge{
public:
	ppPoint src,dst;
	int tag;
	ppEdge(){
		tag = 0;
	}
	ppEdge(ppPoint p1, ppPoint p2, int t=0) {
		src = p1; dst = p2; tag = t;
	}
};
class ppMapBlock {
public:
	vector<ppPoint> points;
	ppPoint center;
	int tag;
	int flag;//0 available, 1 not available
	ppMapBlock() {
		tag = 0;
		flag = 0;
	}
	ppMapBlock(ppPoint a,ppPoint b,ppPoint c, int ttag=0, int tflag = 0) {
		tag = ttag;
		points.push_back(a);
		points.push_back(b);
		points.push_back(c);
		double la = sqrt((double)((b.x-c.x)*(b.x-c.x)+(b.y-c.y)*(b.y-c.y)));
		double lb = sqrt((double)((a.x-c.x)*(a.x-c.x)+(a.y-c.y)*(a.y-c.y)));
		double lc = sqrt((double)((b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y)));
		center.x = (a.x*la)/(la+lb+lc)+(b.x*lb)/(la+lb+lc)+(c.x*lc)/(la+lb+lc);
		center.y = (a.y*la)/(la+lb+lc)+(b.y*lb)/(la+lb+lc)+(c.y*lc)/(la+lb+lc);
		flag = tflag;
	}
};
class ppMap {
	//strorage and structure for delaunay subdivsion
	CvRect   rect;   //Our outer bounding box
	Subdiv2D subdiv;
	Mat img;                    
	vector<ppPoint> border;
	vector<vector<ppPoint>> obstacles;
public:
	void init();
	vector<ppMapBlock> blocks;
	ppMap();
	~ppMap();
	vector<vector<int>> map;
	void createBorder(const vector<ppPoint>& _border);
	void createObstacles(const vector<vector<ppPoint>>& _obstacles); 
	void addMapBlock(ppMapBlock block);
	void createMap ();
	Mat getImage();
};
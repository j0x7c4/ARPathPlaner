#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>
using namespace std;
using namespace cv;

#define VIDEO_WIDTH 1000
#define VIDEO_HEIGHT 1000
#define REGION_THRESHOLD 600
#define REGION_NUMBER  4
#define REGION_OBSTACLE 1
#define REGION_HILL     2
#define REGION_RIVER    3
#define REGION_SPEED_HILL 0.6
#define REGION_SPEED_RIVER 0.8

class ppPoint{
public:
	int x,y,tag;
	ppPoint(){
		x = y = tag = 0;
	}
	ppPoint(int x,int y, int tag=0) { 
		this->x = x ;
		this->y = y;
		this->tag = tag;
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
bool onRight ( const ppPoint& a, const ppPoint& b, const ppPoint& c);
bool isInRegion ( const vector<ppPoint>& region, const ppPoint & p );
float triangleArea(const ppPoint& a,const ppPoint& b, const ppPoint& c);
class ppMapBlock {
public:
  float base_speed;
  float area;
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
    area = triangleArea(a,b,c);
		center.x = (a.x*la)/(la+lb+lc)+(b.x*lb)/(la+lb+lc)+(c.x*lc)/(la+lb+lc);
		center.y = (a.y*la)/(la+lb+lc)+(b.y*lb)/(la+lb+lc)+(c.y*lc)/(la+lb+lc);
		flag = tflag;
	}
  ppMapBlock(const vector<ppPoint>& pts, int tag=0, int flag=0 ) {
    float x=0,y=0;
		this->tag = tag;
    this->flag = flag;
    for ( int i= 0 ; i<pts.size() ; i++ ) {
      points.push_back(pts[i]);
			x+=pts[i].x;
			y+=pts[i].y;
    }
		this->center = ppPoint(x/pts.size(),y/pts.size());
    this->area = triangleArea(pts[0],pts[1],pts[2]) + triangleArea(pts[0],pts[3],pts[2]);
    switch (flag)
    {
    case REGION_HILL:
      this->base_speed = REGION_SPEED_HILL;
      break;
    case REGION_RIVER:
      this->base_speed = REGION_SPEED_RIVER;
      break;
    case REGION_OBSTACLE:
      this->base_speed = 0;
      break;
    default:
      this->base_speed = 1;
      break;
    }
  }
};


void mergeSmallBlockAsObstacle(vector<ppMapBlock>& blocks, vector<vector<int> >& map);

class ppMap {
	//strorage and structure for delaunay subdivsion
	Rect   rect;   //Our outer bounding box
	Subdiv2D subdiv;
	                   
	vector<ppPoint> border;
	vector<vector<vector<ppPoint> > > regions;
public:
  Mat img; 
	void init();
	vector<ppMapBlock> blocks;
	ppMap();
  ppMap(int width, int height);
	~ppMap();
	vector<vector<int> > map;
	void createBorder(const vector<ppPoint>& _border);
  void createRegions(const vector<vector<ppPoint> >& region, int region_tag);
	void createObstacles(const vector<vector<ppPoint> >& _obstacles); 
	void addMapBlock(ppMapBlock block);
	void createMap ();
	Mat getImage();
};


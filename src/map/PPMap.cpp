#include "PPMap.h"
#include <math.h>
#define ZERO(x) (fabs(x)<10e-6)
#define REGION_NUMBER  4
#define REGION_OBSTACLE 1
#define REGION_HILL     2
#define REGION_RIVER    3

ppMap::ppMap () {
  ppMap(VIDEO_WIDTH,VIDEO_HEIGHT);
}
ppMap::ppMap ( int width , int height ) {
  rect = Rect(0,0,width,height);
  img = Mat(cvSize(width,height),CV_MAKE_TYPE(8,3));
  img.setTo(0);
  regions = vector<vector<vector<ppPoint> > >(REGION_NUMBER, vector<vector<ppPoint> >());
}
//initialize
void ppMap::init (){
  Vec2f fp;               //This is our point holder
  subdiv.initDelaunay(rect);
	for(int i = 0; i < border.size(); i++ )
	{
		fp[0] = border[i].x;
		fp[1] = border[i].y;
		subdiv.insert(fp);
	}
  for ( int k=0 ; k<regions.size() ; k++ ) {
	  for ( int i=0; i <regions[k].size() ; i++ ) {
		  for ( int j=0 ; j<regions[k][i].size() ; j++ ) {
			  fp[0] = regions[k][i][j].x;
			  fp[1] = regions[k][i][j].y;
			  subdiv.insert(fp);
		  }
	  }
  }
}
void ppMap::createBorder(const vector<ppPoint>& _border) {
  border.clear();
	for ( int i= 0 ; i<_border.size() ; i++ ) {
		border.push_back(_border[i]);
	}
}
void ppMap::createObstacles(const vector<vector<ppPoint> >& _obstacles) {
  regions[REGION_OBSTACLE].clear();
  for ( int i=0 ; i<_obstacles.size() ; i++ ) {
		regions[REGION_OBSTACLE].push_back(vector<ppPoint>(_obstacles[i].begin(),_obstacles[i].end()));
	}
}
void ppMap::createRegions(const vector<vector<ppPoint> >& _regions, int region_tag){
}
void ppMap::addMapBlock ( ppMapBlock block ) {
	blocks.push_back(block);
}
//area of a triangle
float triangleArea(const ppPoint& a,const ppPoint& b, const ppPoint& c) {
  double la = sqrt((double)((b.x-c.x)*(b.x-c.x)+(b.y-c.y)*(b.y-c.y)));
	double lb = sqrt((double)((a.x-c.x)*(a.x-c.x)+(a.y-c.y)*(a.y-c.y)));
	double lc = sqrt((double)((b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y)));
  double p = (la+lb+lc)/2;
  return sqrt(p*(p-la)*(p-lb)*(p-lc));
}
//two blocks is connected
bool isConnected ( const ppMapBlock& b1,const  ppMapBlock& b2 ) {
	for ( int i=0 ; i<b1.points.size()-1 ; i++ ) {
		for ( int j=i+1 ; j<b1.points.size() ; j++ ) {
			for ( int p = 0 ; p<b2.points.size()-1 ; p++ ) {
				for ( int q = p+1 ; q<b2.points.size() ; q++ ) {
					if ((b1.points[i] == b2.points[p] && b1.points[j] == b2.points[q]) ||
						(b1.points[j] ==  b2.points[p] && b1.points[i] ==  b2.points[q]) ) {
						return true;
					}
				}
			}
		}
	}
	return false;
}
//if cross product is positive, the point c is on the right-side of the line ab 
bool onRight ( const ppPoint& a,const ppPoint& b, const ppPoint& c) {
	float a1 = c.x - a.x, a2 = -c.y + a.y; //the y-coordinate in opencv is inversed
	float b1 = b.x - a.x, b2 = -b.y+a.y;//the y-coordinate in opencv is inversed
	return a1*b2 - a2*b1 > 0;
}
//if cross product is zero, the point c is on the line ab 
bool onLine ( const ppPoint& a,const ppPoint& b, const ppPoint& c) {
	float a1 = c.x - a.x, a2 = -c.y + a.y; //the y-coordinate in opencv is inversed
	float b1 = b.x - a.x, b2 = -b.y+a.y;//the y-coordinate in opencv is inversed
	return ZERO(a1*b2 - a2*b1);
}
bool isRegion ( const ppMapBlock& b , const vector<vector<ppPoint> >& regions) {
	int j;
	for ( int i=0 ; i<regions.size() ; i++ ) {
		int n = regions[i].size();
		if ( !onRight(regions[i][n-1],regions[i][0],b.center) ) continue;
		for ( j=0 ; j<n-1; j++ ) {
			if ( !onRight(regions[i][j],regions[i][j+1],b.center) ) break;
		}
		if ( j==n-1 ) return true;
	}
	return false;
}
void drawBlock ( Mat& image, const vector<ppMapBlock>& blocks, CvScalar color ){
	char str[10];
	vector<vector<Point> > pts;
	for ( int i=0 ; i<blocks.size() ; i++ ) {
		vector<Point> curve;
		for ( int j = 0 ; j<blocks[i].points.size() ; j++ ) {
			curve.push_back(cvPoint(blocks[i].points[j].x,blocks[i].points[j].y));
		}
		pts.push_back(curve);
	}
	fillPoly(image,pts,color);
	polylines(image, pts,true,cvScalar(0,0,0));
	for ( int i=0 ; i<blocks.size() ; i++ ) {
		sprintf(str,"%d",blocks[i].tag);
		putText(image,str,cvPoint(blocks[i].center.x,blocks[i].center.y),FONT_HERSHEY_SIMPLEX,0.5,cvScalar(0,0,0));
	}
}
bool isInRegion ( const vector<ppPoint>& region, const ppPoint & p ) {
	if ( onRight(region[region.size()-1],region[0],p) || onLine(region[region.size()-1],region[0],p ) ) {
		for ( int i=0 ; i<region.size()-1 ; i++ ) {
			if ( !onRight(region[i],region[i+1],p) && !onLine(region[i],region[i+1],p) ) return false;
		}
		return true;
	}
	return false;
}
void mergeSmallBlockAsObstacle(vector<ppMapBlock>& blocks, vector<vector<int> >&map) {
  for ( int i=0 ; i<blocks.size() ; i++ ) {
    if ( blocks[i].flag != REGION_OBSTACLE && blocks[i].area<REGION_THRESHOLD ) {
      for ( int j=0 ; j<blocks.size() ; j++ ) {
        if ( map[i][j] == 2 && blocks[j].area>REGION_THRESHOLD ) {
          blocks[i].flag = REGION_OBSTACLE;
          for ( int k=0 ; k<blocks.size() ; k++ ) {
            map[k][i]=map[i][k] = 0;
          }
        }
      }
    }
  }
  for ( int i=0 ; i<map.size() ; i++ ) {
    for ( int j=i+1 ; j<map[i].size() ; j++ ) {
      if ( map[i][j] == 2 ) {
        map[i][j] = map[j][i] = 0;
      }
    }
  }
}
//merge temp triangles
bool merge ( const ppMapBlock& t1, const ppMapBlock& t2 , ppMapBlock& block) {
	vector<Point2f> convexHullPts;
	vector<int> idx;
	for ( int i=0 ; i<t1.points.size(); i++) {
		convexHullPts.push_back(Point2f(t1.points[i].x,t1.points[i].y));
	}
	for ( int i=0 ; i<t2.points.size(); i++) {
		convexHullPts.push_back(Point2f(t2.points[i].x,t2.points[i].y));
	}
	convexHull(convexHullPts,idx,true,false);
	//the two triangles can be merged into convex quad-, iff the #vert. in convex hull is 4
	if ( idx.size() == 4 ) {
		vector<ppPoint> pts;
		for ( int i=idx.size()-1 ; i>=0 ; i-- ) {
			pts.push_back(ppPoint(convexHullPts[idx[i]].x,convexHullPts[idx[i]].y));
		}
		block = ppMapBlock(pts,0,t1.flag);
		return true;
	}
	return false;
}
void mergeTriangles ( const vector<ppMapBlock>& triangels, const vector<vector<int> >& map, vector<ppMapBlock>& blocks ) {
  int n = triangels.size();
  vector<int> mark(n,0);
  for ( int i=0 ; i<n ; i++ ) {
    if (!mark[i]) {
			int j;
			for ( j=i+1 ; j<n; j++ ) {
				if ( !mark[j] && map[i][j]==1 ) {
					ppMapBlock block;
					if ( merge(triangels[i],triangels[j],block) ) {
						mark[j] = 1;
						blocks.push_back(block);
						break;
					}
				}
			}
			if ( j>=n ) {
				vector<Point2f> convexHullPts;
				vector<int> idx;
				for ( int k=0 ; k<3; k++) {
					convexHullPts.push_back(Point2f(triangels[i].points[k].x,triangels[i].points[k].y));
				}
				convexHull(convexHullPts,idx,true,false);
				blocks.push_back(ppMapBlock(triangels[i].points[idx[2]],triangels[i].points[idx[1]],triangels[i].points[idx[0]]));
			}
		}
  }
}
void ppMap::createMap() {
	vector<Vec6f> triangles;
  vector<ppMapBlock> tempBlocks;
  vector<vector<int> > tempMap;
	int nblock=0;
  subdiv.getTriangleList(triangles);
  //collect temp triangles
	for ( int i=0 ; i<triangles.size() ; i++ ) {
		if ( isInRegion(border,ppPoint(triangles[i][0],triangles[i][1])) &&
				 isInRegion(border,ppPoint(triangles[i][2],triangles[i][3])) &&
				 isInRegion(border,ppPoint(triangles[i][4],triangles[i][5])) ) {
			tempBlocks.push_back(ppMapBlock(ppPoint(triangles[i][0],triangles[i][1]),
																ppPoint(triangles[i][2],triangles[i][3]),
																ppPoint(triangles[i][4],triangles[i][5]),nblock++));
			if ( isRegion(tempBlocks[nblock-1],regions[REGION_OBSTACLE]) )
				tempBlocks[nblock-1].flag = REGION_OBSTACLE;
      else if ( isRegion(tempBlocks[nblock-1],regions[REGION_HILL]) ) {
        tempBlocks[nblock-1].flag = REGION_HILL;
      }
      else if ( isRegion(tempBlocks[nblock-1],regions[REGION_RIVER]) ) {
        tempBlocks[nblock-1].flag = REGION_RIVER;
      }
		}
	}
	tempMap.resize(tempBlocks.size(),vector<int>(tempBlocks.size(),0));
	for ( int i=0 ; i<tempBlocks.size() ; i++ ) {
		for ( int j=i+1 ; j<tempBlocks.size() ; j++ ) {
			if ( isConnected(tempBlocks[i],tempBlocks[j]) ) {
				if ( tempBlocks[i].flag != tempBlocks[j].flag ) {
					tempMap[i][j] = tempMap[j][i] = 2; //plain and obstecal
				}
				else {
					tempMap[i][j] = tempMap[j][i] = 1; // plain and plain or  obstecal and obstecal
				}
			}
		}
	}
	mergeTriangles(tempBlocks,tempMap,blocks);
  
	map.resize(blocks.size(),vector<int>(blocks.size(),0));
	for ( int i=0 ; i<blocks.size() ; i++ ) {
		blocks[i].tag = i;
		for ( int j=i+1 ; j<blocks.size() ; j++ ) {
			if ( isConnected(blocks[i],blocks[j]) ){
        if (blocks[i].flag==blocks[j].flag ) {
				  map[i][j] = map[j][i] = 1;
        }
        else {
          map[i][j] = map[j][i] = 2;
        }
			}

		}
	}
  mergeSmallBlockAsObstacle ( blocks, map );
	vector<ppMapBlock> plains;
	vector<ppMapBlock> obstacles;
	for (int i=0 ; i<blocks.size(); i++ ) {
		if ( blocks[i].flag ) {
			obstacles.push_back(blocks[i]);
		}
		else {
			plains.push_back(blocks[i]);
		}
	}
  img.setTo(0);
  
	drawBlock(img,plains,cvScalar(255,255,187));
	drawBlock(img,obstacles,cvScalar(34,34,178));
  /*
	printf("total blocks:%d\n",blocks.size());
	for ( int i=0 ; i<map.size() ; i++ ) {
		for ( int j=0 ; j<map[i].size(); j++ ) {
			printf("%d",map[i][j]);
		}
		printf("\n");
	}
  */ 
}
 Mat ppMap::getImage() {
	 return img.clone();
 }
 
 ppMap::~ppMap (){

 }
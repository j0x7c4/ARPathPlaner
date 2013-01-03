#include "PPMap.h"
#include <math.h>
#define ZERO(x) (fabs(x)<10e-6)
int mark[VIDEO_HEIGHT][VIDEO_WIDTH];


ppMap::ppMap () {
}

void ppMap::init (){
	rect.x = 0;
	rect.y = 0;
	rect.width = VIDEO_WIDTH;
	rect.height = VIDEO_HEIGHT;
	//initialize
	storage = cvCreateMemStorage(0); 
	subdiv = cvCreateSubdiv2D(CV_SEQ_KIND_SUBDIV2D,sizeof(*subdiv),
	sizeof(CvSubdiv2DPoint),sizeof(CvQuadEdge2D),storage);
	cvInitSubdivDelaunay2D( subdiv, rect ); //rect sets the bounds             
	img=cvCreateImage(cvSize(rect.width,rect.height),8,3);
	active_facet_color = CV_RGB( 255, 0, 0 );
	delaunay_color  = CV_RGB( 0,0,0);
	voronoi_color = CV_RGB(0, 180, 0);
	bkgnd_color = CV_RGB(255,255,255);
	cvSet(img,bkgnd_color,0);
	CvPoint2D32f fp;               //This is our point holder
	for(int i = 0; i < border.size(); i++ )
	{
		fp.x = border[i].x;
		fp.y = border[i].y;
		cvSubdivDelaunay2DInsert( subdiv, fp );
		cvCalcSubdivVoronoi2D( subdiv );  // Fill out Voronoi data in subdiv
	}
	for ( int i=0; i <obstacles.size() ; i++ ) {
		for ( int j=0 ; j<obstacles[i].size() ; j++ ) {
			fp.x = obstacles[i][j].x;
			fp.y = obstacles[i][j].y;
			cvSubdivDelaunay2DInsert( subdiv, fp );
			cvCalcSubdivVoronoi2D( subdiv );  // Fill out Voronoi data in subdiv
		}
	}
}
void ppMap::createBorder(const vector<ppPoint>& _border) {
	for ( int i= 0 ; i<_border.size() ; i++ ) {
		border.push_back(_border[i]);
	}
}
void ppMap::createObstacles(const vector<vector<ppPoint>>& _obstacles) {
	for ( int i=0 ; i<_obstacles.size() ; i++ ) {
		obstacles.push_back(vector<ppPoint>(_obstacles[i].begin(),_obstacles[i].end()));
	}
}
void ppMap::addMapBlock ( ppMapBlock block ) {
	blocks.push_back(block);
}

//get subdiv edge-ends
void get_subdiv_edge( CvSubdiv2DEdge edge, ppPoint & iorg, ppPoint& idst )
{
	CvSubdiv2DPoint* org_pt;
	CvSubdiv2DPoint* dst_pt;
	CvPoint2D32f org;
	CvPoint2D32f dst;

	org_pt = cvSubdiv2DEdgeOrg(edge);
	dst_pt = cvSubdiv2DEdgeDst(edge);

	if( org_pt && dst_pt )
	{
		org = org_pt->pt;
		dst = dst_pt->pt;

		iorg = ppPoint( cvRound( org.x ), cvRound( org.y ));
		idst = ppPoint( cvRound( dst.x ), cvRound( dst.y ));
	}
}
//two blocks is connected
bool isConnected ( const ppMapBlock& b1,const  ppMapBlock& b2 ) {
	if ( b1.flag == 1 || b2.flag == 1 ) return false; //obstacle 
	for ( int i=0 ; i<b1.points.size()-1 ; i++ ) {
		for ( int j=i+1 ; j<b1.points.size() ; j++ ) {
			for ( int p = 0 ; p<b2.points.size()-1 ; p++ ) {
				for ( int q = p+1 ; q<b2.points.size() ; q++ ) {
					if (b1.points[i] == b2.points[p] && b1.points[j] == b2.points[q] || 
						b1.points[j] ==  b2.points[p] && b1.points[i] ==  b2.points[q] ) {
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
bool isObstacle ( const ppMapBlock& b , const vector<vector<ppPoint>>& obstacles) {
	int j;
	for ( int i=0 ; i<obstacles.size() ; i++ ) {
		int n = obstacles[i].size();
		if ( !onRight(obstacles[i][n-1],obstacles[i][0],b.center) ) continue;
		for ( j=0 ; j<n-1; j++ ) {
			if ( !onRight(obstacles[i][j],obstacles[i][j+1],b.center) ) break;
		}
		if ( j==n-1 ) return true;
	}
	return false;
}
void drawBlock ( Mat& image, const vector<ppMapBlock>& blocks, CvScalar color ){
	char str[10];
	vector<vector<Point>> pts;
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
 void ppMap::createMap() {
	CvSeqReader  reader;
	int total = subdiv->edges->total;
	int elem_size = subdiv->edges->elem_size;
	vector<ppEdge> edges;
	vector<ppPoint> points;
	
	int cnt = 0;
	memset(mark,0,sizeof(mark));

	cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

	for( int i = 0; i < total; i++ )
	{
		CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);
		if( CV_IS_SET_ELEM( edge ))
		{
			ppPoint org,dst;
			get_subdiv_edge( (CvSubdiv2DEdge)edge,org,dst);
			if ( isInRegion(border,org) && isInRegion(border,dst) ) {
				if ( !mark[org.x][org.y] ) {
					mark[org.x][org.y] = org.tag = ++cnt;
					points.push_back(org);
				}
				else{
					org.tag = mark[org.x][org.y];
				}
				if ( !mark[dst.x][dst.y] ) {
					mark[dst.x][dst.y] = dst.tag = ++cnt;
					points.push_back(dst);
				}
				else{
					dst.tag = mark[dst.x][dst.y];
				}
				printf ("(%d,%d),(%d,%d)\n",org.x,org.y,dst.x,dst.y);
				edges.push_back(ppEdge(org,dst));
			}
		}
		CV_NEXT_SEQ_ELEM( elem_size, reader );
	}
	vector<vector<bool>> graph(cnt+1,vector<bool>(cnt+1,false));
	int nblock=0;
	for ( int i=0 ; i<edges.size() ; i++ ) {
		int p,q;
		p = edges[i].src.tag;
		q = edges[i].dst.tag;
		graph[p][q]=graph[q][p] = true;
		for ( int j=1 ; j<=cnt ; j++ ) {
			if ( j==p || j==q ) continue;
			if ( graph[p][j] && graph[j][q] ) {
				blocks.push_back(ppMapBlock(points[p-1],points[q-1],points[j-1],nblock++));
				if ( isObstacle(blocks[nblock-1],obstacles) ) {
					blocks[nblock-1].flag = 1;
				}
			}
		}
	}
	map.resize(blocks.size(),vector<int>(blocks.size(),0));
	for ( int i=0 ; i<blocks.size() ; i++ ) {
		for ( int j=i+1 ; j<blocks.size() ; j++ ) {
			if ( isConnected(blocks[i],blocks[j]) ) {
				map[i][j] = map[j][i] = 1;
			}
		}
	}
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
	drawBlock(Mat(img,0),plains,cvScalar(255,255,187));
	drawBlock(Mat(img,0),obstacles,cvScalar(34,34,178));
	printf("total edge:%d\n",edges.size());
	printf("total blocks:%d\n",blocks.size());
	for ( int i=0 ; i<map.size() ; i++ ) {
		for ( int j=0 ; j<map[i].size(); j++ ) {
			printf("%d",map[i][j]);
		}
		printf("\n");
	}
}

 Mat ppMap::getImage() {
	return Mat(img,1);
 }
 
 ppMap::~ppMap (){
	cvReleaseMemStorage( &storage );
	cvReleaseImage(&img);
 }
#include <iostream>
#include <aruco.h>
#include <cvdrawingutils.h>
#include <Search.h>
#include <opencv2\opencv.hpp>
using namespace cv;
using namespace aruco;

//#define RECORD_TEST_VIDEO
//#define RECORD_DEMO_VIDEO
#define obj 250
#define bor 1008
#define en 1000
#define exi 500
#define hill 700
#define river 600

#define MOTION_ENERGY_WINDOW_SIZE 10
#define MOTION_THRESHOLD 10

#define TEST_VIDEO "test_video_3.avi"
#define IMG_BORDER "border.jpg"
#define IMG_TEEMO "teemo.jpg"
#define IMG_ENTER "start.jpg"
#define IMG_EXIT "exit.jpg"
#define IMG_EVIL "tree.jpg"
#define IMG_HILL "hill.jpg"
#define IMG_RIVER "river.jpg"
void put_obj(Mat& input, char *name, int x, int y, int size = 80);
void drawLine(const vector<Point2i>& path, Mat& img);
double calcEnergy ( const vector<Mat>& clips );
bool in_ball(Point2f, Point2f, float);
vector<Point2f> running_teemo(vector<Point> , int);

int motion_state=0;

int main(int argc,char **argv){

  VideoWriter video_writer;
  VideoCapture cap(1);// open the default camera
#ifdef RECORD_TEST_VIDEO
  video_writer.open("test_video_2.avi",CV_FOURCC('D','I','V','X'),30,cvSize(800,600),true);
#endif
#ifdef RECORD_DEMO_VIDEO
  video_writer.open("demo_video_tmp.avi",CV_FOURCC('D','I','V','X'),30,cvSize(800,600),true);
#endif
  
  cap.set(CV_CAP_PROP_FPS,60);
  cap.set(CV_CAP_PROP_FRAME_WIDTH ,800);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT,600);
  if(!cap.isOpened())  // check if we succeeded
    return -1;

  int enter_flag=0;
  int exit_flag=0;
  
  aruco::CameraParameters CamParam;
  MarkerDetector MDetector;
  float MarkerSize=-1;
  //read the input image
  Mat InImage;
  Mat map_image(cvSize(800,600),CV_MAKETYPE(8,3));
  int key;
  //read camera parameters if specifed
  int cnt = 0;
  int reroute = 0;
  int redetect = 0;
  int running = 0;
  int running_path_idx = 0;

  vector<Point2f> running_path;
  vector<Marker> Markers;
  vector<Point2f> borders; //border
  //vector<Marker> Obstacles; //obstacle
  vector<Point2i> route;
  Point2f in_door; //entry and exit
  vector<Point2f> out_doors;
  double energy = 0;
  vector<Mat> energy_images(MOTION_ENERGY_WINDOW_SIZE,Mat());
  
  ///start here!!!
  while(cap.read(InImage)){// get a new frame from camera
#ifdef RECORD_TEST_VIDEO
    video_writer<<InImage;
    imshow("RECORD",InImage);
    if ( waitKey(30)>=0 ) 
      break;
    continue;
    //printf("%d\n",cnt);
#endif
    if ( cnt == 10000*MOTION_ENERGY_WINDOW_SIZE ) cnt = MOTION_ENERGY_WINDOW_SIZE;
    cvtColor(InImage,energy_images[(++cnt)%MOTION_ENERGY_WINDOW_SIZE],CV_RGB2GRAY);
    if ( cnt >= MOTION_ENERGY_WINDOW_SIZE ) {
      energy = calcEnergy(vector<Mat>(energy_images.begin(),energy_images.end()));
      //printf("Energy: %lf\n",energy);
    }
    switch ( motion_state ) {
    case 0:
      if ( energy > MOTION_THRESHOLD )
        motion_state = 1;
      break;
    case 1:
      if ( energy < MOTION_THRESHOLD ) 
        motion_state = 2;
      break;
    case 2:
      motion_state = 0;
      redetect = 1;
      break;
    }
    vector<ppPoint> ppBorder;
    vector<vector<vector<ppPoint> > >ppRegions(REGION_NUMBER, vector<vector<ppPoint> >());
    //vector<vector<ppPoint> > ppObstacles;
    vector<int> idx;
    ppMap map(800,600);
    


    if ( redetect ) {
      //ppHills.clear();
      borders.clear();
      //Obstacles.clear();
      out_doors.clear();
      //Ok, let's detect
      MDetector.detect(InImage,Markers,CamParam,MarkerSize);
      //for each marker, draw info and its boundaries in the image
      for (int i=0;i<Markers.size();i++) {
        //cout<<Markers[i]<<endl;
        vector<ppPoint> object;
        switch(Markers[i].id){
        case bor:
          borders.push_back(Markers[i].getCenter());
          break;
        case en:
          in_door=Markers[i].getCenter();
          enter_flag=1;
          break;
        case exi:
          exit_flag=1;
          out_doors.push_back(Markers[i].getCenter());
          break;
        case obj:
          for(int j=0;j<Markers[i].size();j++){
            //cout << "(" << Obstacles[i][j].x << "," << Obstacles[i][j].y << ")";
            object.push_back(ppPoint(Markers[i][j].x, Markers[i][j].y));
          }
          ppRegions[REGION_OBSTACLE].push_back(object);
          break;
        case hill:
          for(int j=0;j<Markers[i].size();j++){
            //cout << "(" << Obstacles[i][j].x << "," << Obstacles[i][j].y << ")";
            object.push_back(ppPoint(Markers[i][j].x, Markers[i][j].y));
          }
          ppRegions[REGION_HILL].push_back(object);
          break;
        case river:
           for(int j=0;j<Markers[i].size();j++){
            //cout << "(" << Obstacles[i][j].x << "," << Obstacles[i][j].y << ")";
            object.push_back(ppPoint(Markers[i][j].x, Markers[i][j].y));
          }
          ppRegions[REGION_RIVER].push_back(object);
          break;

        }
        
      }
      redetect = 0;
      reroute = 1;
    }
    //draw markers;
    for ( int i=0 ; i<Markers.size() ; i++ ) {
      Markers[i].draw(InImage,Scalar(0,0,255),2);
      switch (Markers[i].id) {
       case bor:
          break;
        case en:
          put_obj(InImage, IMG_ENTER, Markers[i].getCenter().x, Markers[i].getCenter().y, 80);
          break;
        case exi:
          put_obj(InImage, IMG_EXIT , Markers[i].getCenter().x, Markers[i].getCenter().y, 80);
          break;
        case obj:
          put_obj(InImage, IMG_EVIL , Markers[i].getCenter().x, Markers[i].getCenter().y, 80);
          break;
        case hill:
          put_obj(InImage, IMG_HILL, Markers[i].getCenter().x, Markers[i].getCenter().y, 80);
          break;
        case river:
          put_obj(InImage, IMG_RIVER, Markers[i].getCenter().x, Markers[i].getCenter().y, 80);
          break;
      }
      
    }

    if(in_door.x!=0 && in_door.y!=0 ) {
      if ( !running ) {
        put_obj(InImage, IMG_TEEMO, in_door.x, in_door.y, 80);
      }
      //cout << "entry" << "(" << in_door.x << "," << in_door.y << ")" << endl;
    }
    if(borders.size()!=0)
      convexHull(borders,idx,true,false);

    for(int i=0;i<idx.size();i++){
      ppBorder.push_back(ppPoint(borders[idx[idx.size()-1-i]].x,borders[idx[idx.size()-1-i]].y));
      //cout << "border_" << i << "(" << borders[idx[idx.size()-1-i]].x << "," << borders[idx[idx.size()-1-i]].y << ")" << endl;
    }
    /*
    for(int i=0;i<Obstacles.size();i++){
      // cout << "Obstacle" << i << ":";
      
      vector<ppPoint> object;
      for(int j=0;j<Obstacles[i].size();j++){
        //cout << "(" << Obstacles[i][j].x << "," << Obstacles[i][j].y << ")";
        object.push_back(ppPoint(Obstacles[i][j].x, Obstacles[i][j].y));
      }
      ppObstacles.push_back(object);
      //cout << endl;
    }
    */
    if(out_doors.size()>0 ){
      for ( int i=0 ; i<out_doors.size() ; i++ ) {
        if ( out_doors[i].x == 0 || out_doors[i].y ==0 ) continue;
        
        //cout << "exit" << "(" << out_door.x << "," << out_door.y << ")" << endl;
      }
    }
    if(reroute && ppBorder.size() > 2 ){
      map.createBorder(ppBorder);
      //map.createObstacles(ppObstacles);
      map.createRegions(ppRegions[REGION_OBSTACLE],REGION_OBSTACLE);
      map.createRegions(ppRegions[REGION_HILL],REGION_HILL);
      map.createRegions(ppRegions[REGION_RIVER],REGION_RIVER);
      map.init();
      map.createMap();
      map_image = map.getImage();
    }

    if ( reroute && map.blocks.size()>0 && enter_flag && exit_flag ) {
      route.clear();
      vector<int> inOut;
      if ( running_path_idx<running_path.size() ) {
        inOut.push_back(running_path[running_path_idx].x);
        inOut.push_back(running_path[running_path_idx].y);
      } else {
        inOut.push_back(in_door.x);
        inOut.push_back(in_door.y);
      }
      for ( int i=0 ; i<out_doors.size() ; i++ ) {
        inOut.push_back(out_doors[i].x);
        inOut.push_back(out_doors[i].y);
      }
      Search search(inOut, map);
      search.aStar();
      search.getRoute(route);
      running_path = running_teemo(route,2);
      running_path_idx = 0;
      running = 1;
      //cout<<running_path<<endl;
      reroute = 0;
    }
    drawLine(route,InImage);
    if ( running && running_path_idx < running_path.size() ) {
      put_obj(InImage,IMG_TEEMO,running_path[running_path_idx].x,running_path[running_path_idx].y);
      running_path_idx++;
      if ( running_path_idx == running_path.size() )
        running = 0;
    }
#ifdef RECORD_DEMO_VIDEO
    video_writer<<InImage;
#endif
    cv::imshow("map",map_image);
    cv::imshow("in",InImage);
    key = waitKey(30);
    if ( key == 'q' )
      break;
    else if ( key == 'r' ) 
      reroute = 1;
    else if ( key == 'd' ) 
      redetect = 1;
  }
}
void put_obj(Mat& input, char *name, int x, int y, int size){
  Mat picture =imread(name, 1);
  resize(picture, picture, Size(size,size), 0, 0, INTER_NEAREST);
  x-=(size/2);
  y-=size;
  for(int i=0; i<size;i++){
    for(int j=0; j<size;j++){
      if(x+i>0 && y+j>0){
        if(!(picture.at<cv::Vec3b>(j,i)[0]>=220 && picture.at<cv::Vec3b>(j,i)[1]>=220 && picture.at<cv::Vec3b>(j,i)[2]>=220
          ))
          input.at<cv::Vec3b>(y+j,x+i) = picture.at<cv::Vec3b>(j,i);
      }
    }
  }
}
double calcEnergy ( const vector<Mat>& clips ) {
  double e=0;
  uchar* ptr;
  double min_value,max_value;
  Mat m(clips[0].size(),CV_MAKETYPE(16,1));
  m.setTo(0);
  for ( int i=0 ; i<clips.size()-1 ; i+=2 ) {
    m+=abs(clips[i]-clips[i+1]);
  }
  minMaxLoc(m,&min_value,&max_value);
  convertScaleAbs(m,m,255.0/max_value);
  threshold(m,m,128,255,THRESH_BINARY);
  //imshow("energy",m);
  for ( int i = 0 ; i<m.rows;  i++ ) {
    ptr =(uchar*) m.ptr(i);
    for ( int j = 0 ; j<m.cols; j++ ) {
      e+=*ptr++;
    }
  }
  return e/(m.rows*m.cols);
}
/* draw the path line and push the coordinate into pointPath*/
void drawLine(const vector<Point2i>& path, Mat& img){
  if ( path.size() ==0 ) return;
  for ( int i=0 ; i<path.size()-1 ; i++ ) {
    line(img, path[i], path[i+1], cvScalar(255,0 ,0), 2, 3, 0);
  }
}

vector<Point2f> running_teemo(vector<Point> points, int vel){
    vector<Point2f> path;
    path.push_back(points[0]);
    for(int i=0; i<points.size()-1;i++){
        Point2f vec(points[i+1].x-points[i].x, points[i+1].y-points[i].y);
        float len = sqrt(pow(vec.x,2)+pow(vec.y,2));
        vec.x /= len;
        vec.y /= len;
        for(Point2f now(points[i].x+(vec.x*vel),points[i].y+(vec.y*vel)) ; now!=(Point2f)points[i+1]; now.x+=(vec.x*vel), now.y+=(vec.y*vel)){
            if(in_ball(now, points[i+1], vel)){
                //cout << "haha" << endl;
                now = points[i+1];
                path.push_back(now);
                break;
            }
            //cout << now << endl;
            path.push_back(now);
        }
    }
    return path;
}

bool in_ball(Point2f input, Point2f center, float radius){
    if(pow(input.x-center.x,2)+pow(input.y-center.y,2)<=pow(radius, 2))
       return true;
    else
       return false;
}
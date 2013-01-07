#include <iostream>
#include <aruco.h>
#include <cvdrawingutils.h>
#include <Search.h>
#include <opencv2\opencv.hpp>
using namespace cv;
using namespace aruco;

#define obj 250
#define bor 1008
#define en 1000
#define exi 500

#define MOTION_ENERGY_WINDOW_SIZE 10
#define MOTION_THRESHOLD 5

#define TEST_VIDEO "test_video_0.avi"
#define IMG_BORDER "border.jpg"
#define IMG_ENTER "teemo.jpg"
#define IMG_EXIT "exit.jpg"
#define IMG_EVIL "tree.jpg"

void put_obj(Mat& input, char *name, int x, int y, int size);
void drawLine(const vector<Point2i>& path, Mat& img);
double calcEnergy ( const vector<Mat>& clips );

int motion_state=0;

int main(int argc,char **argv){

  VideoWriter video_writer;
  //video_writer.open("test_video.avi",CV_FOURCC('D','I','V','X'),30,cvSize(800,600),true);

  VideoCapture cap(TEST_VIDEO); // open the default camera
  cap.set(CV_CAP_PROP_FPS,30);
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
  Mat map_image;
  int key;
  //read camera parameters if specifed
  int cnt = 0;
  int reroute = 1;
  int redetect = 1;
  vector<Marker> Markers;
  vector<Point2f> borders; //border
  vector<Marker> Obstacles; //obstacle
  vector<Point2i> route;
  Point2f in_door; //entry and exit
  vector<Point2f> out_doors;
  double energy = 0;
  vector<Mat> energy_images(MOTION_ENERGY_WINDOW_SIZE,Mat());
  while(cap.read(InImage)){// get a new frame from camera
    if ( cnt == 10000*MOTION_ENERGY_WINDOW_SIZE ) cnt = MOTION_ENERGY_WINDOW_SIZE;
    cvtColor(InImage,energy_images[(++cnt)%MOTION_ENERGY_WINDOW_SIZE],CV_RGB2GRAY);
    if ( cnt >= MOTION_ENERGY_WINDOW_SIZE ) {
      energy = calcEnergy(vector<Mat>(energy_images.begin(),energy_images.end()));
      printf("Energy: %lf\n",energy);
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
    vector<vector<ppPoint> > ppObstacles;
    vector<int> idx;
    ppMap map(800,600);
    
     
    //video_writer<<InImage;
    //printf("%d\n",cnt);
    
    if ( redetect ) {
      borders.clear();
      Obstacles.clear();
      out_doors.clear();
      //Ok, let's detect
      MDetector.detect(InImage,Markers,CamParam,MarkerSize);
      //for each marker, draw info and its boundaries in the image
      for (int i=0;i<Markers.size();i++) {
        //cout<<Markers[i]<<endl;
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
          Obstacles.push_back(Markers[i]);
          break;
        }
        
      }
      redetect = 0;
      reroute = 1;
    }
    //draw markers;
    for ( int i=0 ; i<Markers.size() ; i++ ) {
      Markers[i].draw(InImage,Scalar(0,0,255),2);
    }

    if(in_door.x!=0 && in_door.y!=0 )
      put_obj(InImage, IMG_ENTER, in_door.x, in_door.y, 80);
    //cout << "entry" << "(" << in_door.x << "," << in_door.y << ")" << endl;

    if(borders.size()!=0)
      convexHull(borders,idx,true,false);

    for(int i=0;i<idx.size();i++){
      ppBorder.push_back(ppPoint(borders[idx[idx.size()-1-i]].x,borders[idx[idx.size()-1-i]].y));
      //cout << "border_" << i << "(" << borders[idx[idx.size()-1-i]].x << "," << borders[idx[idx.size()-1-i]].y << ")" << endl;
    }

    for(int i=0;i<Obstacles.size();i++){
      // cout << "Obstacle" << i << ":";
      put_obj(InImage, IMG_EVIL , Obstacles[i].getCenter().x, Obstacles[i].getCenter().y, 80);
      vector<ppPoint> object;
      for(int j=0;j<Obstacles[i].size();j++){
        //cout << "(" << Obstacles[i][j].x << "," << Obstacles[i][j].y << ")";
        object.push_back(ppPoint(Obstacles[i][j].x, Obstacles[i][j].y));
      }
      ppObstacles.push_back(object);
      //cout << endl;
    }
    if(out_doors.size()>0 ){
      for ( int i=0 ; i<out_doors.size() ; i++ ) {
        if ( out_doors[i].x == 0 || out_doors[i].y ==0 ) continue;
        put_obj(InImage, IMG_EXIT , out_doors[i].x, out_doors[i].y, 80);
        //cout << "exit" << "(" << out_door.x << "," << out_door.y << ")" << endl;
      }
    }
    if(reroute){
      map.createBorder(ppBorder);
      map.createObstacles(ppObstacles);
      map.init();
      map.createMap();
      map_image = map.getImage();
    }

    if ( reroute && map.blocks.size()>0 && enter_flag && exit_flag ) {
      route.clear();
      vector<int> inOut;
      inOut.push_back(in_door.x);
      inOut.push_back(in_door.y);
      for ( int i=0 ; i<out_doors.size() ; i++ ) {
        inOut.push_back(out_doors[i].x);
        inOut.push_back(out_doors[i].y);
      }
      Search search(inOut, map);
      search.aStar();
      search.getRoute(route);
      reroute = 0;
    }
    drawLine(route,InImage);
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
  imshow("energy",m);
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
#include <iostream>
#include <aruco.h>
#include <cvdrawingutils.h>
#include <Search.h>

#define obj 250
#define bor 1008
#define en 1000
#define exi 500

using namespace cv;
using namespace aruco;
#define IMG_BORDER "border.jpg"
#define IMG_ENTER "teemo.jpg"
#define IMG_EXIT "exit.jpg"
#define IMG_EVIL "tree.jpg"

void put_obj(Mat& input, char *name, int x, int y, int size);

int main(int argc,char **argv){


  VideoCapture cap(1); // open the default camera
  if(!cap.isOpened())  // check if we succeeded
    return -1;

  
  aruco::CameraParameters CamParam;
  MarkerDetector MDetector;
  float MarkerSize=-1;
  //read the input image
  cv::Mat InImage;

  //read camera parameters if specifed
  int cnt = 0;

  vector<Marker> Markers;
  vector<Point2f> borders; //border
  vector<Marker> Obstacles; //obstacle
  

  while(1){
    int enter_flag=0;
    int exit_flag=0;
    Point2f in_door; //entry and exit
    vector<Point2f> out_doors;
    vector<ppPoint> ppBorder;
    vector<vector<ppPoint> > ppObstacles;
    vector<int> idx;
    ppMap map(800,600);
    //Ok, let's detect
    cap >> InImage; // get a new frame from camera
    resize(InImage, InImage, Size(800,600), 0, 0, INTER_NEAREST);
    //printf("%d\n",cnt);
    if ( cnt++ > -1 ) {
      borders.clear();
      Obstacles.clear();
      cnt = 0;
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
        Markers[i].draw(InImage,Scalar(0,0,255),2);
      }
    }


    if(in_door.x!=0 && in_door.y!=0 )
        put_obj(InImage, IMG_ENTER, in_door.x, in_door.y, 80);
        cout << "entry" << "(" << in_door.x << "," << in_door.y << ")" << endl;

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
        cout << "(" << Obstacles[i][j].x << "," << Obstacles[i][j].y << ")";
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
    if(borders.size()>2){
      map.createBorder(ppBorder);
      map.createObstacles(ppObstacles);
      map.init();
      map.createMap();
    }
    vector<int> inOut;
    inOut.push_back(in_door.x);
    inOut.push_back(in_door.y);
    for ( int i=0 ; i<out_doors.size() ; i++ ) {
      inOut.push_back(out_doors[i].x);
      inOut.push_back(out_doors[i].y);
    }
    if ( map.blocks.size()>0 && enter_flag && exit_flag ) {
      for ( int i=0 ; i<out_doors.size() ; i++ ) {
        cout<<"Exit: "<<out_doors[i]<<endl;
      }
      Search search(inOut, map);
      search.aStar();
      search.drawLine(InImage);
    }
    cv::imshow("map",map.getImage());
    if(waitKey(100) >= 0) break;
    //show input with augmented information
    cv::imshow("in",InImage);
    int key = waitKey(30);
    if ( key == 'q' )
      break;
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
#include <queue>
#include <iostream>
#include <cmath>
#include <vector>
#include <PPMap.h>
#include <opencv2/opencv.hpp>

using namespace std;

class Node{
	public:
		// Parameter
		int name;
		int x, y;
		double cost;
		double priority;
		int parent;	
		
		// Constructor
		Node(){
		}
		Node(int nName, int nX, int nY, double nCost, double nPriority, int nParent){
			name = nName;
			x = nX;
			y = nY;
			cost = nCost;
			priority = nPriority;
			parent = nParent;
		}
		Node(int nName, int nX, int nY, double nCost, double nPriority){
			name = nName;
			x = nX;
			y = nY;
			cost = nCost;
			priority = nPriority;
		}
		// Overloading operator
		Node& operator=(const Node& n){
			name = n.name;
			priority = n.priority;
			x = n.x;
			y = n.y;
			cost = n.cost;
			parent = n.parent;
      return *this;
		}		
};

class ComparePriority{
	public:
		bool operator()(Node& n1, Node& n2){
			if(n1.priority > n2.priority) return true;
			else return false;
		}
};

class Search{
	public:
		// Parameter
		int mapSize; // how many blocks in total
		int st, en; // start and end block number
		vector<int> startEnd;// startPoint, endPoint
		vector<int> path; // final optimal path
		vector<Point> pointPath;
		vector<ppMapBlock> blocks;  
		vector<Node> visitedNode; 	
		vector< vector<int> > map; // map connected info matrix	
    vector<vector<Point2i>> exitList;
		// Constructor
		Search(vector<int> sten, const ppMap& pMap){
			startEnd = sten;
			map = pMap.map;
			mapSize = pMap.map.size();
			blocks = pMap.blocks;
      exitList.resize(mapSize,vector<Point2i>());
			st = findBlock(sten[0], sten[1]);
      for ( int i=2 ; i<sten.size()-1 ; i++ ) {
			  en = findBlock(sten[i], sten[i+1]);
        exitList[en].push_back(Point2i(sten[i], sten[i+1]));
      }
		}

		// Function
		/* Calculate the cost from start to here */
		double calCost(const Node& curNode, int i){
			return curNode.cost + sqrt(pow(curNode.x - blocks[i].center.x, 2.0) + pow(curNode.y - blocks[i].center.y, 2.0));	
		}
		/* Calculate the dist from here to end */
		double calDist(int i){
			return sqrt(pow(blocks[i].center.x - blocks[en].center.x, 2.0) + pow(blocks[i].center.y - blocks[en].center.y, 2.0));
		}
		/* Whether this node is goal or not */
		bool isGoal(const Node& n){
			if( exitList[n.name].size()>0 ) return true;
			else return false;
		}
		/* Whether this node is already existed */
		bool isExist(const Node& n){
			for(int i = 0; i < visitedNode.size(); i++){
				Node tmpNode = visitedNode.at(i);
				if(n.name == tmpNode.name) return true;
			}
			return false;
		}
		/* Get the path from start to end */
		void getPath(Node& n){
			path.insert(path.begin(), n.name);
			int pa = n.parent;
			while(pa != -1){
				path.insert(path.begin(), pa);				
				for(int i = 0; i < visitedNode.size(); i++){
					Node tmpNode = visitedNode.at(i);
					if(tmpNode.name == pa) pa = tmpNode.parent;
				}
			}
		}
		/* A star algorithm */
		void aStar(){
			bool reach = false;
			double tmpCost, tmpDist, tmpPriority;
			priority_queue<Node, vector<Node>, ComparePriority> queue; // declare a priority queue
			Node startNode(st, startEnd.at(0), startEnd.at(1), 0.0, 0.0, -1);
			queue.push(startNode); // push startNode into priority queue
			while(!queue.empty() && !reach){
				Node currentNode = queue.top();
				/* If currentNode is goal then getPath and return path */
				if(isGoal(currentNode)){
          en = currentNode.name;
					cout << "Find path!!" << endl;
					getPath(currentNode);
					reach = true;
					break;
				}
				visitedNode.push_back(currentNode);
				queue.pop();
				for(int i = 0; i < mapSize; i++){
					if(map[currentNode.name][i] != 0){
						/* Calculate newNode's cost and priority */
						tmpCost = calCost(currentNode, i);
						tmpDist = calDist(i);
						tmpPriority = tmpCost + tmpDist;
						/* Create a new node */
						Node childNode(i, blocks[i].center.x, blocks[i].center.y, tmpCost, tmpPriority, currentNode.name);
						/* If new Node isn't goal and doesn't exist push newNode into queue and mark as visited*/
						if(!isExist(childNode)){
							queue.push(childNode);
							visitedNode.push_back(childNode);
						}
					}
				}
			}
		}
		
		/* draw the path line and push the coordinate into pointPath*/
		void drawLine( Mat& img){
			vector<int> linePoint;
			int pos1, pos2, x, y;
			Point p1, p2, p3;
			if(st == en){
				p1 = cvPoint(startEnd[0], startEnd[1]); //startPoint
        p2 = exitList[en][0]; //EndPoint
				pointPath.push_back(p1); 
				pointPath.push_back(p2);  	
				line(img, p1, p2, cvScalar(255,0 ,0), 2, 3, 0);
			}
			else{
				p1 = cvPoint(startEnd.at(0), startEnd.at(1)); //startPoint
				p2 = cvPoint(blocks[st].center.x, blocks[st].center.y); //startBlockCenter
				pointPath.push_back(p1);
				pointPath.push_back(p2);	
				line(img, p1, p2, cvScalar(255,0 ,0), 2, 3, 0);
			}
			for(int i = 0; i < path.size(); i++){
				if(i + 1 < path.size()){ // judge whether next node is the final node
					pos1 = path.at(i);
					pos2 = path.at(i + 1);
					linePoint = findLine(blocks[pos1], blocks[pos2]);
					x = linePoint.at(0);
					y = linePoint.at(1);	
			 		p1 = cvPoint(blocks[pos1].center.x, blocks[pos1].center.y); // centerOfFirstPoint
			 		p2 = cvPoint(x, y); // midPoint of the line
			 		p3 = cvPoint(blocks[pos2].center.x, blocks[pos2].center.y); // centerOfSecondPoint
					pointPath.push_back(p2);
					pointPath.push_back(p3);
					line(img, p1, p2, cvScalar(255, 0, 0), 2, 3, 0);
					line(img, p2, p3, cvScalar(255, 0, 0), 2, 3, 0);
					linePoint.clear();
				}
			}
			if(st != en){
				p1 = cvPoint(blocks[path.back()].center.x, blocks[path.back()].center.y);
        p2 = exitList[en][0]; //EndPoint;
				pointPath.push_back(p2);
				line(img, p1, p2, cvScalar(255, 0, 0), 2, 3, 0);
			}
		}
		
		/* find the edge between two blocks */
		vector<int> findLine(const ppMapBlock& b1, const ppMapBlock& b2){
			vector<int> lineCenter;
			for(int i = 0; i < b1.points.size() - 1; i++){
				for(int j = i + 1; j < b1.points.size(); j++){
					for(int p = 0; p < b2.points.size() - 1; p++){
						for(int q = p + 1; q < b2.points.size(); q++){
							if((b1.points[i] == b2.points[p] && b1.points[j] == b2.points[q]) ||
							   (b1.points[j] ==  b2.points[p] && b1.points[i] ==  b2.points[q])){
								lineCenter.push_back((b1.points[i].x + b1.points[j].x) / 2);
								lineCenter.push_back((b1.points[i].y + b1.points[j].y) / 2);
								return lineCenter;	
							}
						}
					}
				}
			}
			return lineCenter;
		}

		/* Find which block is the point in */	
		int findBlock(int xx, int yy){
			ppPoint p(xx, yy, 0);
			for(int i = 0; i < mapSize; i++){
				int pointSize = blocks[i].points.size();
				for(int j = 0; j < pointSize; j++){
					if(j < pointSize - 1){
						if(!onRight(blocks[i].points[j], blocks[i].points[j + 1], p)) break;
					}
					else{
						if(!onRight(blocks[i].points[j], blocks[i].points[0], p)) break;
						else return i;
					}
				}
			}
			return 0;
		}
};


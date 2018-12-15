#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include<bits/stdc++.h>
#include <queue>
#include <ctime>
#include <climits>

using namespace cv;
using namespace std;

Mat binary(Mat a)
{
	int i,j;
	for(i = 0; i < a.rows; i++)
	{
		for(j = 0; j < a.cols; j++)
		{
			if(a.at<uchar>(i,j) >= 127)
				a.at<uchar>(i,j) = 255;
			else
				a.at<uchar>(i,j) = 0;
		}
	}
	return a;
}

typedef struct neighbor_node
{
	Point_<float> curr;
	int index;
	float dist;
}neighbor_node;

typedef struct node
{
	int x,y;
	vector< neighbor_node > neighbors;
	float cost;
	int id;

}node;

bool dist_compare(const neighbor_node& struct1, const neighbor_node& struct2)
{
    return (struct1.dist < struct2.dist);
}

int is_obs_present(int p1x, int p1y, int p2x, int p2y, Mat a)
{
	float d = sqrt(pow(p1x - p2x,2) + pow(p1y - p2y,2));
	int p3x, p3y;
	int i;
	for(i = 0; i < d; i++)
	{
		p3x = (i*p2x + (d-i)*p1x)/d;
		p3y = (i*p2y + (d-i)*p1y)/d;
		if(a.at<uchar>(p3x,p3y) == 255)
			return 1;
	}
	return 0;
}

int index_of_best_node(int source_id, vector<float> &dist, vector<int> &visited, vector<node> &graph)
{
	int i,j = -1; float mindist = INT_MAX, d;
	for(i = 0; i < graph.size(); i++)
	{
		if(dist[i] < mindist && visited[i] == 0)
		{
			mindist = dist[i];
			j = i;
			cout<<"j = "<<j<<endl;
		}
	}
	return j;
}

float Euclid(int p1x, int p1y, int p2x, int p2y)
{
	return sqrt(pow(p1x - p2x,2) + pow(p1y - p2y,2));
}

void draw_final_path(int source_id, int dest_id, vector<node> &graph, vector<int> &baap_ka_index,Mat a)
{
	Point2d curr_node;
	curr_node.x = graph[dest_id].x;
	curr_node.y = graph[dest_id].y;
	Point2d baap_node;
	int curr_index = dest_id;
	int baap_index = baap_ka_index[curr_index];
	while(1)
	{
		if(curr_node.x == graph[source_id].x && curr_node.y == graph[source_id].y)
		{
			break;
		}
		baap_node.x = graph[baap_index].x;
		baap_node.y = graph[baap_index].y;
		Point2d start, end;
		start.x = curr_node.y;
		start.y = curr_node.x;
		end.x = baap_node.y;
		end.y = baap_node.x;
		line(a, start, end, 255, 2, 8);
		curr_node = baap_node;
		curr_index = baap_index;
		baap_index = baap_ka_index[curr_index];
	}
}

void dijkstra(int source_id, int dest_id, vector<node> &graph, Mat a)
{
	vector< int > visited(graph.size());
	int i,j,k;
	vector< float > dist(graph.size());
	vector<int> baap_ka_index(graph.size());
	for(i = 0; i < graph.size(); i++)
	{
		dist[i] = INT_MAX;
		visited[i] = 0;
		baap_ka_index[i] = -1;
		if(i == source_id)
			dist[i] = 0;
	}
	int curr_node_id = -1;
	int prev_node_id = -1;
	while(1)
	{
		cout<<"Dijkstra running...."<<endl;
		prev_node_id = curr_node_id;
		curr_node_id = index_of_best_node(source_id, dist, visited, graph);
		cout<<"curr_node_id = "<<curr_node_id<<endl;
		//cout<<graph[curr_node_id].x<<" "<<graph[curr_node_id].y<<endl;
		if(curr_node_id == -1)
		{
			int m = 0;
			for(m = 0; m < graph.size(); m++)
			{
				if(visited[m] == 0)
				{
					curr_node_id = m;
					cout<<"New connected component found"<<endl;
					dist[m] = dist[prev_node_id] + Euclid(graph[m].x, graph[m].y, graph[prev_node_id].x, graph[prev_node_id].y);
					baap_ka_index[m] = prev_node_id;
					break;
				}
			}
		}
		if(curr_node_id == dest_id) break;
		for(j = 0; j < graph[curr_node_id].neighbors.size(); j++)
		{
			//cout<<"neighbors :"<<graph[curr_node_id].neighbors[j].curr.x<<" "<<graph[curr_node_id].neighbors[j].curr.y<<endl;
			if(!is_obs_present(graph[curr_node_id].x, graph[curr_node_id].y, graph[curr_node_id].neighbors[j].curr.x, graph[curr_node_id].neighbors[j].curr.y, a))
			{
				//cout<<"Ellooo "<<graph[curr_node_id].neighbors[j].index<<endl;
				if(!visited[graph[curr_node_id].neighbors[j].index])
				{
					node temp;
					temp.x = graph[curr_node_id].neighbors[j].curr.x;
					temp.y = graph[curr_node_id].neighbors[j].curr.y;
					if(dist[curr_node_id] + Euclid(graph[curr_node_id].x,graph[curr_node_id].y, temp.x, temp.y) < dist[graph[curr_node_id].neighbors[j].index])
					{
						dist[graph[curr_node_id].neighbors[j].index] = dist[curr_node_id] + Euclid(graph[curr_node_id].x,graph[curr_node_id].y, temp.x, temp.y);
						baap_ka_index[graph[curr_node_id].neighbors[j].index] = curr_node_id;
						//cout<<"padhos: "<<graph[curr_node_id].neighbors[j].index<<endl;
					}
				}
			}
		}
		visited[curr_node_id] = 1;
	}
	draw_final_path(source_id, dest_id, graph, baap_ka_index, a);
}


int main()
{

	Mat img = imread("Untitled1.png", 0);
	img = binary(img);
	cout<<img.rows<<" "<<img.cols<<endl;
	int N,i,j,K, num =0;
	// cout<<"Enter number of initial nodes in Roadmap: "<<endl;
	// cin>>N;
	N = 3600;
	cout<<"Enter the value of K to find K nearest neighbors: "<<endl;
	cin>>K;
	srand(time(0));
	vector<node> graph;
	while(graph.size() < N)
	{
		node new_point;
		new_point.x = rand()%img.rows;
		new_point.y = rand()%img.cols;
		if(img.at<uchar>(new_point.x, new_point.y) != 255)
		{
			new_point.id = graph.size();
			graph.push_back(new_point);
			//img.at<uchar>(new_point.x, new_point.y) = 255;
		}
	}
	// while(1)
	// {
	// 	int flag= waitKey(10);
	// 	if(flag == 27) break;
	// }
	for(i = 0; i < N; i++)
	{
		vector<neighbor_node> distances;
		// while(graph[i].neighbors.size() < K)
		// {
		// 	// node rand_point;
		// 	// rand_point.x = rand()%a.rows;
		// 	// rand_point.y = rand()%a.cols;
		// 	// dist = pow(rand_point.x - graph[i].x,2) + pow(rand_point.y - graph[i].y,2)
		// 	// 
		// 	// padhos_wala.x = (radius*rand_point.x + (dist- radius)*graph[i].x)/dist;
		// 	// padhos_wala.y = (radius*rand_point.y + (dist- radius)*graph[i].y)/dist;
		// 	// if(img.at<uchar>(padhos_wala.x, padhos_wala.y) != 255)
		// 	// {
		// 	// 	graph[i].neighbors.push_back(padhos_wala);
		// 	// }


		// }
		neighbor_node padhos_wala;
		for(j = 0; j < N; j++)
		{
			if(i !=j )
			{
				float d = sqrt(pow(graph[j].x - graph[i].x,2) + pow(graph[j].y - graph[i].y,2));
				padhos_wala.curr.x = graph[j].x;
				padhos_wala.curr.y = graph[j].y;
				padhos_wala.dist = d;
				padhos_wala.index = j;
				distances.push_back(padhos_wala);
			}
		}
		sort(distances.begin(), distances.end(), dist_compare);
		for(j = 0; j < N-1; j++)
		{
			if(!is_obs_present(graph[i].x, graph[i].y, distances[j].curr.x, distances[j].curr.y, img))
				graph[i].neighbors.push_back(distances[j]);
			if(graph[i].neighbors.size() >= K)
				break;
		}
		distances.clear();
	}

	cout<<"Enter the coordinates of Source :"<<endl;
	node source, dest;
	cin>>source.x>>source.y;
	cout<<"Enter the coordinates of Destination :"<<endl;
	cin>>dest.x>>dest.y;
	graph.push_back(source);
	graph.push_back(dest);
	vector<neighbor_node> src_distances;
	vector<neighbor_node> dest_distances;
	i = N;
	for(j = 0; j < N; j++)
	{
		if(i !=j )
		{
			neighbor_node padhos_wala;
			float d = sqrt(pow(graph[j].x - graph[i].x,2) + pow(graph[j].y - graph[i].y,2));
			padhos_wala.curr.x = graph[j].x;
			padhos_wala.curr.y = graph[j].y;
			padhos_wala.dist = d;
			padhos_wala.index = j;
			src_distances.push_back(padhos_wala);
		}
	}
	sort(src_distances.begin(), src_distances.end(), dist_compare);
	// for(j = 0; j < N-1; j++)
	// {
	// 	cout<<src_distances[j].dist<<endl;
	// }
	for(j = 0; j < N-1; j++)
	{
		if(!is_obs_present(graph[i].x, graph[i].y, src_distances[j].curr.x, src_distances[j].curr.y, img))
			graph[i].neighbors.push_back(src_distances[j]);
		if(graph[i].neighbors.size() >= K)
			break;
	}
	i = N+1;
	for(j = 0; j < N; j++)
	{
		if(i !=j )
		{
			neighbor_node padhos_wala;
			float d = sqrt(pow(graph[j].x - graph[i].x,2) + pow(graph[j].y - graph[i].y,2));
			padhos_wala.curr.x = graph[j].x;
			padhos_wala.curr.y = graph[j].y;
			padhos_wala.dist = d;
			padhos_wala.index = j;
			dest_distances.push_back(padhos_wala);
		}
	}
	sort(dest_distances.begin(), dest_distances.end(), dist_compare);
	for(j = 0; j < N-1; j++)
	{
		if(!is_obs_present(graph[i].x, graph[i].y, dest_distances[j].curr.x, dest_distances[j].curr.y, img))
			graph[i].neighbors.push_back(dest_distances[j]);
		if(graph[i].neighbors.size() >= K)
			break;
	}
	//cout<<"N = "<<N<<" graph = "<<graph.size()<<endl;
	//vector<int> baap_ka_index(N+2);
	
	dijkstra(N, N+1, graph, img);
	namedWindow("Roadmap", WINDOW_NORMAL);
	imshow("Roadmap", img);
	while(1)
	{
	 	int flag= waitKey(10);
	 	if(flag == 27) break;
	}
	return 0;
}
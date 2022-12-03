
#include <bits/stdc++.h>
#include <vector>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <map>
#include <cstdio>
#include <cstring>
#include <functional>
using namespace std;
/*
#先输入边信息（edge), 再输入道路等级信息（wayType), 最后输入轨迹信息（trajectory）， 中间没有空行。
N # 表示有多少条路段 (int)
id p1 p2 way_string way_type c x_1 y_1 x_2 y_2 .... x_c y_c # 接下来N行 每行一个路段信息： 路段id (int) 路段起点id (int) 路段终点id (int) 路段等级名称 way_string (str) 路段等级 way_type (int) c (int) 个地理采样点连起来构成这个路段 (x_1,y_1) -> (x_2,y_2) -> ... -> (x_c,y_c) 表示这c个经纬度坐标 (double, double)
....
M # 表示多少条轨迹
timestamp1 x1 y1 # 表示第一条轨迹的第一个时间戳， 经纬度 （int, double, double) 时间戳单调递增
timestamp2 x2 y2 # 表示第一条轨迹的第二个时间戳， 经纬度 （int, double, double)
....
timestampk xk yk
0 # 表示第0条轨迹输入完毕
timestamp1 x1 y1 # 表示第二个轨迹的第一个时间戳， 经纬度 （int, double, double)
timestamp2 x2 y2
....
timestampt xt yt
1 # 表示第1条轨迹输入完毕
...
...
...
timestamp1 x1 y1 # 表示第M个轨迹的第一个时间戳， 经纬度 （int, double, double)
timestamp2 x2 y2
....
timestampt xt yt
M-1 # 表示第M条轨迹输入完毕
*/
/*输出
M #表示有多少条轨迹 接下来每一行一条轨迹的匹配路段序列 与输入轨迹的顺序对应
r_1 r_2 ... r_k #表示第一条轨迹每个点匹配的路段id 序列 （int，int，..., int）
r_1 r_2 ... r_t #表示第二条轨迹每个点匹配的路段id 序列 （int，int，..., int）
....
r_1 r_2 ... r_t #表示第M条轨迹每个点匹配的路段id 序列 （int，int，..., int）
保证同一轨迹的时间戳按照时间顺序给出。
*/
double r = 0.004;
int N, M;
//计算每段路径的长度
inline double lenth_cal(pair<double, double> a, pair<double, double> b)
{
    return sqrt((a.first - b.first) * (a.first - b.first) + (a.second - b.second) * (a.second - b.second));
}

typedef struct Edge
{
    int id;
    int p1;
    int p2;
    char way_string[20];
    int way_type;
    int c;
    vector<pair<double, double>> points;
    double length;
} Edge;

vector<Edge> edges(121281);

typedef struct Trajectory
{
    int timestamp;
    double x;
    double y;
} Trajectory;

typedef struct Rec
{
    double x1;
    double y1;
    double x2;
    double y2;
} Rec;

// RTree的节点
typedef struct RTreeNode
{
    int id = -1; //边的id,-1表示非叶节点。
    RTreeNode *parent = NULL;
    Rec rec = {150, 150, 0, 0}; //矩形
    int child_num = 0;
    vector<RTreeNode *> children = {};

} RTreeNode;

vector<RTreeNode *> leaf_nodes(121281);
RTreeNode *root;

typedef struct candidate
{
    int id;
    double distance;
    double length;
    double x;
    double y; //投影点坐标
} candidate;

void distance_cal(int id, double x, double y, candidate &c)
{
    // distance 为(x,y)到编号为id的路段的最短距离，length为(x,y)到编号为id的路段的最短距离的点沿该路段到路段起点的距离
    double min_distance = 1e9;
    double min_length = 0;
    double min_x = 0;
    double min_y = 0;
    for (int i = 0; i < edges[id].points.size() - 1; i++)
    {
        double x1 = edges[id].points[i].first;
        double y1 = edges[id].points[i].second;
        double x2 = edges[id].points[i + 1].first;
        double y2 = edges[id].points[i + 1].second;
        double a = x - x1;
        double b = y - y1;
        double c = x2 - x1;
        double d = y2 - y1;
        double dot = a * c + b * d;
        double len_sq = c * c + d * d;
        double param = -1;
        if (len_sq != 0)
            param = dot / len_sq;
        double xx, yy;
        if (param < 0)
        {
            xx = x1;
            yy = y1;
        }
        else if (param > 1)
        {
            xx = x2;
            yy = y2;
        }
        else
        {
            xx = x1 + param * c;
            yy = y1 + param * d;
        }
        double dis = sqrt((x - xx) * (x - xx) + (y - yy) * (y - yy));
        if (dis < min_distance)
        {
            min_distance = dis;
            min_length = lenth_cal(edges[id].points[i], make_pair(xx, yy));
            min_x = xx;
            min_y = yy;
        }
    }
    c.distance = min_distance;
    c.length = min_length;
    c.x = min_x;
    c.y = min_y;
}

inline void leaf_nodes_init()
{
    for (int i = 0; i < 121281; i++)
    {
        leaf_nodes[i] = new RTreeNode;
        leaf_nodes[i]->id = i;
        leaf_nodes[i]->rec.x1 = min(edges[i].points[0].first, edges[i].points[edges[i].c - 1].first);
        leaf_nodes[i]->rec.y1 = min(edges[i].points[0].second, edges[i].points[edges[i].c - 1].second);
        leaf_nodes[i]->rec.x2 = max(edges[i].points[edges[i].c - 1].first, edges[i].points[0].first);
        leaf_nodes[i]->rec.y2 = max(edges[i].points[edges[i].c - 1].second, edges[i].points[0].second);
    }
    sort(leaf_nodes.begin(), leaf_nodes.end(), [](RTreeNode *a, RTreeNode *b)
         { return a->rec.x1 < b->rec.x1; });
}

// Sort-Tile-Recursive algorithm
void build_RTree(vector<RTreeNode *> &nodes)
{
    if (nodes.size() == 1)
    {
        root = nodes[0];
        return;
    }
    vector<RTreeNode *> parent_nodes(nodes.size() / 11 + 1);
    for (int i = 0; i < parent_nodes.size(); i++)
    {
        parent_nodes[i] = new RTreeNode;
    }
    for (int i = 0; i < nodes.size(); i++)
    {
        nodes[i]->parent = parent_nodes[i / 11];
        parent_nodes[i / 11]->children.push_back(nodes[i]);
        parent_nodes[i / 11]->child_num++;
        parent_nodes[i / 11]->rec.x1 = min(parent_nodes[i / 11]->rec.x1, nodes[i]->rec.x1);
        parent_nodes[i / 11]->rec.y1 = min(parent_nodes[i / 11]->rec.y1, nodes[i]->rec.y1);
        parent_nodes[i / 11]->rec.x2 = max(parent_nodes[i / 11]->rec.x2, nodes[i]->rec.x2);
        parent_nodes[i / 11]->rec.y2 = max(parent_nodes[i / 11]->rec.y2, nodes[i]->rec.y2);
    }
    build_RTree(parent_nodes);
}
//判断两个矩形是否相交
inline bool is_intersect(Rec a, Rec b)
{
    if (a.x1 > b.x2 || a.x2 < b.x1 || a.y1 > b.y2 || a.y2 < b.y1)
        return false;
    return true;
}
// RTree的查询
void RTree_query(RTreeNode *node, Rec rec, vector<candidate> &result)
{
    if (node->id != -1)
    {
        candidate c;
        c.id = node->id;
        distance_cal(node->id, (rec.x1 + rec.x2) / 2, (rec.y1 + rec.y2) / 2, c);
        result.push_back(c);
        return;
    }
    for (int i = 0; i < node->child_num; i++)
    {
        if (is_intersect(node->children[i]->rec, rec))
        {
            RTree_query(node->children[i], rec, result);
        }
    }
}



int main()
{
    freopen("E://c++codes//PJ//sample.in", "r", stdin);
    scanf("%d", &N);
    for (int i = 0; i < N; i++)
    {
        Edge edge;
        scanf("%d%d%d%s%d%d", &edge.id, &edge.p1, &edge.p2, &edge.way_string, &edge.way_type, &edge.c);
        for (int j = 0; j < edge.c; j++)
        {
            double x, y;
            scanf("%lf %lf", &x, &y);
            edge.points.push_back(make_pair(x, y));
        }
        for (int j = 0; j < edge.c - 1; j++)
        {
            edge.length += lenth_cal(edge.points[j], edge.points[j + 1]);
        }
        edges[i] = edge;
    }
    leaf_nodes_init();
    root = new RTreeNode;
    build_RTree(leaf_nodes);
    scanf("%d", &M);
    printf("%d\n", M);
    for (int i = 0; i < M; i++)
    {
        int timestamp;
        double x, y;
        vector<Trajectory> trajectories;
        while (scanf("%d", &timestamp) != EOF)
        {
            if (timestamp == i)
            {
                break;
            }
            scanf("%lf %lf", &x, &y);
            Trajectory trajectory;
            trajectory.timestamp = timestamp;
            trajectory.x = x;
            trajectory.y = y;
            trajectories.push_back(trajectory);
        }
        vector<int> result(trajectories.size());
        vector<vector<candidate>> candidates(trajectories.size());
        for (int j = 0; j < trajectories.size(); j++)
        {
            Rec rec;
            rec.x1 = trajectories[j].x - r;
            rec.y1 = trajectories[j].y - r;
            rec.x2 = trajectories[j].x + r;
            rec.y2 = trajectories[j].y + r;
            RTree_query(root, rec, candidates[j]);
            sort(candidates[j].begin(), candidates[j].end(), [](candidate a, candidate b)
                 { return a.distance < b.distance; });
        }

        for (int j = 0; j < trajectories.size(); j++)
        {
            result[j] = candidates[j][0].id;
        }
        for (int j = 0; j < result.size(); j++)
        {
            printf("%d ", result[j]);
        }
        printf("\n");
    }
    return 0;
}
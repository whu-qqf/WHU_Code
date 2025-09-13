#include<iostream>
#include<string>
#include<vector>
#include<map>
using namespace std;

struct Node       //节点结构体
{
    string name;  //节点名
} node[4];        //四个节点
struct Arc        //弧段结构体
{ 
    int id;       //弧段ID
    Node* start;  //弧段起始点
    Node* end;    //弧段终点
} arc[6];         //六个弧段
struct Polygon            //多边形结构体
{  
    int id;               //多边形ID 
    vector<int> arc_ids;  //存放多边形的边的向量
};
vector<Polygon> polygons; //存放多边形的向量
struct ArcPolyInfo     //弧段左右多边形信息
{ 
    int left = -1;     //用数字代表多边形，-1表示未分配
    int right = -1;    //若有数字，则表明左右多边形为ID为此数字的多边形
};
map<int, ArcPolyInfo> arcPolyMap;    //映射，弧段和弧段左右多边形信息的映射
int node_arcs[4][3] = {  //节点及其周围弧段（顺时针）
    {3,2,1},             // A
    {4,6,1},             // B
    {2,5,4},             // C
    {3,6,5}              // D
};
int GetNodeIdx(Node* n)               // 获取节点索引函数 
{
    for (int i = 0; i < 4; ++i)       //遍历四个节点，找到与n对应的节点
        if (&node[i] == n) return i;  //0->A ,1->B ,2->C ,3->D
    return -1;
}

int FindNextArc(int nodeIdx/*节点下标*/, int ArcId/*当前弧段ID*/)    // 顺时针查找下一个弧段函数
{
    for (int i = 0; i < 3; ++i)
    {
        if (node_arcs[nodeIdx][i] == ArcId)            //在节点下标下依据ArcID寻找弧段
            return node_arcs[nodeIdx][(i + 1) % 3];    //返回下一条弧段ID
    }
    return -1;
}
Arc* FindArc(int id)                          //通过ID查找弧段 
{  
    for (int i = 0; i < 6; ++i)               //在六条弧段中寻找
        if (arc[i].id == id) return &arc[i];  //返回弧段指针
    return nullptr;
}
void BuildPolygon() {              //构建多边形函数
    int polyCount = 0;              //多边形计数
    for (int i = 0; i < 6; i++) {   //遍历六条弧段
        int aid = arc[i].id;        //当前弧段ID
        if (arcPolyMap[aid].left != -1 && arcPolyMap[aid].right != -1) continue;    //判断左右多边形是否为空，若都不为空则跳到下一条弧段
        polyCount++;                        //多边形计数+1
        if (arcPolyMap[aid].left == -1)     //若左多边形为空
        {
			Polygon poly;                   //创建一个左多边形
            poly.id = polyCount;            //多边形ID为计数
            Arc* cArc = &arc[i];            //当前弧段
            Node* sNode = cArc->start;      //弧段开始节点记为多边形起始点
            Node* cNode = cArc->end;        //把当前弧段终点记为当前节点
            poly.arc_ids.push_back(cArc->id);       //把当前弧段加入到多边形中
            arcPolyMap[cArc->id].left = poly.id;    //记录左多边形
            while (cNode != sNode)                  //只要当前节点不为开始节点，循环找寻
            {
                int nodeIdx = GetNodeIdx(cNode);    //获取节点索引
                int nextArcId = FindNextArc(nodeIdx, cArc->id);  //根据节点索引和当前弧段寻找该节点下一条弧段（顺时针）
                Arc* nextArc = FindArc(nextArcId);        //根据弧段ID找到弧段
                poly.arc_ids.push_back(nextArcId);        //把该弧段加入到多边形中
                if (cNode == nextArc->start)              //判断当前节点是否为此弧段的开始点
                    arcPolyMap[nextArcId].left = poly.id; //若是，则多边形为此弧段左多边形
                else
                    arcPolyMap[nextArcId].right = poly.id;//不是，则为右多边形
                cNode = (nextArc->start == cNode) ? nextArc->end : nextArc->start;  //更新当前节点，当前节点为nextArc的另一端点
                cArc = nextArc;                           //当前弧段更新为下一条弧段
            }
            polygons.push_back(poly);                     //把此多边形加入到多边形向量中
            continue;                                     //结束此次循环，查找下一条弧段
        } 
        if (arcPolyMap[aid].right == -1)        //如果右多边形为空
        {
            Polygon poly;                         //创建一个右多边形
            poly.id = polyCount;                  //多边形ID
            Arc* cArc = &arc[i];                  //设置当前弧段
            Node* sNode = cArc->end;              //弧段终点记为多边形起始点
            Node* cNode = cArc->start;            //把当前弧段开始点记为当前节点
            poly.arc_ids.push_back(cArc->id);     //把当前弧段加入到该多边形中 
            arcPolyMap[cArc->id].right = poly.id; //记录为右多边形
            while (cNode != sNode)                //只要当前节点不为开始节点，循环找寻
            {
                int nodeIdx = GetNodeIdx(cNode);  //获取节点索引
                int nextArcId = FindNextArc(nodeIdx, cArc->id);  //根据节点索引和当前弧段寻找该节点下一条弧段（顺时针）
                Arc* nextArc = FindArc(nextArcId);               //根据弧段ID找到弧段
                poly.arc_ids.push_back(nextArcId);               //把该弧段加入到多边形中
                if (cNode == nextArc->start)                     //判断当前节点是否为此弧段的开始点
                    arcPolyMap[nextArcId].left = poly.id;        //若是，则多边形为此弧段左多边形
                else
                    arcPolyMap[nextArcId].right = poly.id;       //不是，则为右多边形
                cNode = (nextArc->start == cNode) ? nextArc->end : nextArc->start;   ////更新当前节点，当前节点为nextArc的另一端点
                cArc = nextArc;                   //当前弧段更新为下一条弧段
            }
            polygons.push_back(poly);             //把此多边形加入到多边形向量中
            continue;                             //结束此次循环，查找下一条弧段
        }
    }
}

int main()
{
    node[0].name = "A";    // 初始化节点
    node[1].name = "B";
    node[2].name = "C";
    node[3].name = "D";
    arc[0] = { 1, &node[0], &node[1] };     // 初始化弧段
    arc[1] = { 2, &node[2], &node[0] };
    arc[2] = { 3, &node[3], &node[0] };
    arc[3] = { 4, &node[1], &node[2] };
    arc[4] = { 5, &node[2], &node[3] };
    arc[5] = { 6, &node[1], &node[3] };
    BuildPolygon();   //多边形构建 
    cout << "弧段  左多边形  右多边形" << endl;    //输出结果
    for (int i = 1; i <= 6; ++i) 
    {
        cout << i << "      ";
        if (arcPolyMap[i].left != -1)
            cout << "P" << arcPolyMap[i].left << "        ";
        else
            cout << "         ";
        if (arcPolyMap[i].right != -1)
            cout << "P" << arcPolyMap[i].right;
        cout << endl;
    }
    return 0;
}
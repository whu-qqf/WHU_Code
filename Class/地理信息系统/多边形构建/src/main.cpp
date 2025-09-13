#include<iostream>
#include<string>
#include<vector>
#include<map>
using namespace std;

struct Node       //�ڵ�ṹ��
{
    string name;  //�ڵ���
} node[4];        //�ĸ��ڵ�
struct Arc        //���νṹ��
{ 
    int id;       //����ID
    Node* start;  //������ʼ��
    Node* end;    //�����յ�
} arc[6];         //��������
struct Polygon            //����νṹ��
{  
    int id;               //�����ID 
    vector<int> arc_ids;  //��Ŷ���εıߵ�����
};
vector<Polygon> polygons; //��Ŷ���ε�����
struct ArcPolyInfo     //�������Ҷ������Ϣ
{ 
    int left = -1;     //�����ִ������Σ�-1��ʾδ����
    int right = -1;    //�������֣���������Ҷ����ΪIDΪ�����ֵĶ����
};
map<int, ArcPolyInfo> arcPolyMap;    //ӳ�䣬���κͻ������Ҷ������Ϣ��ӳ��
int node_arcs[4][3] = {  //�ڵ㼰����Χ���Σ�˳ʱ�룩
    {3,2,1},             // A
    {4,6,1},             // B
    {2,5,4},             // C
    {3,6,5}              // D
};
int GetNodeIdx(Node* n)               // ��ȡ�ڵ��������� 
{
    for (int i = 0; i < 4; ++i)       //�����ĸ��ڵ㣬�ҵ���n��Ӧ�Ľڵ�
        if (&node[i] == n) return i;  //0->A ,1->B ,2->C ,3->D
    return -1;
}

int FindNextArc(int nodeIdx/*�ڵ��±�*/, int ArcId/*��ǰ����ID*/)    // ˳ʱ�������һ�����κ���
{
    for (int i = 0; i < 3; ++i)
    {
        if (node_arcs[nodeIdx][i] == ArcId)            //�ڽڵ��±�������ArcIDѰ�һ���
            return node_arcs[nodeIdx][(i + 1) % 3];    //������һ������ID
    }
    return -1;
}
Arc* FindArc(int id)                          //ͨ��ID���һ��� 
{  
    for (int i = 0; i < 6; ++i)               //������������Ѱ��
        if (arc[i].id == id) return &arc[i];  //���ػ���ָ��
    return nullptr;
}
void BuildPolygon() {              //��������κ���
    int polyCount = 0;              //����μ���
    for (int i = 0; i < 6; i++) {   //������������
        int aid = arc[i].id;        //��ǰ����ID
        if (arcPolyMap[aid].left != -1 && arcPolyMap[aid].right != -1) continue;    //�ж����Ҷ�����Ƿ�Ϊ�գ�������Ϊ����������һ������
        polyCount++;                        //����μ���+1
        if (arcPolyMap[aid].left == -1)     //��������Ϊ��
        {
			Polygon poly;                   //����һ��������
            poly.id = polyCount;            //�����IDΪ����
            Arc* cArc = &arc[i];            //��ǰ����
            Node* sNode = cArc->start;      //���ο�ʼ�ڵ��Ϊ�������ʼ��
            Node* cNode = cArc->end;        //�ѵ�ǰ�����յ��Ϊ��ǰ�ڵ�
            poly.arc_ids.push_back(cArc->id);       //�ѵ�ǰ���μ��뵽�������
            arcPolyMap[cArc->id].left = poly.id;    //��¼������
            while (cNode != sNode)                  //ֻҪ��ǰ�ڵ㲻Ϊ��ʼ�ڵ㣬ѭ����Ѱ
            {
                int nodeIdx = GetNodeIdx(cNode);    //��ȡ�ڵ�����
                int nextArcId = FindNextArc(nodeIdx, cArc->id);  //���ݽڵ������͵�ǰ����Ѱ�Ҹýڵ���һ�����Σ�˳ʱ�룩
                Arc* nextArc = FindArc(nextArcId);        //���ݻ���ID�ҵ�����
                poly.arc_ids.push_back(nextArcId);        //�Ѹû��μ��뵽�������
                if (cNode == nextArc->start)              //�жϵ�ǰ�ڵ��Ƿ�Ϊ�˻��εĿ�ʼ��
                    arcPolyMap[nextArcId].left = poly.id; //���ǣ�������Ϊ�˻���������
                else
                    arcPolyMap[nextArcId].right = poly.id;//���ǣ���Ϊ�Ҷ����
                cNode = (nextArc->start == cNode) ? nextArc->end : nextArc->start;  //���µ�ǰ�ڵ㣬��ǰ�ڵ�ΪnextArc����һ�˵�
                cArc = nextArc;                           //��ǰ���θ���Ϊ��һ������
            }
            polygons.push_back(poly);                     //�Ѵ˶���μ��뵽�����������
            continue;                                     //�����˴�ѭ����������һ������
        } 
        if (arcPolyMap[aid].right == -1)        //����Ҷ����Ϊ��
        {
            Polygon poly;                         //����һ���Ҷ����
            poly.id = polyCount;                  //�����ID
            Arc* cArc = &arc[i];                  //���õ�ǰ����
            Node* sNode = cArc->end;              //�����յ��Ϊ�������ʼ��
            Node* cNode = cArc->start;            //�ѵ�ǰ���ο�ʼ���Ϊ��ǰ�ڵ�
            poly.arc_ids.push_back(cArc->id);     //�ѵ�ǰ���μ��뵽�ö������ 
            arcPolyMap[cArc->id].right = poly.id; //��¼Ϊ�Ҷ����
            while (cNode != sNode)                //ֻҪ��ǰ�ڵ㲻Ϊ��ʼ�ڵ㣬ѭ����Ѱ
            {
                int nodeIdx = GetNodeIdx(cNode);  //��ȡ�ڵ�����
                int nextArcId = FindNextArc(nodeIdx, cArc->id);  //���ݽڵ������͵�ǰ����Ѱ�Ҹýڵ���һ�����Σ�˳ʱ�룩
                Arc* nextArc = FindArc(nextArcId);               //���ݻ���ID�ҵ�����
                poly.arc_ids.push_back(nextArcId);               //�Ѹû��μ��뵽�������
                if (cNode == nextArc->start)                     //�жϵ�ǰ�ڵ��Ƿ�Ϊ�˻��εĿ�ʼ��
                    arcPolyMap[nextArcId].left = poly.id;        //���ǣ�������Ϊ�˻���������
                else
                    arcPolyMap[nextArcId].right = poly.id;       //���ǣ���Ϊ�Ҷ����
                cNode = (nextArc->start == cNode) ? nextArc->end : nextArc->start;   ////���µ�ǰ�ڵ㣬��ǰ�ڵ�ΪnextArc����һ�˵�
                cArc = nextArc;                   //��ǰ���θ���Ϊ��һ������
            }
            polygons.push_back(poly);             //�Ѵ˶���μ��뵽�����������
            continue;                             //�����˴�ѭ����������һ������
        }
    }
}

int main()
{
    node[0].name = "A";    // ��ʼ���ڵ�
    node[1].name = "B";
    node[2].name = "C";
    node[3].name = "D";
    arc[0] = { 1, &node[0], &node[1] };     // ��ʼ������
    arc[1] = { 2, &node[2], &node[0] };
    arc[2] = { 3, &node[3], &node[0] };
    arc[3] = { 4, &node[1], &node[2] };
    arc[4] = { 5, &node[2], &node[3] };
    arc[5] = { 6, &node[1], &node[3] };
    BuildPolygon();   //����ι��� 
    cout << "����  ������  �Ҷ����" << endl;    //������
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
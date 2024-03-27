#pragma once

#include <stdio.h>
//#include <stdarg.h>
//#include <direct.h> 
#include "EMst.h"
#include<tuple>

//template<typename T>
//void debug_notime(T t);
//template<typename T, typename... U>
//void debug_notime(T t, U... ts);
//template<typename T, typename... U>
//void logger(T t, U... ts);
//
//string s(int s);
//string  ss(int num);
//void quickSort(vector<int> &arr, int _left, int _right);
//void outputLineFile(p_Mypoint &P, vector<int> &nodeNumList, vector<int> &pathList, string name);
//void outputPolylineFile(p_Mypoint &P, vector<int> &nodeNumList, vector<int> &pathList, string name);
//void cleanFile(string strPath);
//void clear(vector<int> &v);
//void connect(vector<int> &v1, vector<int> &v2);
//void connectByMapping(vector<int> &v1, vector<int> &v2, vector<int> &map);
//void inverseMapping(vector<int> &v1, vector<int> &map);
//vector<int> createSequentialList(int first, int length);
//ostream & operator<<(ostream &cout, vector<int> &pathList);

template<typename T>
void clear(vector<T> &v)
{
	v.swap(vector<T>());
}

string s(int s)
{
	return to_string(s);
}

string ss(int num)
{
	return num > 9 ? s(num) : s(0) + s(num);
}


ostream & operator<<(ostream &cout, vector<int> &list) {
	//���Խ��
	string x = "";
	for (int i = 0; i < list.size(); i++)
	{
		x = x + to_string(list[i]) + ",";
	}
	cout << x;
	return cout;
}


template<typename T>
void debug_notime(T t)
{
	std::cout << t << endl;
}

template<typename T, typename... U>
void debug_notime(T t, U... ts) {
	std::cout << t ;
	debug_notime(ts...);
}

template<typename T, typename... U>
void logger(T t, U... ts)   
{
	time_t time_seconds = time(0);
	struct tm t_tm;
	localtime_s(&t_tm, &time_seconds);
	string   str = "[" + ss(t_tm.tm_hour) + ":" + ss(t_tm.tm_min) + ":" + ss(t_tm.tm_sec) + "] ";
	std::cout << str;
	debug_notime(t, ts...);
}

void quickSort(vector<int> &arr, int _left, int _right) {
	int left = _left;
	int right = _right;
	int temp = 0;
	if (left < right) {   //�������Ԫ�����������������
		temp = arr[left];  //������ĵ�һ��Ԫ����Ϊ��׼Ԫ��
		while (left != right) {   //���������߽���ɨ�裬ֱ��left = right

			while (right > left&& arr[right] >= temp)
				right--;        //��������ɨ�裬�ҵ���һ���Ȼ�׼Ԫ��С��Ԫ��
			arr[left] = arr[right];  //�ҵ�����Ԫ��arr[right]����arr[left]����

			while (left < right && arr[left] <= temp)
				left++;         //��������ɨ�裬�ҵ���һ���Ȼ�׼Ԫ�ش��Ԫ��
			arr[right] = arr[left];  //�ҵ�����Ԫ��arr[left]����arr[right]����

		}
		arr[right] = temp;    //��׼Ԫ�ع�λ
		quickSort(arr, _left, left - 1);  //�Ի�׼Ԫ����ߵ�Ԫ�ؽ��еݹ�����
		quickSort(arr, right + 1, _right);  //�Ի�׼Ԫ���ұߵĽ��еݹ�����
	}
}

void connectByMapping(vector<int> &v1, vector<int> &v2, vector<int> &map)
{
	for each (int num in v2)
	{
		v1.push_back(map[num]);
	}
}

void inverseMapping(vector<int> &v1, vector<int> &map) {
	for (int i = 0; i < v1.size(); i++)
	{
		v1[i] = map[v1[i]];
	}

}

void connect(vector<int> &v1, vector<int> &v2)
{
	v1.insert(v1.end(), v2.begin(), v2.end());
}



void outputLineFile(p_Mypoint &P, vector<int> &nodeNumList, vector<int> &pathList, string name)
{
	string filename = path;
	ofstream PLfile1(filename, ios::app);
	for (int i = 0; i < pathList.size() - 1; i++) {
		int start = nodeNumList[pathList[i]];
		int end = nodeNumList[pathList[i + 1]];
		PLfile1 << "GOCAD PLine 1\n";
		PLfile1 << "HEADER{\n";
		PLfile1 << "name:" << "line-" << name << "\n";
		PLfile1 << "}\n";
		PLfile1 << "ILINE\n";
		PLfile1 << "VRTX " << 0 << " " << P->points[start].x << " " << P->points[start].y << " " << P->points[start].z << "\n";
		PLfile1 << "VRTX " << 1 << " " << P->points[end].x << " " << P->points[end].y << " " << P->points[end].z << "\n";
		PLfile1 << "SEG " << 0 << " " << 1 << "\n";
		PLfile1 << "END";
	}
	PLfile1.close();
};

void outputPolylineFile(p_Mypoint &P, vector<int> &nodeNumList, vector<int> &pathList, string name)
{
	string filename = path + name +"3c04-30-30-vw0.005.pl";
	ofstream PLfile1(filename, ios::app);
	PLfile1 << "GOCAD PLine 0.02sim\n";
	PLfile1 << "HEADER{\n";
	PLfile1 << "name:" << "als-" << name << "\n";
	PLfile1 << "}\n";
	PLfile1 << "ILINE\n";
	for (int i = 0; i < pathList.size(); i++) {
		int start = nodeNumList[pathList[i]];
		PLfile1 << "VRTX " << i << " " << P->points[start].x << " " << P->points[start].y << " " << P->points[start].z << "\n";
	}
	for (int i = 0; i < pathList.size() - 1; i++) {

		PLfile1 << "SEG " << i << " " << i + 1 << "\n";

	}
	//�����������·�����ᾭ�����������Լ�������м�㣬������������ر�ע����ǣ������н���ű���
	//�Ϳ��Լ���ÿ�����Ӧ�ı���ˡ�
	//���������P0����ȫ���е�
	
	PLfile1 << "END";
	PLfile1.close();
};


void cleanFile(string strPath) 
{
	ofstream PLfile1(strPath);
	PLfile1.close();
}

vector<int> createSequentialList(int first, int length)
{
	vector <int> v;
	for (int i = 0; i < length; i++)
	{
		v.push_back(first + i);
	}
	return v;
}

//���������Ϊ�˲�ѯԪ��������������Ĵ���
int order_element_in_vector(const vector<int> &v, int element) {
	for (int order = 0; order < v.size(); order++) {
		if (element == v[order])
			return order;
	}
	//Ϊ���������������
	return -1;
}

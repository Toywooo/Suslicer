#include "slicer.h"
#include <iostream>
#include<fstream>

using namespace std;

map<int, vector<vector<float> > > Slicing(Trimesh& m, std::vector<float>& P)
{
	vector<vector<float> >temp;
	vector<vector<float>> s;
	ofstream myout("C:/Users/wuhua/Desktop/suslicer/bunny.txt");
	
	////输入为三角形列表T（已按zmin排序），平面列表P，层厚度，布尔参数srt
   	 //需调用BUILD-TRIANGLE-LISTS函数
	//构建有效集合A，并调用COMPUTE-INTERSECTION函数
	//输出为完成分组的三角形列表S，按zmin及层的z坐标分组
	map<int, vector<vector<float> > > segs;
	map<int, vector<unsigned int> > fgroup = Build_triangle_list(P, m);
	vector<unsigned int> A;
	int N = fgroup.size();
	for (int iLayer = 0; iLayer < N; iLayer++)
	{
		A.insert(A.end(), fgroup[iLayer].begin(), fgroup[iLayer].end());
		vector<vector<float>> S;
		vector<unsigned int> Temp;

		//std::cout << iLayer << ": " << A.size() << std::endl;
		
		for (unsigned int i = 0; i < A.size(); i++)
		{
			unsigned int fi = A[i];
			float zmax = m.get_f_maxzx(fi);

			if (zmax < P[iLayer])
			{
				Temp.push_back(fi);
			}
			else
			{
				vector<float> seg = ComputeIntersection(fi, P[iLayer], m);
				if (seg[2] != 0)
				{
					S.push_back(seg);
				}
			}
		}
		
		//To be optimized: remove faces under the current plane.
		for (unsigned int i = 0; i < Temp.size(); i++)
		{
			A.erase(std::remove(A.begin(), A.end(), Temp[i]), A.end());
		}
		segs[iLayer] = S;
		//cout << "------"<<endl;
		//contour_construction(N, S);
		temp=contour_construction(N, S);
		
		for (int i = 0; i < temp.size(); i++)
		{
			myout << temp[i][0]<<" " << temp[i][1] <<" "<<temp[i][2] <<" " << endl;
		}
	}
	myout.close();
	return segs;
}

std::map<int, vector<unsigned int> > Build_triangle_list(std::vector<float>& P, Trimesh& m)
{
	//int N = P.size() + 1;
	map<int, vector<unsigned int> > face_group_by_layer;

	for (unsigned int fi = 0; fi < m.num_F(); fi++)
	{
		unsigned int li = binary_searching(P, fi, m);
		face_group_by_layer[li].push_back(fi);
	}

	return face_group_by_layer;
};

//对于一个三角形做二分查找操作
int binary_searching(vector<float>& P, unsigned int fi, Trimesh& m)
{
	float min_zx = m.get_f_minzx(fi);
	int k = P.size();
	if (min_zx > P[k - 1]) return k;
	if (min_zx < P[0]) return 0;
	int low = 0;
	int high = k - 1;
	while ((high - low) > 1)
	{
		int m = (high + low) / 2;//m为int型，对数值向下取整
		if (min_zx > P[m])
			low = m;
		else
			high = m;
	}
	return high;
}

//https://stackoverflow.com/questions/3142469/determining-the-intersection-of-a-triangle-and-a-plane
std::vector<float> ComputeIntersection(unsigned int fi, float plane, Trimesh& m)
{
	std::vector<float> seg(6, 0.0f);
	std::vector<std::vector<float> > upperVerts, downVerts;
	const unsigned int* pF = m.F(fi);
	const unsigned int vi0 = m.F(fi)[0];
	vector<float>vert0(3, 0);
	vert0[0] = m.V(vi0)[0];
	vert0[1] = m.V(vi0)[1];
	vert0[2] = m.V(vi0)[2];
	const unsigned int vi1 = m.F(fi)[1];
	vector<float>vert1(3, 0);
	vert1[0] = m.V(vi1)[0];
	vert1[1] = m.V(vi1)[1];
	vert1[2] = m.V(vi1)[2];
	const unsigned int vi2 = m.F(fi)[2];
	vector<float>vert2(3, 0);
	vert2[0] = m.V(vi2)[0];
	vert2[1] = m.V(vi2)[1];
	vert2[2] = m.V(vi2)[2];
	float dis_to_plane0 = vert0[2] - plane;
	float dis_to_plane1 = vert1[2] - plane;
	float dis_to_plane2 = vert2[2] - plane;


	if (dis_to_plane0 != 0 && dis_to_plane1 != 0 && dis_to_plane2 != 0)
	{
		if (dis_to_plane0 > 0)
		{
			upperVerts.push_back(vert0);
		}
		else
		{
			downVerts.push_back(vert0);
		}
		if (dis_to_plane1 > 0)
		{
			upperVerts.push_back(vert1);
		}
		else
		{
			downVerts.push_back(vert1);
		}
		if (dis_to_plane2 > 0)
		{
			upperVerts.push_back(vert2);
		}
		else
		{
			downVerts.push_back(vert2);
		}
	}
	else if (dis_to_plane0 == 0&& dis_to_plane1 == 0)
	{
		seg.clear();
		seg.push_back(vert0[0]);
		seg.push_back(vert0[1]);
		seg.push_back(vert0[2]);
		seg.push_back(vert1[0]);
		seg.push_back(vert1[1]);
		seg.push_back(vert1[2]);
		return seg;
	}
	else if (dis_to_plane1 == 0 && dis_to_plane2 == 0)
	{
		seg.clear();
		seg.push_back(vert1[0]);
		seg.push_back(vert1[1]);
		seg.push_back(vert1[2]);
		seg.push_back(vert2[0]);
		seg.push_back(vert2[1]);
		seg.push_back(vert2[2]);
		return seg;
	}
	else if (dis_to_plane0 == 0 && dis_to_plane2 == 0)
	{
		seg.clear();
		seg.push_back(vert0[0]);
		seg.push_back(vert0[1]);
		seg.push_back(vert0[2]);
		seg.push_back(vert2[0]);
		seg.push_back(vert2[1]);
		seg.push_back(vert2[2]);
		return seg;
	}


	int k = 0;
	for (int i = 0; i < upperVerts.size(); i++)
	{
		for (int j = 0; j < downVerts.size(); j++)
		{
			seg[k * 3] = (upperVerts[i][2] - plane) / (upperVerts[i][2] - downVerts[j][2])
				* (downVerts[j][0] - upperVerts[i][0]) + upperVerts[i][0];
			seg[k * 3 + 1] = (upperVerts[i][2] - plane) / (upperVerts[i][2] - downVerts[j][2])
				* (downVerts[j][1] - upperVerts[i][1]) + upperVerts[i][1];
			seg[k * 3 + 2] = plane;
			k++;
		}
	}
	/*for (int i = 0; i < seg.size(); i++)
	{
		std::cout << seg[i] << ", ";
	}
	std::cout << std::endl;*/
	return seg;
}


vector<vector<float> >
contour_construction(int q, vector<vector<float> >S)
{
	multimap<int, vector<vector<float> > > contours;
	multimap<vector<float>, vector<float>> hashmap1,hashmap2;
	vector<float> mymap1, mymap2,mymap;
	vector < vector<float> >Temp;

	for (int i = 1; i < S.size(); i++)
	{
		mymap1.push_back(S[i][0]);
		mymap1.push_back(S[i][1]);
		mymap1.push_back(S[i][2]); 
		mymap2.push_back(S[i][3]);
		mymap2.push_back(S[i][4]);
		mymap2.push_back(S[i][5]);
	
		hashmap1.insert(pair < vector<float>, vector<float>>(mymap1, mymap2));
		hashmap2.insert(pair < vector<float>, vector<float>>(mymap2, mymap1));
		mymap1.clear();
		mymap2.clear();
	}

	mymap1.push_back(S[0][0]);
	mymap1.push_back(S[0][1]);
	mymap1.push_back(S[0][2]);
	mymap2.push_back(S[0][3]);
	mymap2.push_back(S[0][4]);
	mymap2.push_back(S[0][5]);
	Temp.push_back(mymap1);
	Temp.push_back(mymap2);

	while (!hashmap1.empty())
	{
		multimap<vector<float>, vector<float>>::iterator vit1 = hashmap1.find(mymap2);
		multimap<vector<float>, vector<float>>::iterator vit2 = hashmap2.find(mymap2);
		
		if (vit1 != hashmap1.end())
		{
			int count2=0;
			mymap1 = (*vit1).first;
			mymap2.clear();
			mymap2.push_back((*vit1).second[0]);
			mymap2.push_back((*vit1).second[1]);
			mymap2.push_back((*vit1).second[2]);
			hashmap1.erase(vit1);
			Temp.push_back(mymap2);

			pair<multimap<vector<float>, vector<float>>::iterator,
				multimap<vector<float>, vector<float>>::iterator> 
				ps2 = hashmap2.equal_range(mymap2);
			multimap<vector<float>, vector<float>>::iterator it2 = ps2.first;

			count2 = hashmap2.count(mymap2);
			if (count2 == 2)
			{
				while (it2 != ps2.second)
				{
					if (it2->second == mymap1)
					{
						hashmap2.erase(it2++);
					}
					else
					{
						it2++;
					}
				}
			}
			else
			{
				hashmap2.erase(mymap2);
			}
		}
		
		else if (vit2 != hashmap2.end())//没有用新的mymap2循环查找
		{
			int count1=0;
			mymap1 = mymap2;
			mymap2.clear();
			//hashmap1.erase(it);将it从hashmap1中删除，则后续不能调用it
			mymap2.push_back((*vit2).second[0]);
			mymap2.push_back((*vit2).second[1]);
			mymap2.push_back((*vit2).second[2]);
			hashmap2.erase(vit2);
			//hashmap1.erase(mymap2);
			Temp.push_back(mymap2);

			pair<multimap<vector<float>, vector<float>>::iterator,
				multimap<vector<float>, vector<float>>::iterator> 
				ps1 = hashmap1.equal_range(mymap2);
			multimap<vector<float>, vector<float>>::iterator it1 = ps1.first;

			count1 = hashmap1.count(mymap2);
			if (count1 == 2)
			{
				while (it1 != ps1.second)
				{
					if (it1->second == mymap1)
					{
						hashmap1.erase(it1++);
					}
					else
					{
						it1++;
					}
				}
			}
			else
			{
				hashmap1.erase(mymap2);
			}
		}
		else
		{
			Temp.push_back(mymap2);
			multimap<vector<float>, vector<float>>::iterator pit1 = hashmap1.begin();
			mymap1.clear();
			mymap2.clear();
			mymap1.push_back((*pit1).first[0]);
			mymap1.push_back((*pit1).first[1]);
			mymap1.push_back((*pit1).first[2]);
			mymap2.push_back((*pit1).second[0]);
			mymap2.push_back((*pit1).second[1]);
			mymap2.push_back((*pit1).second[2]);
			hashmap1.erase(pit1);

			Temp.push_back(mymap1);
			Temp.push_back(mymap2);
		}
	}
	
	return Temp;

}




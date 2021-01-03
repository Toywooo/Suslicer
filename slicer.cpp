#include "slicer.h"
#include <iostream>
#include<fstream>
using namespace std;

map<int, vector<vector<float> > > Slicing(Trimesh& m, std::vector<float>& P)
{
	vector<vector<float> >temp;
	//-------------------------multimap<int, vector<vector<float>>>temp;
	ofstream myout("C:/Users/wuhua/Desktop/suslicer/bunny2.txt");
	//-------------------------ofstream myout("C:/Users/wuhua/Desktop/suslicer/一个奇奇怪怪的皮卡丘.txt");
	
	map<int, vector<vector<float> > > segs;
	map<int, vector<unsigned int> > fgroup = Build_triangle_list(P, m);
	vector<unsigned int> A;
	int N = fgroup.size();
	if (P.size() < fgroup.size())
		N = N - 1;
	for (int iLayer = 0; iLayer < N; iLayer++)
	{
		//if (iLayer != 426) continue;
		A.insert(A.end(), fgroup[iLayer].begin(), fgroup[iLayer].end());
		vector<vector<float>> S;
		vector<unsigned int> Temp;
		
		std::cout << iLayer << ": " << A.size() << std::endl;
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
				vector<float> seg = ComputeIntersection1(fi, P[iLayer], m);
				S.push_back(seg);
			}
		};
		//To be optimized: remove faces under the current plane.
		for (unsigned int i = 0; i < Temp.size(); i++)
		{
			A.erase(std::remove(A.begin(), A.end(), Temp[i]), A.end());
		}
		segs[iLayer] = S;
		temp=contour_construction(N, S);
		for (int i = 0; i < temp.size(); i++)
		{
			myout << temp[i][0] << "     " << temp[i][1] << "     " << temp[i][2] << "     " << endl;
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

int binary_searching(vector<float>& P, unsigned int fi, Trimesh& m)
{
	float min_zx = m.get_f_minzx(fi);
	int k = P.size();
	if (min_zx > P[k - 1]) return k;
	if (min_zx < P[0]) return 0;
	int low = 0;
	int high = k - 1;
	while (high - low > 1)
	{
		int m = (high + low) / 2;
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
	for (int i = 0; i < 3; i++)
	{
		const unsigned int vi = m.F(fi)[i];
		std::vector<float> vert(4,0);      
		vert[0] = m.V(vi)[0];
		vert[1] = m.V(vi)[1];
		vert[2] = m.V(vi)[2];
		float dis_to_plane = vert[2] - plane;
		
		if (dis_to_plane > 0)
		{
			vert[3] = dis_to_plane;
			upperVerts.push_back(vert);
		} 
		else 
		{
			vert[3] = -dis_to_plane;
			downVerts.push_back(vert);
		}
	}
	int k = 0;
	for (int i = 0; i < upperVerts.size(); i++)
	{
		for (int j = 0; j < downVerts.size(); j++)
		{
			seg[k * 3] = (upperVerts[i][2] - plane) / (upperVerts[i][2] - downVerts[j][2])
				* (downVerts[j][0] - upperVerts[i][0]) + upperVerts[i][0];
			seg[k * 3+1] = (upperVerts[i][2] - plane) / (upperVerts[i][2] - downVerts[j][2])
				* (downVerts[j][1] - upperVerts[i][1]) + upperVerts[i][1];
			seg[k * 3 + 2] = plane;
			k++;
		}
	}
	return seg;//seg中只存放两个点的坐标，每次调用返回的都是一条线段的两个端点
}

vector<float> ComputeIntersection1(unsigned int fi, float plane, Trimesh& m)
{
	std::vector<float> seg(6, 0.0f);
	std::vector<std::vector<float> > upperVerts, downVerts;
	const unsigned int* pF = m.F(fi);
	for (int i = 0; i < 3; i++)
	{
		const unsigned int vi = m.F(fi)[i];
		std::vector<float> vert(4, 0);
		vert[0] = m.V(vi)[0];
		vert[1] = m.V(vi)[1];
		vert[2] = m.V(vi)[2];
		float dis_to_plane = vert[2] - plane;

		if (dis_to_plane > 0)
		{
			vert[3] = dis_to_plane;
			upperVerts.push_back(vert);
		}
		else
		{
			vert[3] = -dis_to_plane;
			downVerts.push_back(vert);
		}
	}

	int k = 0;
	if (upperVerts.size() != 0)
	{
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
	}
	else
	{
		int count = 0;
		vector<int> index;
		for (int k = 0; k < downVerts.size(); k++)
		{
			if (downVerts[k][2] - plane == 0)
			{
				count++;
				index.push_back(k);
			}
		}
		if (count == 1)
		{
			for (int i = 0; i < 3; i++)
			{
				seg[i]= downVerts[index[0]][i];
				seg[i+3]=downVerts[index[0]][i];
			}
		}
		else if (count == 2)
		{
			for (int i = 0; i < 3; i++)
			{
				seg[i]= downVerts[index[0]][i];
				seg[i+3]= downVerts[index[1]][i];
			}
		}
	}
	return seg;
}
void deletePoint(multimap<vector<float>, vector<float>>& hashmap,
	const vector<float>point1, const vector<float>point2)
{
	pair<multimap<vector<float>, vector<float>>::iterator,
		multimap<vector<float>, vector<float>>::iterator>
		ps = hashmap.equal_range(point2);
	multimap<vector<float>, vector<float>>::iterator it = ps.first;

	int count = hashmap.count(point2);
	if (count == 2)
	{
		while (it != ps.second)
		{
			if (it->second == point1)
			{
				hashmap.erase(it++);
			}
			else
			{
				it++;
			}
		}
	}
	else
	{
		hashmap.erase(point2);
	}
}
vector<vector<float> >
contour_construction(int q, vector<vector<float> >S)
{
	multimap<vector<float>, vector<float>> hashForward,hashOpposite;
	vector<float> FirstPoint, SecondPoint;
	vector < vector<float> >Contour;
	for (int i = 1; i < S.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			FirstPoint.push_back(S[i][j]);
			SecondPoint.push_back(S[i][j+3]);
		}
		hashForward.insert(pair < vector<float>, vector<float>>(FirstPoint, SecondPoint));
		hashOpposite.insert(pair < vector<float>, vector<float>>(SecondPoint, FirstPoint));
		FirstPoint.clear();
		SecondPoint.clear();
	}
	for (int i = 0; i < 3; i++)
	{
		FirstPoint.push_back(S[0][i]);
		SecondPoint.push_back(S[0][i + 3]);
	}
	Contour.push_back(FirstPoint);
	Contour.push_back(SecondPoint);

	while (!hashForward.empty())
	{
		multimap<vector<float>, vector<float>>::iterator fit1 = hashForward.find(SecondPoint);
		multimap<vector<float>, vector<float>>::iterator fit2 = hashOpposite.find(SecondPoint);
		
		if (fit1 != hashForward.end())
		{
			FirstPoint = fit1->first;
			SecondPoint = fit1->second;
			hashForward.erase(fit1);
			Contour.push_back(SecondPoint);
			deletePoint(hashOpposite, FirstPoint, SecondPoint);
		}
		else if (fit2 != hashOpposite.end())
		{
			int count1=0;
			FirstPoint = SecondPoint;
			SecondPoint = fit2->second;
			hashOpposite.erase(fit2);
			Contour.push_back(SecondPoint);
			deletePoint(hashForward, FirstPoint, SecondPoint);
		}
		else
		{
			//Temp.push_back(mymap2);
			multimap<vector<float>, vector<float>>::iterator pit1 = hashForward.begin();
			FirstPoint = pit1->first;
			SecondPoint = pit1->second;
			hashForward.erase(pit1);
			deletePoint(hashOpposite, FirstPoint, SecondPoint);
			Contour.push_back(FirstPoint);
			Contour.push_back(SecondPoint);
		}
	}
	return Contour;
}


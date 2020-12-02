/*
Copyright (c) 2015, Yuan Yao (fly2mars@gmail.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#include <Engines/Mesh/stl_reader.h>
// The other choice is Eigen that we can use a matrix V and a matrix F to represent the triangle mesh.
// A simple triangle class
class Triangle
{
public:
	Triangle() { fid = -1; }
	// (x,y,z) = t.v[0][0], t.v[0][1], t.v[0][2]
	float* v(int vid, std::vector<float> &coords) {
		return &coords[3.0 * vids[vid]];
	}
	float zmax() { };// return ()
public:
	std::vector<int> vids;   //vert id  int[3]
	size_t fid;              //face id  in trimesh
	//size_t是一个与机器相关的unsigned类型，其大小足以保证存储内存中对象的大小。
};

/** \brief Class Trimesh() is a simple triangle mesh class.
*          Here we simple encalpsulate StlMesh class from stl_reader .
*/
class Trimesh
{
public:
	Trimesh() { p_mesh = NULL; }//P平面网格为空
	void read_file(const char* file_name);
	float height() { return maxP_[2] - minP_[2]; }
	float get_f_minzx(unsigned int fi);

	////////////////////////////////////////////
	float Trimesh::get_f_maxzx(unsigned int fi);
	/////////////////////////////////////////////////


	//Ecapsulate funcs
	const float* V(const size_t vi) { return p_mesh->vrt_coords(vi); }
	const unsigned int* F(const size_t fi) { return p_mesh->tri_corner_inds(fi); }
	const float* N(const size_t ni) { return p_mesh->tri_normal(ni); }
	size_t num_F() { return p_mesh->num_tris(); }

	//IO
	void save_to_ascii(char* filename);
	void save_faces_to_ascii(char* filename, std::vector<unsigned int> faces);
	
public:
	// All members are public for convinient using.
	stl_reader::StlMesh <float, unsigned int> *p_mesh;
	std::vector<float> minP_;
	std::vector<float> maxP_;

};

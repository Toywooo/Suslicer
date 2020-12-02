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
#include <vector>
#include <map>
#include<unordered_map>
#include "suMesh.h"

/** Slicing() return segments map<int,int> for each slice planes
*   Only general case is considered(unsorted faces & ununiformed slicing),
*   because the situation is common.
*   This algorithm is an implementation of the idea from Rodrigo Minetto etc.
*
* \param m          [in] The input mesh
* \param P          [in] The slice plane height array
* \returns		    Group of Indice of triangles mapping with each layer.
*/
std::map<int, std::vector<std::vector<float> > > Slicing(Trimesh& m, std::vector<float>& P);
/** Build_triangle_list() group the input triangle into each slice planes
*   Only general case is considered(unsorted faces & ununiformed slicing),
*   because the situation is common. 
*
* \param P          [in] The slice plane height array
* \param m          [in] The input mesh
* \returns		    Group of Indice of triangles mapping with each layer.
*/
std::map<int, std::vector<unsigned int> > Build_triangle_list(std::vector<float> &P, Trimesh &m);

/** binary_searching() find which plane intersection with the input triangle
*
* \param P          [in] The slice plane height array
* \param fi      	[in] The triangle face id for querying
* \param m          [in] The input mesh
* \returns		    Index of a plane that the input triangle blonging.
*/
int binary_searching(std::vector<float> &P, unsigned int fi, Trimesh &m);

/** ComputeIntersection() find segments(p1,p2) at the intersection of the plane and the input triangle
*
* \param P          [in] The slice plane height array
* \param fi      	[in] The triangle face id for querying
* \param m          [in] The input mesh
* \returns		    segments (point 1, point 2).
*/
std::vector<float> ComputeIntersection(unsigned int fi, float plane, Trimesh &m);

/** binary_searching() find which plane intersection with the input triangle
*
* \param P          [in] The slice plane height array
* \param fi      	[in] The triangle face id for querying
* \param m          [in] The input mesh
* \returns		    Index of a plane that the input triangle blonging.
*/

std::vector<std::vector<float> >
contour_construction(int q, std::vector<std::vector<float> >S);

std::multimap<int, std::vector<std::vector<float>>>
contour_construction1(int q, std::vector<std::vector<float> >S);



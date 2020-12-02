
#include <iostream>
#include <Engines/Utility/suCMDParser.h>
#include "slicer.h"
using namespace std;

void helper()
{
	std::cout << "suSlicer is a mesh slicer developed by RMEC of Shanghai University. \n";
	std::cout << "Usage: \n    suSlicer -i [input stl] -o [output contour]" << std::endl;
	std::cout << "Other parameter include -t -tfile" << std::endl;
	std::cout << std::endl;
	std::cout << "-t Specify thickness of layers[default: 0.2]." << std::endl;
	std::cout << "-r Specify thickness config file" << std::endl;

}

void main(int argc, char* argv[])
{
	suCMDParser option(argv, argc);
	std::string inputFile;
	std::string outputFile;
	float thickness = 0;


	if (option.findOption("-h") || argc == 1)
	{
		helper();
		return;
	}

	////////////////////////////////////////// get parameters获取参数
	try {
		//get input file
		if (option.findOption("-i"))//打开输入stl文件，没有输入文件则抛出异常
		{
			inputFile = option.findOptionValue("-i");
			if (option.findOption("-o"))
			{
				outputFile = option.findOptionValue("-o");
			}
			else {
				outputFile = inputFile + ".out";
			}			

		}
		else {
			throw std::exception("No input file!");
		}
		if (option.findOption("-t")) //读取层厚，如果未输入层厚，则抛出异常
		{
			//In order reduce the complex,  we simplely specify one thickness
			std::string strValue = option.findOptionValue("-t");
			thickness = std::stof(strValue);//stof函数可以将字符串转换成数字
		}
		else {
			throw std::exception("Please specify a legal thickness!");
		}
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
		//what() 是异常类提供的一个公共方法，它已被所有子异常类重载。这将返回异常产生的原因。
		helper();
		return;
	}

	//Pre-processing
	//read stl file                     // 0       1     2
	//compute stl model size(length, weight, height)
	//height = max(z) - min(z)
	//std::vector<float> coords, normals; //(x,y,z,x,y,z,x,y,z....)
	//std::vector<unsigned int> tris, solids;//(0,1,2,2,3,0)三角形  立方体
	Trimesh mesh;
	try {
		mesh.read_file(inputFile.c_str());	
		//c_str是为了与c语言兼容，在c语言中没有string类型，故必须通过string类对象的成员函数c_str()把string 对象转换成c中的字符串样式。	
	}
	catch (std::exception& e) {
		std::cout << e.what() << std::endl;
		return;
	}

	
	std::vector<float> P;
	
	//size_t n = tris.size() / 3;
	//m.read_triangles(coords, normals, tris);坐标 法线  三角形
	float h = mesh.height();
	float p = 0;
	while (p < h)
	{
		p = p + thickness;
		P.push_back(p);//将所有层的z坐标存入容器P中
	}
	//std::vector<std::vector<std::vector<int> > > seg_group = Slicing(n, mesh, P);//
	//for each layer
	for (int i = 0; i < P.size(); i++)
	{
		std::vector<std::vector<int> > segs;//线段
		//segs = seg_group[i];
		/////////////////////////
		std::vector<std::vector<std::vector<float> > > contours;//闭合图形

		int q = segs.size();
		
		//Slicing(mesh, P);

		//save contours for each layer.
		//..
		
		//contours = contour_construction(q, segs);
	}
}
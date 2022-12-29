#include <iostream>
#include "PositionReader.h"
#include "BVHreader.h"

void IKAllFrame(std::vector<double>& all_frame)
{

}

double* copyMotion(std::vector<double>& all_frames)
{
	double* allFrameMotion = new double[all_frames.size()];
	int ID = all_frames.size() - 1;
	while (!all_frames.empty())
	{
		allFrameMotion[ID] = all_frames.back();
		all_frames.pop_back();
		ID--;
	}
	return allFrameMotion;
}

void CopyBVHFile(std::string oldFile, std::string newFile)
{
	std::ifstream input(oldFile, ios::binary);
	std::ofstream output(newFile, ios::binary);
	std::string line;

	while (getline(input, line)) output << line << "\n";
	input.close();
	output.close();
}

int main(int argc, char** args) 
{
	if (argc != 4) 
	{
		std::cout << "Error: argument count not enough, expect 3 but now is "<< argc - 1 << std::endl;
		return 0;
	}
	std::string positionFileName(args[1]);
	std::string bvhFileName(args[2]);
	std::string saveFileName(args[3]);
	// read position file
	//PositionReader position_file(".\\data\\positions_Tue_09_m_04_s_X-05.txt");
	PositionReader position_file(positionFileName);

	// read bvh file
	//BVHreader bvhReader(".\\data\\Tue_09_m_03_m_X-05_skeleton.bvh");
	BVHreader bvhReader(bvhFileName.c_str());

	// compute IK
	std::vector<double> all_frames;
	bvhReader.IsDamped = true;
	bvhReader.IsControl = false;
	bvhReader.lambda = 0.1;

	bvhReader.bvhFile->motion[0] = position_file.Frames[0].Joints[0].x;
	bvhReader.bvhFile->motion[1] = position_file.Frames[0].Joints[0].y;
	bvhReader.bvhFile->motion[2] = position_file.Frames[0].Joints[0].z;

	bvhReader.forwardK(0, 1.0f);
	bvhReader.printJoint();
	for (int frame_ID = 0; frame_ID < position_file.frame_num; frame_ID++)
	{
		std::vector<double> positions;
		for (int i = 0; i < position_file.Frames[frame_ID].joint_num; i++) {
		//for (int i = 0; i < 8; i++) {
			positions.push_back(position_file.Frames[frame_ID].Joints[i].x);
			positions.push_back(position_file.Frames[frame_ID].Joints[i].y);
			positions.push_back(position_file.Frames[frame_ID].Joints[i].z);
		}

		//positions[20] += 100;
		//positions[32] += 100;

		bvhReader.bvhFile->motion[0] = positions[0];
		bvhReader.bvhFile->motion[1] = positions[1];
		bvhReader.bvhFile->motion[2] = positions[2];

		bvhReader.forwardK(0, 1.0f);

		//--------------------------
		//std::vector<int> jointSet;
		//jointSet.push_back(6);
		//jointSet.push_back(4);
		//jointSet.push_back(4);
		//jointSet.push_back(4);
		//jointSet.push_back(4);

		//refresh original motion with the new one
		double* result = bvhReader.inverseK(0, 1.0f, positions);
		//double* result = bvhReader.inverseK(0, 1.0f, positions, jointSet);
		for (int i = 0; i < bvhReader.bvhFile->num_channel; i++)
		{
			bvhReader.bvhFile->motion[i] = result[i];
		}

		// copy one frame motion into vector
		for (int i = 0; i < bvhReader.bvhFile->num_channel; i++) {
			all_frames.push_back(result[i]);
		}
	}
	
	// write bvh file
	delete bvhReader.bvhFile->motion;
	bvhReader.bvhFile->motion = copyMotion(all_frames);
	
	bvhReader.bvhFile->num_frame = position_file.frame_num;
	// copy a file
	CopyBVHFile(bvhFileName, saveFileName);
	bvhReader.bvhFile->Write(saveFileName);
	
	return 0;
}


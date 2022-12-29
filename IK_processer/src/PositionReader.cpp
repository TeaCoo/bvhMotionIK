#include "PositionReader.h"

void PositionReader::Init()
{
	this->frame_num = 0;
	this->isDestoried = true;
}

PositionReader::PositionReader()
{
	Init();
}

PositionReader::PositionReader(std::string fileName)
{
	Init();
	ReadPositionFile(fileName);
}

PositionReader::~PositionReader()
{
	FreePosition();
	if (!this->isDestoried) 
	{
		std::cout << "Warning: Position array hasn't be destoried!" << std::endl;
		system("pause");
		exit(0);
	}
}

void PositionReader::ReadPositionFile(std::string fileName)
{
	// open file
	std::ifstream ifs;
	ifs.open(fileName);
	if (!ifs.is_open()) {
		std::cout << "Error: Open position file fail!" << std::endl;
	}
	
	// load data
	std::string buf;
	while (getline(ifs, buf)) {
		std::istringstream ss(buf);
		StorePosition(ss);
	}
	// close file
	this->isDestoried = false;
	ifs.close();
}

Position PositionReader::getPosition(int frameID, int jointID)
{
	return this->Frames[frameID].Joints[jointID];
}

void PositionReader::StorePosition(std::istringstream& ss)
{
	OneFrame oneFrame;
	Position Pos;
	int count = 0;
	while (ss)
	{
		std::string s;
		if (!getline(ss, s, ',')) break;

		// fill position structure
		if (count == 0)
		{
			Pos.x = std::stod(s);
			count++;
		}
		else if (count == 1)
		{
			Pos.y = std::stod(s);
			count++;
		}
		else if (count == 2)
		{
			Pos.z = std::stod(s);
			oneFrame.Joints.push_back(Pos);
			oneFrame.joint_num++;
			count = 0;
		}
	}
	this->Frames.push_back(oneFrame);
	this->frame_num++;
}

void PositionReader::FreePosition()
{
	while (this->Frames.size())
	{
		while (this->Frames.back().Joints.size()) {
			this->Frames.back().Joints.pop_back();
			this->Frames.back().joint_num--;
		}	
		this->Frames.pop_back();
		this->frame_num--;
	}

	if (this->frame_num == 0)
		this->isDestoried = true;
}

// stream input
std::istream& operator >> (std::istream& inStream, Position& value)
{ // stream output
	inStream >> value.x >> value.y >> value.z;
	return inStream;
} // stream output

// stream output
std::ostream& operator << (std::ostream& outStream, const Position& value)
{ // stream output
	outStream << std::setprecision(10) << value.x << " " << std::setprecision(10) << value.y << " " << std::setprecision(10) << value.z;
	return outStream;
} // stream output

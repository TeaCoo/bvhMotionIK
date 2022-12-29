#pragma once
#ifndef POSITION_H
#define POSITION_H

#include <string>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <iomanip>

class Position
{
public:
	double x = 0;
	double y = 0;
	double z = 0;
};

std::istream& operator >> (std::istream& inStream, Position& value);
std::ostream& operator << (std::ostream& outStream, const Position& value);

class OneFrame
{
public:
	int joint_num = 0;
	std::vector<Position> Joints;
};

class PositionReader
{
public:
	PositionReader();
	PositionReader(std::string fileName);
	~PositionReader();
	void Init();
	void ReadPositionFile(std::string fileName);
	Position getPosition(int frameID, int jointID);
	bool isDestoried;
	int frame_num;
	std::vector<OneFrame> Frames;

private:
	void StorePosition(std::istringstream& ss);
	void FreePosition();
};

#endif
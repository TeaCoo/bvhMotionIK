//////////////////////////////////////////////////////////////////////
//
//  University of Leeds
//  COMP 5823M Animation and Simulation
//  The reader of bvh file
//
//  November, 2021
//
//  -----------------------------
//  BVHreader.cpp
//  -----------------------------
//  
//  Read information and process them from bvh file
//  
////////////////////////////////////////////////////////////////////////
#include "BVHreader.h"
#include<math.h>

BVHreader::BVHreader(const char* filename)
{
	bvhFile = new BVH(filename);
	currentMatrix = Eigen::MatrixXd(4, 4);
	currentMatrix <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	matrixStack.push(currentMatrix);	
	newMotion = new double[bvhFile -> num_channel];
	for (int i = 0; i < bvhFile->num_channel; i++) 
	{
		allGains.push_back(1);
	}
	
}

BVHreader::~BVHreader()
{
	delete newMotion;
}

double BVHreader::calculateLength(BVH::Joint* joint) {
	double length = sqrt(pow(joint->offset[0],2) + pow(joint->offset[1],2) + pow(joint->offset[2], 2));
	BVH::Joint* parent = joint->parent;
	while (parent != NULL) {
		length += sqrt(pow(parent->offset[0], 2) + pow(parent->offset[1], 2) + pow(parent->offset[2], 2));
		parent = parent->parent;
	}
	return length;
}

BVH::Joint* getRoot(BVH::Joint* joint) {
	BVH::Joint* current = joint;
	BVH::Joint* parent = joint->parent;
	while (parent != NULL) {
		current = parent;
		parent = parent->parent;
	}
	return current;
}

Eigen::MatrixXd BVHreader::fdJaco(vector< BVH::Joint* >  allJoints, int frame_no, float scale, double* oldMotion) {
	
	double deltaSeta = 1e-6;
	double px1 = allJoints[0]->pos_1, py1 = allJoints[0]->pos_2, pz1 = allJoints[0]->pos_3;
	// store the position of locked joint into posSet
	vector<double> posSet;
	for (int i = 1; i < allJoints.size(); i++) {
		posSet.push_back(allJoints[i]->pos_1);
		posSet.push_back(allJoints[i]->pos_2);
		posSet.push_back(allJoints[i]->pos_3);
	}
	
	Eigen::MatrixXd Jaco = Eigen::MatrixXd::Zero(allJoints.size()*3, bvhFile->channels.size() - posChannel);
	
	for (int i = 0; i < allJoints.size() * 3 * (bvhFile->channels.size() - posChannel); i += allJoints.size() * 3) {
		newMotion[posChannel + i / (allJoints.size() * 3)] = oldMotion[posChannel + i / (allJoints.size() * 3)] + deltaSeta;
		forwardK(frame_no, scale, newMotion);
		double px2 = allJoints[0]->pos_1, py2 = allJoints[0]->pos_2, pz2 = allJoints[0]->pos_3;
		Jaco(i) = (px2 - px1) / deltaSeta;
		Jaco(i + 1) = (py2 - py1) / deltaSeta;
		Jaco(i + 2) = (pz2 - pz1) / deltaSeta;
		for (int j = 0,index = 1; j < (allJoints.size() - 1) * 3; j += 3,index++) {
			px2 = allJoints[index]->pos_1; py2 = allJoints[index]->pos_2; pz2 = allJoints[index]->pos_3;
			Jaco(i + 3 + j) = (px2 - posSet[j]) / deltaSeta;
			Jaco(i + 4 + j) = (py2 - posSet[j+1]) / deltaSeta;
			Jaco(i + 5 + j) = (pz2 - posSet[j+2]) / deltaSeta;
		}	
		newMotion[posChannel + i / (allJoints.size() * 3)] = oldMotion[posChannel + i / (allJoints.size() * 3)];
	}
	allJoints[0]->pos_1 = px1;
	allJoints[0]->pos_2 = py1;
	allJoints[0]->pos_3 = pz1;
	return Jaco;
}

Eigen::MatrixXd BVHreader::fdJaco(vector< BVH::Joint* >  allJoints, int frame_no, float scale, double* oldMotion, int startJoint) {

	double deltaSeta = 1e-6;
	//double px1 = allJoints[0]->pos_1, py1 = allJoints[0]->pos_2, pz1 = allJoints[0]->pos_3;
	// store the position of locked joint into posSet
	vector<double> posSet;
	for (int i = 0; i < allJoints.size(); i++) {
		posSet.push_back(allJoints[i]->pos_1);
		posSet.push_back(allJoints[i]->pos_2);
		posSet.push_back(allJoints[i]->pos_3);
	}
	Eigen::MatrixXd Jaco = Eigen::MatrixXd(allJoints.size() * 3, allJoints.size() * 3);
	Jaco.setZero();
	for (int i = 0; i < allJoints.size() * 3 * allJoints.size() * 3; i += allJoints.size() * 3) {
		newMotion[posChannel + i / (allJoints.size() * 3) + startJoint * 3] = oldMotion[posChannel + i / (allJoints.size() * 3) + startJoint * 3] + deltaSeta;
		
		forwardK(frame_no, scale, newMotion);

		for (int j = 0, index = 0; j < allJoints.size() * 3; j += 3, index++) {
			double px2 = allJoints[index]->pos_1, py2 = allJoints[index]->pos_2, pz2 = allJoints[index]->pos_3;	
			Jaco(i + 0 + j) = (px2 - posSet[j + 0]) / deltaSeta;
			Jaco(i + 1 + j) = (py2 - posSet[j + 1]) / deltaSeta;
			Jaco(i + 2 + j) = (pz2 - posSet[j + 2]) / deltaSeta;
		}
		newMotion[posChannel + i / (allJoints.size() * 3) + startJoint * 3] = oldMotion[posChannel + i / (allJoints.size() * 3) + startJoint * 3];
	}
	//allJoints[0]->pos_1 = px1;
	//allJoints[0]->pos_2 = py1;
	//allJoints[0]->pos_3 = pz1;
	return Jaco;
}

void BVHreader::printJoint()
{
	printJoint(this->bvhFile->joints[0]);
}

void BVHreader::printJoint(BVH::Joint* joint)
{
	printJoint(joint, 0);
}

void BVHreader::printJoint(BVH::Joint* joint, int depth)
{
	for (int i = 0; i < depth; i++) {
		std::cout << "\t";
	}
	std::cout << joint->name << std::endl;
	for (int i = 0; i < joint->children.size(); i++) {
		printJoint(joint->children[i], depth + 1);
	}
}

double* BVHreader::inverseK(BVH::Joint* joint, int frame_no, float scale, double x, double y, double z) {
	// get all locked joints and save them in allJoints
	vector< BVH::Joint* >  allJoints;
	allJoints.push_back(joint);
	for (int i = 0; i < bvhFile->joints.size(); i++) {
		if (bvhFile->joints[i]->has_locked == true)
			allJoints.push_back(bvhFile->joints[i]);
	}
	
	int iterMax = 1, iter = 0;
	double tol = 0.1;
	double now_pos_x = joint->pos_1, now_pos_y = joint->pos_2, now_pos_z = joint->pos_3;
	double distence = sqrt( pow((x - now_pos_x), 2)  + pow((y - now_pos_y) , 2) + pow((z - now_pos_z),2));

	double deltaSeta = 0.001;
	double* oldMotion = bvhFile->motion + frame_no * bvhFile->num_channel;
	for (int i = 0; i < bvhFile->num_channel; i++) {
		newMotion[i] = oldMotion[i];
	}

	// if moved joint out of the range of root joint, move root joint
	BVH::Joint* root = bvhFile->joints[0];
	double currentLength = sqrt(pow((x - root->pos_1), 2) + pow((y - root->pos_2), 2) + pow((z - root->pos_3), 2));
	if (currentLength > calculateLength(joint)) {
		posChannel = 0;
	}
	else {
		posChannel = 3;// when this = 3, we don't consider the position channels of root
	}

	// store the initial position of input joints
	Eigen::MatrixXd oldPos = Eigen::MatrixXd(allJoints.size() * 3, 1);
	// the position of moved joint
	oldPos(0) = now_pos_x;
	oldPos(1) = now_pos_y;
	oldPos(2) = now_pos_z;
	// the position of locked joints
	for (int i = 1; i < allJoints.size(); i++) {
		oldPos(i * 3) = allJoints[i]->pos_1;
		oldPos(i * 3 + 1) = allJoints[i]->pos_2;
		oldPos(i * 3 + 2) = allJoints[i]->pos_3;
	}
	// store the final position of joints after IK
	Eigen::MatrixXd desPos = Eigen::MatrixXd(allJoints.size() * 3, 1);
	// the position of moved joint
	desPos(0) = x;
	desPos(1) = y;
	desPos(2) = z;
	// the position fo locked joints
	for (int i = 1; i < allJoints.size(); i++) {
		desPos(i * 3) = allJoints[i]->pos_1;
		desPos(i * 3 + 1) = allJoints[i]->pos_2;
		desPos(i * 3 + 2) = allJoints[i]->pos_3;
	}

	while (distence > tol && iter < iterMax) {
		Eigen::MatrixXd Jaco = Eigen::MatrixXd(allJoints.size() * 3, bvhFile->channels.size() - posChannel);
		Jaco = fdJaco(allJoints, frame_no, scale, oldMotion);
		Eigen::MatrixXd deltaDegree = Eigen::MatrixXd(bvhFile->channels.size() - posChannel, 1);
		Eigen::MatrixXd TJaco = Eigen::MatrixXd(allJoints.size() * 3, allJoints.size() * 3);

		TJaco = Jaco * Jaco.transpose();	
		// whether damped button clicked
		if(IsDamped==true)
			TJaco = TJaco + pow(lambda, 2) * Eigen::MatrixXd::Identity(allJoints.size() * 3, allJoints.size() * 3);
		
		Eigen::MatrixXd newChannels = Eigen::MatrixXd(bvhFile->channels.size() - posChannel, allJoints.size() * 3);
		newChannels = Jaco.transpose() * TJaco.inverse();

		deltaDegree = newChannels * (desPos - oldPos);

		if (IsControl == true) {
			Eigen::MatrixXd z = Eigen::MatrixXd(bvhFile->channels.size() - posChannel, 1);
			Eigen::MatrixXd gain = Eigen::MatrixXd(bvhFile->channels.size() - posChannel, 1);
			for (int i = posChannel; i < allGains.size(); i++) {
				gain(i - posChannel) = allGains[i];
			}
			z = gain.array()* deltaDegree.array() * deltaDegree.array();
			
			Eigen::MatrixXd temp = Eigen::MatrixXd(allJoints.size() * 3, 1);
			temp = (desPos - oldPos) + Jaco * z;
			
			deltaDegree = newChannels * temp - z;
		}

		for (int i = posChannel; i < bvhFile->num_channel; i++) {
			newMotion[i] = newMotion[i] + deltaDegree(i- posChannel);
		}

		forwardK(frame_no, scale, newMotion);

		// store the position of moved joint into now_pos
		now_pos_x = joint->pos_1, now_pos_y = joint->pos_2, now_pos_z = joint->pos_3;
		// store the position of locked joint into posSet
		vector<double> posSet;
		for (int i = 1; i < allJoints.size(); i++) {
			posSet.push_back(allJoints[i]->pos_1);
			posSet.push_back(allJoints[i]->pos_2);
			posSet.push_back(allJoints[i]->pos_3);
		}
		// refresh oldPos
		// moved joint
		oldPos(0) = now_pos_x;
		oldPos(1) = now_pos_y;
		oldPos(2) = now_pos_z;
		//locked joints
		for (int i = 1; i < allJoints.size(); i++) {
			oldPos(i * 3) = allJoints[i]->pos_1;
			oldPos(i * 3 + 1) = allJoints[i]->pos_2;
			oldPos(i * 3 + 2) = allJoints[i]->pos_3;
		}

		distence = sqrt(pow((x - now_pos_x), 2) + pow((y - now_pos_y), 2) + pow((z - now_pos_z), 2));
		oldMotion = newMotion;
		iter += 1;
	}
	
	return newMotion;
}

double* BVHreader::inverseK(int frame_no, float scale, std::vector<double> positions)
{
	// fixed root
	this->posChannel = 3;
	int iterMax = 1, iter = 0;
	double tol = 0.1;

	// set initial motion
	double* oldMotion = bvhFile->motion + frame_no * bvhFile->num_channel;
	for (int i = 0; i < bvhFile->num_channel; i++) {
		newMotion[i] = oldMotion[i];
	}
	/*
	for (int i = 0; i < bvhFile->num_channel; i++) {
		std::cout<< newMotion[i] << ",";
	}
	std::cout << std::endl << std::endl;
	for (int i = 0; i < bvhFile->joints.size(); i++) {
		std::cout << bvhFile->joints[i]->pos_1 << ","<<
			bvhFile->joints[i]->pos_2 << "," << 
			bvhFile->joints[i]->pos_3 << ";" ;
	}
	std::cout << std::endl << std::endl;
	*/
	// store the initial position of input joints
	Eigen::MatrixXd oldPos = Eigen::MatrixXd(bvhFile->joints.size() * 3, 1);
	// the position of moved joint
	for (int i = 0; i < bvhFile->joints.size(); i++) {
		oldPos(i * 3 + 0) = bvhFile->joints[i]->pos_1;
		oldPos(i * 3 + 1) = bvhFile->joints[i]->pos_2;
		oldPos(i * 3 + 2) = bvhFile->joints[i]->pos_3;
	}

	// store the final position of joints after IK
	Eigen::MatrixXd desPos = Eigen::MatrixXd(bvhFile->joints.size() * 3, 1);
	// the position of moved joint
	for (int i = 0; i < bvhFile->joints.size(); i++) {
		desPos(i * 3 + 0) = positions[i * 3 + 0];
		desPos(i * 3 + 1) = positions[i * 3 + 1];
		desPos(i * 3 + 2) = positions[i * 3 + 2];
	}

	// compute average distence of all joints
	double distences = 0;
	for (int i = 0; i < bvhFile->joints.size(); i++) {
		distences += sqrt(
			pow((positions[i * 3 + 0] - bvhFile->joints[i]->pos_1), 2) +
			pow((positions[i * 3 + 1] - bvhFile->joints[i]->pos_2), 2) +
			pow((positions[i * 3 + 2] - bvhFile->joints[i]->pos_3), 2));
	}
	double avgDistence = distences / bvhFile->joints.size();

	// ======================================== IK ======================================== 
	while (avgDistence > tol && iter < iterMax) {
		Eigen::MatrixXd Jaco = Eigen::MatrixXd(bvhFile->joints.size() * 3, bvhFile->channels.size() - posChannel);
		Jaco = fdJaco(bvhFile->joints, frame_no, scale, oldMotion);
		Eigen::MatrixXd deltaDegree = Eigen::MatrixXd(bvhFile->channels.size() - posChannel, 1);
		Eigen::MatrixXd TJaco = Eigen::MatrixXd(bvhFile->joints.size() * 3, bvhFile->joints.size() * 3);

		TJaco = Jaco * Jaco.transpose();
		// whether damped button clicked
		if (IsDamped == true)
			TJaco = TJaco + pow(lambda, 2) * Eigen::MatrixXd::Identity(bvhFile->joints.size() * 3, bvhFile->joints.size() * 3);

		Eigen::MatrixXd newChannels = Eigen::MatrixXd(bvhFile->channels.size() - posChannel, bvhFile->joints.size() * 3);
		newChannels = Jaco.transpose() * TJaco.inverse();

		deltaDegree = newChannels * (desPos - oldPos);

		if (IsControl == true) {
			Eigen::MatrixXd z = Eigen::MatrixXd(bvhFile->channels.size() - posChannel, 1);
			Eigen::MatrixXd gain = Eigen::MatrixXd(bvhFile->channels.size() - posChannel, 1);
			for (int i = posChannel; i < allGains.size(); i++) {
				gain(i - posChannel) = allGains[i];
			}
			z = gain.array() * deltaDegree.array() * deltaDegree.array();

			Eigen::MatrixXd temp = Eigen::MatrixXd(bvhFile->joints.size() * 3, 1);
			temp = (desPos - oldPos) + Jaco * z;

			deltaDegree = newChannels * temp - z;
		}

		for (int i = posChannel; i < bvhFile->num_channel; i++) {
			newMotion[i] = newMotion[i] + deltaDegree(i - posChannel);
		}

		forwardK(frame_no, scale, newMotion);

		// refresh oldPos
		for (int i = 0; i < bvhFile->joints.size(); i++) {
			oldPos(i * 3 + 0) = bvhFile->joints[i]->pos_1;
			oldPos(i * 3 + 1) = bvhFile->joints[i]->pos_2;
			oldPos(i * 3 + 2) = bvhFile->joints[i]->pos_3;
		}

		// compute average distence of all joints
		distences = 0;
		for (int i = 0; i < bvhFile->joints.size(); i++) {
			distences += sqrt(
				pow((positions[i * 3 + 0] - bvhFile->joints[i]->pos_1), 2) +
				pow((positions[i * 3 + 1] - bvhFile->joints[i]->pos_2), 2) +
				pow((positions[i * 3 + 2] - bvhFile->joints[i]->pos_3), 2));
		}
		avgDistence = distences / bvhFile->joints.size();

		//std::cout << avgDistence << std::endl;
		oldMotion = newMotion;
		//================================================
		/*
		std::cout << "======================"<<std::endl;
		for (int i = 0; i < bvhFile->num_channel; i++) {
			std::cout << newMotion[i] << ",";
		}
		std::cout << std::endl << std::endl;
		for (int i = 0; i < bvhFile->joints.size(); i++) {
			std::cout << bvhFile->joints[i]->pos_1 << "," <<
				bvhFile->joints[i]->pos_2 << "," <<
				bvhFile->joints[i]->pos_3 << ";";
		}
		std::cout << std::endl << std::endl;
		*/
		//================================================
		iter += 1;
	}

	return newMotion;
}

/*
* This function is used to do IK seperatly. For example, first do IK for spine, then arms, then legs.
*/
double* BVHreader::inverseK(int frame_no, float scale, std::vector<double> positions, std::vector<int> jointSet)
{
	// fixed root
	this->posChannel = 3;

	// set initial motion
	double* oldMotion = bvhFile->motion + frame_no * bvhFile->num_channel;
	for (int i = 0; i < bvhFile->num_channel; i++) {
		newMotion[i] = oldMotion[i];
	}

	int jointPtr = 0;
	int startJoint = 0;
	// for each set of joint
	for (int jointID = 0; jointID < jointSet.size(); jointID++) {
		// store the joints which need to be processed
		std::vector< BVH::Joint* > processedJoints;
		// store the initial position of input joints
		Eigen::MatrixXd oldPos = Eigen::MatrixXd(jointSet[jointID] * 3, 1);
		// the position of moved joint
		int posPtr = jointPtr;
		for (int i = 0; i < jointSet[jointID]; i++) {
			oldPos(i * 3 + 0) = bvhFile->joints[posPtr]->pos_1;
			oldPos(i * 3 + 1) = bvhFile->joints[posPtr]->pos_2;
			oldPos(i * 3 + 2) = bvhFile->joints[posPtr]->pos_3;
			processedJoints.push_back(bvhFile->joints[posPtr]);
			posPtr++;
		}

		//std::cout << std::endl << oldPos << std::endl;

		//std::cout << std::endl << startJoint << std::endl;
		// store the final position of joints after IK
		Eigen::MatrixXd desPos = Eigen::MatrixXd(jointSet[jointID] * 3, 1);
		// the position of moved joint
		posPtr = jointPtr;
		for (int i = 0; i < jointSet[jointID]; i++) {
			desPos(i * 3 + 0) = positions[startJoint * 3 + i * 3 + 0];
			desPos(i * 3 + 1) = positions[startJoint * 3 + i * 3 + 1];
			desPos(i * 3 + 2) = positions[startJoint * 3 + i * 3 + 2];
			posPtr++;
			jointPtr++;
		}
		//std::cout << std::endl << desPos << std::endl;

		// ======================================== IK ======================================== 
		Eigen::MatrixXd Jaco = Eigen::MatrixXd(jointSet[jointID] * 3, jointSet[jointID] * 3);//(position x, y, z, channel x, y, z)
		//std::cout << std::endl << processedJoints[0]->name << std::endl;
		Jaco = fdJaco(processedJoints, frame_no, scale, oldMotion, startJoint);

		Eigen::MatrixXd deltaDegree = Eigen::MatrixXd(jointSet[jointID] * 3, 1);
		Eigen::MatrixXd TJaco = Eigen::MatrixXd(jointSet[jointID] * 3, jointSet[jointID] * 3);

		TJaco = Jaco * Jaco.transpose();

		// whether damped button clicked
		if (IsDamped == true)
			TJaco = TJaco + pow(lambda, 2) * Eigen::MatrixXd::Identity(jointSet[jointID] * 3, jointSet[jointID] * 3);

		Eigen::MatrixXd newChannels = Eigen::MatrixXd(jointSet[jointID] * 3, jointSet[jointID] * 3);
		newChannels = Jaco.transpose() * TJaco.inverse();
		
		

		deltaDegree = newChannels * (desPos - oldPos);
		
		//std::cout << std::endl << deltaDegree << std::endl;
		Eigen::MatrixXd deltaDegree_motion = Eigen::MatrixXd(bvhFile->channels.size() - posChannel, 1).setZero();
		//std::cout << std::endl << deltaDegree_motion << std::endl;
		for (int i = 0; i < bvhFile->channels.size() - posChannel; i++) {
			if (i >= startJoint * 3 + (3 - posChannel) && i < (startJoint + jointSet[jointID]) * 3 + (3 - posChannel)) {
				for (int j = 0; j < jointSet[jointID] * 3; j++) {
					deltaDegree_motion(i) = deltaDegree(j);
					i++;
				}
			}
			else	
				deltaDegree_motion(i) = 0;
		}
		//std::cout << std::endl << deltaDegree_motion << std::endl;
		//std::cout << std::endl << deltaDegree_motion << std::endl;

		//for (int i = posChannel; i < bvhFile->num_channel; i++) {
		//	std::cout << newMotion[i] << ", ";
		//}

		for (int i = posChannel; i < bvhFile->num_channel; i++) {
			newMotion[i] = newMotion[i] + deltaDegree_motion(i - posChannel);
		}
		//std::cout << std::endl << std::endl;
		//for (int i = posChannel; i < bvhFile->num_channel; i++) {
		//	std::cout << newMotion[i] << ", ";
		//}

		forwardK(frame_no, scale, newMotion);
		startJoint += jointSet[jointID];
	}

	return newMotion;
}

void BVHreader::myTranslatef(double x, double y, double z) {

	Eigen::MatrixXd T = Eigen::MatrixXd(4,4);
	T << 1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1;
	
	matrixStack.pop();
	matrixStack.push(currentMatrix*T);
	currentMatrix = matrixStack.top();
}

void BVHreader::myRotatef(double degree, double x, double y, double z) {

	Eigen::MatrixXd T = Eigen::MatrixXd(4,4);
	double m1 = cos(degree * 3.141593 / 180);
	double m2 = sin(degree * 3.141593 / 180);
	if (z == 1.0f) {
		T << m1, -m2, 0, 0,
			m2, m1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}
	if (y == 1.0f) {
		T << m1, 0, m2, 0,
			0, 1, 0, 0,
			-m2, 0, m1, 0,
			0, 0, 0, 1;
	}
	if (x == 1.0f) {
		T << 1, 0, 0, 0,
			0, m1, -m2, 0,
			0, m2, m1, 0,
			0, 0, 0, 1;
	}
	matrixStack.pop();
	matrixStack.push(currentMatrix * T);
	currentMatrix = matrixStack.top();
}

Eigen::MatrixXd BVHreader::myMultMatrixd(Eigen::MatrixXd V) {
	matrixStack.pop();
	matrixStack.push(currentMatrix * V);
	currentMatrix = matrixStack.top();
	return currentMatrix;
}

void BVHreader::myPushMatrix() {

	Eigen::MatrixXd tempM = currentMatrix;
	matrixStack.push(tempM);
	currentMatrix = matrixStack.top();
}

void BVHreader::myPopMatrix() {
	matrixStack.pop();
	currentMatrix = matrixStack.top();
}

void BVHreader::forwardK(int frame_no, float scale)
{
	BVHreader::forwardK(bvhFile->joints[0], bvhFile->motion + frame_no * bvhFile->num_channel, scale);
}

void BVHreader::forwardK(int frame_no, float scale, double* newMotion)
{
	
	BVHreader::forwardK(bvhFile->joints[0], newMotion, scale);
}

void  BVHreader::forwardK(BVH::Joint* joint, double* data, float scale)
{
	myPushMatrix();
	// ルート関節の場合は平行移動を適用
	if (joint->parent == NULL)
	{
		myTranslatef(data[0] * scale, data[1] * scale, data[2] * scale);
		BVH::Joint* root = joint;
		joint->pos_1 = data[0] * scale;
		joint->pos_2 = data[1] * scale;
		joint->pos_3 = data[2] * scale;

	}
	// 子関節の場合は親関節からの平行移動を適用
	else
	{
		myTranslatef(joint->offset[0] * scale, joint->offset[1] * scale, joint->offset[2] * scale);
	}
	// 親関節からの回転を適用（ルート関節の場合はワールド座標からの回転）
	int  i;
	for (i = 0; i < joint->channels.size(); i++)
	{
		BVH::Channel* channel = joint->channels[i];
		if (channel->type == BVH::X_ROTATION)
			myRotatef(data[channel->index], 1.0f, 0.0f, 0.0f);
		else if (channel->type == BVH::Y_ROTATION)
			myRotatef(data[channel->index], 0.0f, 1.0f, 0.0f);
		else if (channel->type == BVH::Z_ROTATION)
			myRotatef(data[channel->index], 0.0f, 0.0f, 1.0f);
	}
	// リンクを描画
	// 関節座標系の原点から末端点へのリンクを描画
	if (joint->children.size() == 0)
	{

	}
	// 関節座標系の原点から次の関節への接続位置へのリンクを描画
	if (joint->children.size() >= 1)
	{
		BVH::Joint* child = joint->children[0];
		myPushMatrix();
		Eigen::MatrixXd pos(4, 4);
		pos << child->offset[0] * scale, 0, 0, 0,
			child->offset[1] * scale, 0, 0, 0,
			child->offset[2] * scale, 0, 0, 0,
			1, 0, 0, 0;
		Eigen::MatrixXd resultM = myMultMatrixd(pos);
		child->pos_1 = resultM(0);
		child->pos_2 = resultM(1);
		child->pos_3 = resultM(2);
		myPopMatrix();
	}
	/*
	if (joint->children.size() == 1)
	{
		BVH::Joint* child = joint->children[0];
		myPushMatrix();
		Eigen::MatrixXd pos(4, 4);
		pos << child->offset[0] * scale, 0, 0, 0,
			child->offset[1] * scale, 0, 0, 0,
			child->offset[2] * scale, 0, 0, 0,
			1, 0, 0, 0;
		Eigen::MatrixXd resultM = myMultMatrixd(pos);
		child->pos_1 = resultM(0);
		child->pos_2 = resultM(1);
		child->pos_3 = resultM(2);	
		myPopMatrix();
	}
	// 全関節への接続位置への中心点から各関節への接続位置へ円柱を描画
	if (joint->children.size() > 1)
	{
		// 原点と全関節への接続位置への中心点を計算
		float  center[3] = { 0.0f, 0.0f, 0.0f };
		for (i = 0; i < joint->children.size(); i++)
		{
			BVH::Joint* child = joint->children[i];
			center[0] += child->offset[0];
			center[1] += child->offset[1];
			center[2] += child->offset[2];
		}
		center[0] /= joint->children.size() + 1;
		center[1] /= joint->children.size() + 1;
		center[2] /= joint->children.size() + 1;

		// 中心点から次の関節への接続位置へのリンクを描画
		for (i = 0; i < joint->children.size(); i++)
		{
			BVH::Joint* child = joint->children[i];
			
			myPushMatrix();
			myTranslatef(center[0] * scale, center[1] * scale, center[2] * scale);
			Eigen::MatrixXd pos(4,4);
			pos << child->offset[0] * scale, 0, 0, 0,
				child->offset[1] * scale, 0, 0, 0,
				child->offset[2] * scale, 0, 0, 0,
				1, 0, 0, 0;
			Eigen::MatrixXd resultM = myMultMatrixd(pos);
			child->pos_1 = resultM(0);
			child->pos_2 = resultM(1);
			child->pos_3 = resultM(2);
			myPopMatrix();
		}
	}
	*/
	// 子関節に対して再帰呼び出し
	for (i = 0; i < joint->children.size(); i++)
	{
		forwardK(joint->children[i], data, scale);
	}

	myPopMatrix();
}
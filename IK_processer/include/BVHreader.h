//////////////////////////////////////////////////////////////////////
//
//  University of Leeds
//  COMP 5823M Animation and Simulation
//  The reader of bvh file
//
//  November, 2021
//
//  -----------------------------
//  BVHreader.h
//  -----------------------------
//  
//  Read information and process them from bvh file
//  
////////////////////////////////////////////////////////////////////////
#pragma once
#include "BVH.h"
#include "Eigen/Dense"
#include <stack>
// the class itself

class BVHreader
{
public:
    BVH *bvhFile;   // the pointer of bvh file
    Eigen::MatrixXd currentMatrix;  // the current matrix
    stack<Eigen::MatrixXd> matrixStack; // used to doing matrix transformation
    double* newMotion;  // pointer of motion store in bvh file
    int posChannel = 0; // whether we consider the position of root joint, when this = 3 we don't consider the position change of root
    double lambda = 0;  // the initial lambda of transformation
    bool IsDamped = false; // whether damped button clicked
    bool IsControl = false; // whether control button clicked
    vector<double> allGains;    // used to store all gains

public:
    BVHreader(const char*);
    ~BVHreader();

    // used to do forward kinematics
    void myTranslatef(double x, double y, double z);
    void myRotatef(double degree, double x, double y, double z);
    Eigen::MatrixXd myMultMatrixd(Eigen::MatrixXd V);
    void myPushMatrix();
    void myPopMatrix();
    double* inverseK(BVH::Joint* joint, int frame_no, float scale, double x, double y, double z);
    double* inverseK(int frame_no, float scale, std::vector<double> positions);
    double* inverseK(int frame_no, float scale, std::vector<double> positions, std::vector<int> jointSet);
    double calculateLength(BVH::Joint* joint);

    void forwardK(int frame_no, float scale);
    void forwardK(int frame_no, float scale, double *newMotion);
    void forwardK(BVH::Joint* joint, double* data, float scale);
    Eigen::MatrixXd BVHreader::fdJaco(vector< BVH::Joint* >  allJoints, int frame_no, float scale, double* oldMotion);
    Eigen::MatrixXd BVHreader::fdJaco(vector< BVH::Joint* >  allJoints, int frame_no, float scale, double* oldMotion, int startJoint);
    void printJoint();
    void printJoint(BVH::Joint* joint);
    void printJoint(BVH::Joint* joint, int depth);
};
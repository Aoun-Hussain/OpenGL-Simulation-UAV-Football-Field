#pragma once
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <thread>
#include <mutex>

using namespace std;

class ECE_UAV
{
    /*
     main UAV class for taking care of threading
     velocities, and collisions
    */

public:
    ECE_UAV();

    void start();  //function to start the thread
    void stop();   //function to end the thread

    void initLocation(double x_, double y_, double z_); //initializes x,y,z coordinates
    void draw();           //draws UAV
    void updateLocation(); //updates location

    //getters
    double getX()
    {
        return x;
    };

    double getY()
    {
        return y;
    };

    double getZ()
    {
        return z;
    };

    double getVx()
    {
        return Vx;
    };

    double getVy()
    {
        return Vy;
    };

    double getVz()
    {
        return Vz;
    };

    //function to adjust location when Elastic Collision happens
    void setElasticCollision(double Vx_, double Vy_, double Vz_, double x, double y, double z);

    //function to update color
    void updateColor();

private:
    double color = 255.0;   //red color initial
    bool asc = false;      //flag for color oscillation

    double Fx, Fy, Fz, Fg;  //force parameters
    double Vx, Vy, Vz;      //velcity parameters
    double Fmax;
    double m;
    double x, y, z;          //position parameters


    std::thread uav_thread;   //threading variables
    std::mutex m_safe;

    double nx, ny, nz;  //normals
    double ax, ay, az;  //acceleration vectors
    double bx, by, bz;
    double theta;       //parameter for angular velocity
    double tx, ty, tz;  //target location at the sphere
};

void elasticCollision(ECE_UAV *uavs, int rank); //checks elastic collision

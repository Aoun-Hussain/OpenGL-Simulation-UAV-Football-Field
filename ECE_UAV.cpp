#define pi 3.1415926
#include <locale>
#include <thread>
#include <mutex>
#include "ECE_UAV.h"
#include <iostream>
#include <GLUT/glut.h>
#include <chrono>
#include <math.h>

using namespace std;

// constructor
ECE_UAV::ECE_UAV()
{
    /*
    constructor for initializing all 13 elements of a UAV class
     and then calculating normals, velocity, etc
    */

    //initializer for default variables
    m = 1;
    Fmax = 20;
    Fx = 0.0;
    Fy = 0.0;
    Fz = 0.0;
    Fg = m * 10.0;
    Vx = 0;
    Vy = 0;
    Vz = 0;
    
    //random normalized vector n to define its orbit
    nx = rand() % 11 - 5.0;
    ny = rand() % 11 - 5.0;
    nz = rand() % 11 - 5.0;

    theta = 0.0;

    double temp_x = 1.0;
    double temp_y = 0.0;
    double temp_z = 0.0;
    
    ax = ny * temp_z - nz * temp_y;
    ay = nz * temp_x - nx * temp_z;
    az = nx * temp_y - ny * temp_x;
    if (ax == 0.0 && ay == 0.0 && az == 0.0)
    {
        temp_x = 0.0;
        temp_y = 1.0;
        temp_z = 0.0;
        ax = ny * temp_z - nz * temp_y;
        ay = nz * temp_x - nx * temp_z;
        az = nx * temp_y - ny * temp_x;
    }
    
    bx = ny * az - nz * ay;
    by = nz * ax - nx * az;
    bz = nx * ay - ny * ax;
    
    //normalize a and b vectors
    double a = distanceOf(ax, ay, az, 0, 0, 0);
    ax /= a;
    ay /= a;
    az /= a;
    double b = distanceOf(bx, by, bz, 0, 0, 0);
    bx /= b;
    by /= b;
    bz /= b;
    
    //using the Parametric equation of a circle in 3D space to calculate the target location on that orbit
    tx = 10 * cos(2 * pi * theta) * ax + 10 * sin(2 * pi * theta) * bx;
    ty = 10 * cos(2 * pi * theta) * ay + 10 * sin(2 * pi * theta) * by;
    tz = 10 * cos(2 * pi * theta) * az + 10 * sin(2 * pi * theta) * bz + 50.0;
}

void ECE_UAV::initLocation(double x_, double y_, double z_)
{
    x = x_; //initializes the uav location using given location
    y = y_;
    z = z_;
}

void ECE_UAV::draw()
{
    /*
    draws UAV using the GLUT library as Icosahedron at the current location
    initialized at the start of the thread
    */

    glPushMatrix();
        glColor3f(this->color / 255.0, 0.0, 0.0); //dynamic color
        glTranslatef(x, y, z);
        glutSolidIcosahedron();
    glPopMatrix();
}

void ECE_UAV::updateLocation()
{
    /*
    This is the Thread function for updating location and velocity and acceleration
     at each time step
    */

    //Sleep for 5 seconds
    std::this_thread::sleep_for(std::chrono::seconds(5));

    //distance of current location to its target location
    double distanceToTarget = distanceOf(x, y, z, tx, ty, tz);

    //the time for each time step is 0.1s
    double t = 0.1;
    
    //get current velocity
    double v = sqrt(pow(Vx, 2) + pow(Vy, 2) + pow(Vz, 2));
    
    // adjust the target point velocity according to the location of uav and its speed
    //formula "theta = v / R"
    if (distanceToTarget >= 3.0)
    {
        // if they are too far
        theta += v * 0.1 / 100.0;
    }
    else
    {
        // when they are close enough,
        theta += v * 1.0 / 100.0;
    }
    
    //use the parametric equation of a circle in 3D space to update the target point's location
    double targetX = 10 * cos(2 * pi * theta) * ax + 10 * sin(2 * pi * theta) * bx;
    double targetY = 10 * cos(2 * pi * theta) * ay + 10 * sin(2 * pi * theta) * by;
    double targetZ = 10 * cos(2 * pi * theta) * az + 10 * sin(2 * pi * theta) * bz + 50.0;
    
    //calculate the direction vector from uav to its target
    double dx = targetX - x;
    double dy = targetY - y;
    double dz = targetZ - z;
    double d  = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    
    //get the relevant force vector
    Fx = dx / d * (Fmax - Fg);
    Fy = dy / d * (Fmax - Fg);
    Fz = dz / d * (Fmax - Fg);
    
    //get the relevant acceleration
    double ax = Fx / m;
    double ay = Fy / m;
    double az = Fz / m;
    
    //get the relevant velocity under that acceleration
    double Vx_ = Vx + ax * t;
    double Vy_ = Vy + ay * t;
    double Vz_ = Vz + az * t;
    double V_  = sqrt(pow(Vx_, 2) + pow(Vy_, 2) + pow(Vz_, 2));
    
    //get the distance from uav to the point(0,0,50)
    double distance = distanceOf(x, y, z, 0, 0, 50);

    if (distance >= 11.0)
    {
        if (V_ > 2.0)
        {
            //when it is approaching the point -> speed limit is 2m/s
            Vx_ /= V_ * 2.0;
            Vy_ /= V_ * 2.0;
            Vz_ /= V_ * 2.0;
        }
    }
    else
    {
        if (V_ > 10.0)
        {
            //when it is on the surface of the sphere -> speed limit is 2-10m/s
            Vx_ /= V_ * 10.0;
            Vy_ /= V_ * 10.0;
            Vz_ /= V_ * 10.0;
        }
    }
    
    //update the location according to curved speed
    x += 0.5 * (Vx + Vx_) * t;
    y += 0.5 * (Vy + Vy_) * t;
    z += 0.5 * (Vz + Vz_) * t;
    
    //set velocity into uav class
    Vx = Vx_;
    Vy = Vy_;
    Vz = Vz_;
    
    //set target location into uav class
    tx = targetX;
    ty = targetY;
    tz = targetZ;
}

void ECE_UAV::updateColor()
{
    /*
    for the oscillation of red color (255, 0, 0)
    */

    m_safe.lock();  //making thread safe

    // update the color from 128 to 255
    if (asc)
    {
        // if ascending, check whether it reached the upper bound
        if (color == 255.0)
        {
            // if yes, change into descending
            color--;
            asc = not asc;
        }
        else
        {
            // if no, keep ascending
            color++;
        }
    }
    else
    {
        // if decending, check whether it reached the lower bound
        if (color == 128.0)
        {
            // if yes, change into ascending
            color++;
            asc = not asc;
        }
        else
        {
            // if no, keep descending
            color--;
        }
    }

    m_safe.unlock();  //making thread safe
}

double distanceOf(double x1, double y1, double z1, double x2, double y2, double z2)
{
    /*
     function to calculate 3d distance between two points
    */
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

void ECE_UAV::start()
{
    uav_thread = std::thread(updateLocation(), this);
}

void ECE_UAV::stop()
{
    if (uav_thread.joinable())
    {
        uav_thread.join();
    }
}

void elasticCollision(ECE_UAV *uavs, int rank)
{
    /*
    helper function used in set Elastic collision
    Detects elastic collision and updates the location
    rank is the index of UAV in the UAV array
    */

    double x = uavs[rank].getX(), y = uavs[rank].getY(), z = uavs[rank].getZ();
    int closestUav = -1;
    double dist = numeric_limits<double>::max();
    
    //traverse all uavs
    for (int i = 0; i < 15; i++)
    {
        if (i == rank)
        {
            // skip itself
            continue;
        }
        
        // calculate distance
        double tmpDist = distanceOf(x, y, z, uavs[i].getX(), uavs[i].getY(), uavs[i].getZ());
        if (tmpDist <= dist && tmpDist <= 1.01)
        {
            closestUav = i;
            dist = tmpDist;
        }
    }
    
    if (closestUav == -1)
    {
        return;
    }
    else
    {
        //call function to update status of two
        uavs[rank].setElasticCollision(uavs[closestUav].getVx(), uavs[closestUav].getVy(), uavs[closestUav].getVz(),
                                       uavs[closestUav].getX(),  uavs[closestUav].getY(),  uavs[closestUav].getZ());
    }
}

void ECE_UAV::setElasticCollision(double Vx_, double Vy_, double Vz_, double x, double y, double z)
{
    /*
    called within the main thread to detect collision and change coordinates if found
    */

    m_safe.lock();

    Vx = Vx_;
    Vy = Vy_;
    Vz = Vz_;

    double midX = (this->x + x) / 2;
    double midY = (this->y + y) / 2;
    double midZ = (this->z + z) / 2;

    //force it to get away from each other by 2m
    double midDist = distanceOf(this->x, this->y, this->z, midX, midY, midZ);
    this->x = midX + (this->x - midX) / midDist * 1.0;
    this->y = midY + (this->y - midY) / midDist * 1.0;
    this->z = midZ + (this->z - midZ) / midDist * 1.0;

    m_safe.unlock();  //making thread safe
}




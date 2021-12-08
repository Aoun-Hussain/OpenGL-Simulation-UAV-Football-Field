/*
 References:
    https://solarianprogrammer.com/2018/11/19/cpp-reading-writing-bmp-images/
    https://thispointer.com//c11-multithreading-part-4-data-sharing-and-race-conditions/
    https://learnopengl.com/Getting-started/Hello-Triangle
    https://www.geeksforgeeks.org/multithreading-in-cpp/
*/

#include "ECE_UAV.h"
#include "BMP.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <GLUT/glut.h>
#include "iomanip"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <chrono>
#include <thread>

using namespace std;

//texture image struct
GLuint texture;
struct Image
{
    unsigned long sizeX;
    unsigned long sizeY;
    char* data;
};

typedef struct Image Image;

//class for reading bitmap, taken from link in references
BMP inBitmap;

//light0 attribute
GLfloat light0_ambient[] = { 0.2, 0.2, 0.2, 1.0 };

//light1 attribute
GLfloat light1_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };
GLfloat light1_position[] = { 5.0, 5.0, 8.0};
GLfloat light1_specular[] = { 0.3, 0.3, 0.3, 1.0 };

//press "ESC" to exit program
#define ESC 27

//press R to rotate the view
double rotateDegree = 0.0;

// press s/w to move the forward/backward
double forwardDist = 0.0;

//number of elements per UAV
const int numElements = 13;

//uav class array
ECE_UAV *uavs = new ECE_UAV[15];

void displayFF()
{
    /*
     Display the center sphere and the rectangular field
     Code adapted from online tutorials of GLUT
    */

    //display sphere
    glPushMatrix();
    glTranslatef(0.0, 0.0, 50.0);
    glColor3f(0.8, 0.8, 0.0);

    //draw the point at (0,0,50)
    glutSolidSphere(1.0, 100, 100);
    glColor3f(1.0, 1.0, 1.0);

    //draw the virtual sphere with center at (0,0,50) and a radius of 10
    glutWireSphere(10.0, 20, 20);
    glPopMatrix();

    //display field
    glPushMatrix();
    //enable texture
    glEnable(GL_TEXTURE_2D);
    //bind texture
    glBindTexture(GL_TEXTURE_2D, texture);

    glColor3f(0.0, 1.0, 0.0);
    //draw a quad
    glBegin(GL_QUADS);

    //set normal
    glNormal3f(0,0,1);
    //bind it with texture(0,1)
    glTexCoord2f(0, 1);

    //parameters to scale the field
    double scaleX = 1.13;
    double scaleY = 1.27;
    glVertex3f( 24.4 * scaleX,  55.0 * scaleY, 0.0);

    glTexCoord2f(0, 0);
    glVertex3f(-24.4 * scaleX,  55.0 * scaleY, 0.0);

    glTexCoord2f(1, 0);
    glVertex3f(-24.4 * scaleX, -55.0 * scaleY, 0.0);

    glTexCoord2f(1, 1);
    glVertex3f( 24.4 * scaleX, -55.0 * scaleY, 0.0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glPopMatrix();
}

void changeSize(int w, int h)
{
    /*
    Window size has been set/changed to w by h pixels.
    Code adapted from online tutorials of GLUT
    */

    float ratio = ((float)w) / ((float)h); //window aspect ratio
    glMatrixMode(GL_PROJECTION); //projection matrix is active
    glLoadIdentity(); //reset the projection
    gluPerspective(60.0, ratio, 0.1, 1000.0);
    glMatrixMode(GL_MODELVIEW); //return to modelview mode
    glViewport(0, 0, w, h); //set viewport (drawing area)
}

void drawUAVs()
{
    /*
     function to draw UAVs
     Code adapted from online tutorials of GLUT
    */

    for (int i = 0; i < 15; i++)
    {
        uavs[i].draw();
    }
}


void renderScene()
{
    /*
    Draw the entire scene
    We first update the camera location based on its distance from the
    origin and its direction.
    Code adapted from online tutorials of GLUT
    */

    //Clear color and depth buffers
    glClearColor(0.2, 0.2, 0.2, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //Reset transformations
    glLoadIdentity();

    //rotate the field according to key press
    glTranslatef(4.0, 4.0, 0.0);
    glRotatef(rotateDegree, 0.0, 0.0, 1.0);
    glTranslatef(-4.0, -4.0, 0.0);

    //set the look-at according to key press
    gluLookAt(-200 + forwardDist, 0.0, 80.0,
               0.0, 0.0, 50.0,
               0.0, 0.0,  1.0);

    //set model view
    glMatrixMode(GL_MODELVIEW);

    displayFF(); //display ff, sphere, and UAVs
    drawUAVs();

    glutSwapBuffers();
}

void timerFunction(int id)
{
    /*
    shoots  after 100ms
    */

    glutPostRedisplay();
    glutTimerFunc(100, timerFunction, 0);
}

void keyboard(unsigned char key, int x, int y)
{
    /*
    for my ease, shortcuts for simulation video
    idea and code taken from online tutorials
    */

    switch (key)
    {
        case ESC: // press ESC to quit
            exit(0);
            break;
        case 'r': // press r to rotate the field
            rotateDegree += 10.0;
            glutPostRedisplay();
            break;
        case 'R': // same as R
            rotateDegree += 10.0;
            glutPostRedisplay();
        case 'w': // press w to move the camera forward
            forwardDist += 10.0;
            glutPostRedisplay();
            break;
        case 'W': // same as w
            forwardDist += 10.0;
            glutPostRedisplay();
        case 's': // press s to move the camera backward
            forwardDist -= 10.0;
            glutPostRedisplay();
            break;
        case 'S': // same as s
            forwardDist -= 10.0;
            glutPostRedisplay();
        default:
            break;
    }
}


void mainOpenGL(int argc, char**argv)
{
    /*
    main loop to render all objects
    Code adapted from online tutorials of GLUT
    */

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

    //set window size and location
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(400, 400);

    //create window, set background color and title
    glutCreateWindow(argv[0]);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    
    //set and enable modes
    glShadeModel(GL_SMOOTH);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    
    //set light
    glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light1_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light1_position);

    //enable light
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    
    //set up texture part
    inBitmap.read("../ff.bmp");
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //scale linearly when image bigger than texture
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //scale linearly when image smalled than texture
    glTexImage2D(GL_TEXTURE_2D, 0, 3, inBitmap.bmp_info_header.width, inBitmap.bmp_info_header.height, 0,
            GL_BGR_EXT, GL_UNSIGNED_BYTE, &inBitmap.data[0]);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    
    //set recall functions
    glutReshapeFunc(changeSize);
    glutDisplayFunc(renderScene);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(100, timerFunction, 0);
    
    //enter the opengl main loop through first thread and never return
    //other threads keep updating UAV locations
    glutMainLoop();
}

int main(int argc, char**argv)
{
    /*
    main loop for the simulation and multi threading
    Code adapted from online tutorials of GLUT
    The main thread only renders the first view
    and then other 15 threads keep updating the location and look for collisions
    All threads end when join is called after approximately 60 seconds
    */

    int rank;  //tells the index of UAV in the array

    //initializing the uav's location
    for (int i = 0; i < 15; i++)
    {
        double x = i % 3 * 24.4 - 24.4;
        double y = i / 3 * 27.5 - 55.0;
        double z = 0;
        uavs[i].initLocation(x, y, z);
    }

    mainOpenGL(argc, argv);  //main loop for rendering in thread 0

    for (int rank = 0; rank < 15; rank ++)   //now starting other 15 threads
    {
        uavs[rank].start();
    }

    while (1)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now(); //time starts when simulation starts

        for (rank = 0; rank < 15; rank ++) //iterating on the UAV array in the first thread
        {
            elasticCollision(uavs, rank);

            //update the color of uavs
            uavs[rank].updateColor();

            //update the location of uavs
            uavs[rank].updateLocation();
        }

        //end the program if 60 seconds have elapsed after start
        //but there will be execution delays so its more than 60 seconds
        if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - begin).count() == 60000)
        {
            for (int rank = 0; rank < 15; rank ++)
            {
                uavs[rank].stop();  //terminating other threads
            }
            std::this_thread.join();    //terminating thread 0
            break;
        }
    }

    return 0;
}

/* David Warrick
 * game.cpp
 *
 * This file is the main file that actually runs the game/physics engine.
 */

#include <GL/glew.h>
#include <GL/glut.h>
#include <math.h>
#define _USE_MATH_DEFINES
#include <float.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <ctime>

#include "object.h"
#include "physicalObject.h"
#include "boundaryObject.h"
#include "util.h"
#include "halfEdge.h"

/*** Movement constants ***/
/* Keep in mind that positions are updated every frame, not every second, hence
 * the seemingly low values */
// Every frame an object's velocity is multiplied by this amount
#define SLOWDOWN 0.99f
// Don't make g 9.8 because this applies to every frame, not every second...
#define G .2f
// Value added to the player's velocity when the player controls their character
#define MOVEMENTACCEL 0.2f
// Maximum movement speed for the player character
#define MOVEMENTSPEEDMAX .7

#define EPSILON 0.01f

// Set the distance from which the camera will follow the player
#define TRACKDISTX 0
#define TRACKDISTY 1.5f
#define TRACKDISTZ 5
// The multiplier for rotation speed in TRACKPLAYER mode
#define ANGLEMULT 0.001f

using namespace std;

struct Point_Light
{
    float position[4];
    float color[3];
    float attenuation_k;
};

/******************************************************************************/
// Function prototypes

void init(void);
void setupObjects();
void reshape(int width, int height);
void display(void);

void init_lights();
void set_lights();
void draw_objects();

void physics();
void verletIntegration();
void draw_text(int x, int y, string target);
void display_all_text();

void onMouse(int button, int state, int x, int y);
void onMotion(int x, int y);
Vector3d get_arcball_vector(int x, int y);
void key_pressed(unsigned char key, int x, int y);
Vector3d get_camera_direction();

double rad2deg(double);
double deg2rad(double);

void create_lights();

/******************************************************************************/
// Global variables
int xres = 500, yres = 500;
int drawState = 0;

int groundtmp = 0;
int score;
int tempiterator = 0;

// Toggle for printing debug values. Set at program initiation.
bool printDebug = false;

// Vector to store the objects in the game
vector<PhysicalObject> objects;
// Player character. The player is always the first element in the objects
// vector.
int player = 0;
// Vector to store the objects that represent the world boundaries
// (ground, walls, etc.)
vector<BoundaryObject> boundaries;

bool jumped = false;

/*----- Arcball Globals -----*/
int last_mx = 0, last_my = 0, cur_mx = 0, cur_my = 0;
bool arcball_on = false;
double mouse_scale_x, mouse_scale_y;

const double step_size = 0.2;
const double x_view_step = 90.0, y_view_step = 90.0;
double x_view_angle = 0, y_view_angle = 0;
Vector3d rotAxis(0,1,0);
// Matrix to store total rotations from the arcball
MatrixXd total_rotate = MatrixXd::Identity(4,4);

/*----- Camera globals -----*/
// Flag for whether or not to use the player-tracking camera or the scene camera
bool trackPlayer = true;

double cam_position[] = {0, 2, 9};

double cam_orientation_axis[] = {1, 1, 1};

double cam_orientation_angle = 0; // degrees

// These values are only use if TRACKPLAYER is 1
float xAngle = 0;
float yAngle = 0;
float tplast_mx = 0, tplast_my = 0, tpcur_mx = 0, tpcur_my = 0;

double near_param = 1, far_param = 35,
      left_param = -1, right_param = 1,
      top_param = 1, bottom_param = -1;
vector<Point_Light> lights;

/******************************************************************************/
// Function declarations

// Function to initialize and set up the program.
void init(void)
{
    // Set the background to white
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CW);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
   
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(left_param, right_param,
              bottom_param, top_param,
              near_param, far_param);
    glMatrixMode(GL_MODELVIEW);
    
    setupObjects();
    create_lights();
    
    init_lights();
}

// Function to perform the initial setting up of objects. For now it's just a
// floor with a pair of balls
void setupObjects()
{
    MatrixXd floorScl = matrix4to3(get_scale_mat(25, 1, 25));
    MatrixXd floorRot = matrix4to3(get_rotate_mat(0, 1, 0, 0));
    Vector3d floorTrans(0, -5, 0);
    Vector3d floorRGB(0.1, 0.1, 0.1);
    
    MatrixXd wallLRScl = matrix4to3(get_scale_mat(1, 25, 25));
    MatrixXd wallLRRot = matrix4to3(get_rotate_mat(0, 1, 0, 0));
    Vector3d wallRTrans(14.5, 12, 0);
    Vector3d wallLTrans(-14.5, 12, 0);
    Vector3d wallRGB(0.2, 0.2, 0.2);
    
    MatrixXd wallFBScl = matrix4to3(get_scale_mat(1, 25, 25));
    MatrixXd wallFBRot = matrix4to3(get_rotate_mat(0, 1, 0, deg2rad(90)));
    Vector3d wallFTrans(0, 12, 14.5);
    Vector3d wallBTrans(0, 12, -14.5);
    
    MatrixXd objScl = matrix4to3(get_scale_mat(0.5, 0.5, 0.5));
    MatrixXd objRot = matrix4to3(get_rotate_mat(0, 1, 0, 0));
    Vector3d objTrans1(0, 8, 0);
    Vector3d objTrans2(5, 8, 0);
    Vector3d objVel(0, 0, 0);
    Vector3d objRGB1(0, 0, 1);
    Vector3d objRGB2(1, 0, 0);
    
    BoundaryObject floor (floorScl, floorRot, floorTrans, floorRGB, 1, .1, .1, GROUND, .9);
    BoundaryObject wallR (wallLRScl, wallLRRot, wallRTrans, wallRGB, 1, .1, .1, WALL_RIGHT, .95);
    BoundaryObject wallB (wallFBScl, wallFBRot, wallBTrans, wallRGB, 1, .01, .01, WALL_BACK, .95);
    PhysicalObject obj1 (objScl, objRot, objTrans1, objRGB1, 1, 1, 1, objVel, 1);
    PhysicalObject obj2 (objScl, objRot, objTrans2, objRGB2, 1, 1, 1, objVel, 1);
    
    boundaries.push_back(floor);
    boundaries.push_back(wallR);
    boundaries.push_back(wallB);
    objects.push_back(obj1);
    objects.push_back(obj2);
}

// Function to handle window resizing
void reshape(int width, int height)
{
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;
    glViewport(0, 0, width, height);
    mouse_scale_x = (double) (right_param - left_param) / (double) width;
    mouse_scale_y = (double) (top_param - bottom_param) / (double) height;
    glutPostRedisplay();
}

// Function to handle rendering
void display(void)
{
    //clock_t start = clock();
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Handle all of the position and velocity updates
    physics();

    glLoadIdentity();

    if (trackPlayer) {
        // Using gluLookAt for the camera
        Vector3d viewPoint = objects.at(player).getPosition();
        
        Vector4d camPos(TRACKDISTX, TRACKDISTY, TRACKDISTZ, 1);
        MatrixXd rot = get_rotate_mat(0, 1, 0, yAngle);
        MatrixXd trans = get_translate_mat(viewPoint(0), viewPoint(1), viewPoint(2));
        camPos = trans * rot * camPos;
        
        gluLookAt(camPos(0), camPos(1), camPos(2),
                  viewPoint(0), viewPoint(1), viewPoint(2), 
                  0, 1, 0);
    }
    else {
        // Apply camera transformations
        glRotatef(-cam_orientation_angle, cam_orientation_axis[0], 
                  cam_orientation_axis[1], cam_orientation_axis[2]);
        glTranslatef(-cam_position[0], -cam_position[1], -cam_position[2]);

        // Apply the accumulative rotations from the arcball
        GLfloat rot[16] = {total_rotate(0,0), total_rotate(1,0), total_rotate(2,0), total_rotate(3,0),
                           total_rotate(0,1), total_rotate(1,1), total_rotate(2,1), total_rotate(3,1),
                           total_rotate(0,2), total_rotate(1,2), total_rotate(2,2), total_rotate(3,2),
                           total_rotate(0,3), total_rotate(1,3), total_rotate(2,3), total_rotate(3,3)};
        glMultMatrixf(rot);
    }
    
    
    // Draw the scene
    set_lights();
    draw_objects();

    display_all_text();        

    glutSwapBuffers();
    
    // Code for counting FPS
    /*glFinish();
    
    clock_t currentTime = clock();
    
    double duration = (currentTime - start) / (double) CLOCKS_PER_SEC;
    
    printf("%f seconds to draw frame\n", duration);*/
}

// Function to enable lights
void init_lights()
{
    glEnable(GL_LIGHTING);
    int num_lights = lights.size();
    for(int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;
        glEnable(light_id);
        glLightfv(light_id, GL_AMBIENT, lights[i].color);
            glLightfv(light_id, GL_DIFFUSE, lights[i].color);
            glLightfv(light_id, GL_SPECULAR, lights[i].color);
        glLightf(light_id, GL_QUADRATIC_ATTENUATION, lights[i].attenuation_k);
    }
}

// Function to set the position of the lights
void set_lights()
{
    int num_lights = lights.size();
    
    for(int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;
        
        glLightfv(light_id, GL_POSITION, lights[i].position);
    }
}

// Contains all draw_text calls for convenience.
void display_all_text()
{
    char scoretxt[10];
    sprintf(scoretxt, "%d", score);

    char scoretxt2[20];
    strcpy(scoretxt2, "Score: ");
    strcat(scoretxt2, scoretxt);

    draw_text(10, yres - 20, scoretxt2);    
    draw_text(10, yres - 40, "Davy S-ranked Tharja");    

}

// Draws the text used for hp, score, etc. Exactly what it says on the tin.
void draw_text(int x, int y, string target)
{
    glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, xres, 0.0, yres);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor3f(0.0f, 1.0f, 0.0f);

    glRasterPos2i(x, y);
    void * font = GLUT_BITMAP_9_BY_15;
    for (string::iterator i = target.begin(); i != target.end(); ++i)
    {
        char c = *i;
        glutBitmapCharacter(font, c);
    }  

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glEnable(GL_TEXTURE_2D);
   
}

// Function to actually render objects to the screen
void draw_objects()
{
    // Set background to white
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glEnable(GL_COLOR_MATERIAL);
    //glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    float colrs1[3]; float colrs2[3]; float colrs3[3]; 
    colrs1[0]=.5; colrs1[1]=.5; colrs1[2]=.5;
    glMaterialfv(GL_FRONT, GL_AMBIENT, colrs1);

    colrs2[0]=.9; colrs2[1]=.9; colrs2[2]=.9;
    glMaterialfv(GL_FRONT, GL_DIFFUSE, colrs2);

    colrs3[0]=.3; colrs3[1]=.3; colrs3[2]=.3;    
    glMaterialfv(GL_FRONT, GL_SPECULAR, colrs3);

    glMaterialf(GL_FRONT, GL_SHININESS, 8);

    // Draw every object in the scene
    for (int j = 0; j < objects.size(); j++)
    {
        vector<HEF*> *halfFaces = objects.at(j).getHalfFaces();
        vector<HEV*> *halfVertices = objects.at(j).getHalfVertices();
        vector<Vector3d> *normals = objects.at(j).getNormals();
        Vector3d RGB = objects.at(j).getRGB();
        
        // If object was just hit
        if(objects.at(j).get_hitstun() > 0)
        {
            RGB(0) = 1.0;
            RGB(1) = 1.0;
            RGB(2) = 0.8;
            objects.at(j).reduce_hitstun();
        }

        float alpha = objects.at(j).getAlpha();
        
        for (int i = 0; i < halfFaces->size(); i++)
        {
            int ind1 = halfFaces->at(i)->a;
            int ind2 = halfFaces->at(i)->b;
            int ind3 = halfFaces->at(i)->c;
        
            glPushMatrix();
            
            // Draw the face
            glColor4f(RGB(0), RGB(1), RGB(2), alpha);
            glBegin(GL_TRIANGLES);

            glNormal3f(normals->at(ind1)(0), normals->at(ind1)(1),
                       normals->at(ind1)(2));
            glVertex3f(halfVertices->at(ind1)->x, halfVertices->at(ind1)->y,
                       halfVertices->at(ind1)->z);

            glNormal3f(normals->at(ind2)(0), normals->at(ind2)(1),
                       normals->at(ind2)(2));
            glVertex3f(halfVertices->at(ind2)->x, halfVertices->at(ind2)->y,
                       halfVertices->at(ind2)->z);

            glNormal3f(normals->at(ind3)(0), normals->at(ind3)(1),
                       normals->at(ind3)(2));
            glVertex3f(halfVertices->at(ind3)->x, halfVertices->at(ind3)->y,
                       halfVertices->at(ind3)->z);

            glEnd();
            
            glPopMatrix();
        }
    }
    // Draw the boundaries
    for (int j = 0; j < boundaries.size(); j++)
    {
        vector<HEF*> *halfFaces = boundaries.at(j).getHalfFaces();
        vector<HEV*> *halfVertices = boundaries.at(j).getHalfVertices();
        vector<Vector3d> *normals = boundaries.at(j).getNormals();
        Vector3d RGB = boundaries.at(j).getRGB();
        float alpha = boundaries.at(j).getAlpha();
        
        for (int i = 0; i < halfFaces->size(); i++)
        {
            int ind1 = halfFaces->at(i)->a;
            int ind2 = halfFaces->at(i)->b;
            int ind3 = halfFaces->at(i)->c;
        
            glPushMatrix();
            
            // Draw the face
            glColor4f(RGB(0), RGB(1), RGB(2), alpha);
            glBegin(GL_TRIANGLES);

            glNormal3f(normals->at(ind1)(0), normals->at(ind1)(1),
                       normals->at(ind1)(2));
            glVertex3f(halfVertices->at(ind1)->x, halfVertices->at(ind1)->y,
                       halfVertices->at(ind1)->z);

            glNormal3f(normals->at(ind2)(0), normals->at(ind2)(1),
                       normals->at(ind2)(2));
            glVertex3f(halfVertices->at(ind2)->x, halfVertices->at(ind2)->y,
                       halfVertices->at(ind2)->z);

            glNormal3f(normals->at(ind3)(0), normals->at(ind3)(1),
                       normals->at(ind3)(2));
            glVertex3f(halfVertices->at(ind3)->x, halfVertices->at(ind3)->y,
                       halfVertices->at(ind3)->z);

            glEnd();
            
            glPopMatrix();
        }
    }
}

/* This function handles all of the "physics" that occur in a single frame, i.e.
 * collision detection and updating objects' position and velocity.
 */
void physics()
{
    vector<Vector3d> netNewSpeeds;
    vector<Vector3d> newPositions;
    for (int i = 0; i < objects.size(); i++) {
        Vector3d zeros(0,0,0);
        netNewSpeeds.push_back(zeros);
        newPositions.push_back(objects.at(i).getPosition());
    }
    
    /* Handle collision detection first. If two objects collide, calculate their
     * updated speed and add it to their "net new speed". */
    // The currently implemented collision detection is a naive check-every-
    // -object-against-every-other-object algorithm. We will want to change
    // this later.
    for (int i = 0; i < objects.size(); i++) {
        // Don't want to make object comparisons that have already happened,
        // hence starting the second loop at i instead of 0. The +1 is so
        // we don't compare an object to itself.
        for (int j = i+1; j < objects.size(); j++) {
            if(objects.at(i).collidedWith(&objects.at(j))) {
                // Test for "enemy hit detection"
                score+=100;
                objects.at(j).set_hitstun();
       //         printf("Collided with %d\n", j);            

                float mI = objects.at(i).getMass();
                float mJ = objects.at(j).getMass();
                Vector3d vI = objects.at(i).getVelocity();
                Vector3d vJ = objects.at(j).getVelocity();
                
                Vector3d newVI = (mI - mJ) / (mI + mJ) * (vI - vJ) + vJ;
                Vector3d newVJ = (2 * mI) / (mI + mJ) * (vI - vJ) + vJ;
                
                netNewSpeeds.at(i) = netNewSpeeds.at(i) + newVI;
                netNewSpeeds.at(j) = netNewSpeeds.at(j) + newVJ;
                
                // Make sure the objects aren't within each other
                Vector3d direc = objects.at(j).getPosition() -
                                 objects.at(i).getPosition();
                Vector3d obj1Impact = objects.at(i).getFarthestPointInDirection(direc);
                Vector3d obj2Impact = objects.at(j).getFarthestPointInDirection(direc * -1);
                Vector3d objInnerDist = objects.at(i).getPosition() - obj1Impact;
                newPositions.at(i) = obj2Impact + objInnerDist;
            }
        }
    }
    
    // Apply the new net speed to every object. Also ensure that the objects
    // aren't leaving the world boundaries, and apply any slowdown or gravity
    // constants.
    for (int i = 0; i < objects.size(); i++) {
        Vector3d newSpeed = netNewSpeeds.at(i);
        // If the newSpeed vector is zero, then this object didn't collide with
        // anything. Maintain its velocity.
        if(vecIsZero(newSpeed)) {
            newSpeed = objects.at(i).getVelocity();
        }
        // Apply a slow down factor in every direction, to simulate air
        // resistance and friction
        newSpeed = newSpeed * SLOWDOWN;
        
        bool applyGravity = true;
        Vector3d pos = newPositions.at(i);
        
        // Check if the object has hit any world boundaries, and if it has,
        // alter the object's velocity accordingly.
        // The current method of during this assumes that all world boundaries
        // are rigid walls at 90 degree angles. If you want slopes, you'll need
        // something more complex...
        for (int j = 0; j < boundaries.size(); j++) {
            if(objects.at(i).collidedWith(&boundaries.at(j))) {
                objects.at(i).set_grounded(false);
                float dampen = boundaries.at(j).getDampening();
                // Ensure that the object is not within the boundary
                Vector3d objBottom, objInnerDist;
                Vector3d direc(0, 0, 0);
                float newPos = 0;
                switch(boundaries.at(j).getBoundaryType()) {                    
                    case GROUND:
                        if (newSpeed(1) < EPSILON) {
                            // Don't bother applying acceleration due to gravity
                            // if the object is at vertical rest on the ground
                            objects.at(i).set_grounded(true);
                        }
                        if (newSpeed(1) <= 0) {
                            newSpeed(1) = newSpeed(1) * -1 * dampen;
                        }
                        direc(1) = -1;                        
                        objBottom = objects.at(i).getFarthestPointInDirection(direc);
                        objInnerDist = objects.at(i).getPosition() - objBottom;
                        newPos = boundaries.at(j).getBoundary() + objInnerDist(1);                    
                        if (newPos > pos(1)) pos(1) = newPos;
                        break;
                    case CEILING:
                        if (newSpeed(1) >= 0) {
                            newSpeed(1) = newSpeed(1) * -1 * dampen;
                        }
                        direc(1) = 1;
                        objBottom = objects.at(i).getFarthestPointInDirection(direc);
                        objInnerDist = objects.at(i).getPosition() - objBottom;
                        pos(1) = boundaries.at(j).getBoundary() + objInnerDist(1);
                        if (newPos < pos(1)) pos(1) = newPos;
                        break;
                    case WALL_LEFT:
                        if (newSpeed(0) <= 0) {
                            newSpeed(0) = newSpeed(0) * -1 * dampen;
                        }
                        direc(0) = -1;
                        objBottom = objects.at(i).getFarthestPointInDirection(direc);
                        objInnerDist = objects.at(i).getPosition() - objBottom;
                        newPos = boundaries.at(j).getBoundary() + objInnerDist(0);
                        if (newPos > pos(0)) pos(0) = newPos;
                        break;
                    case WALL_RIGHT:
                        if (newSpeed(0) >= 0) {
                            newSpeed(0) = newSpeed(0) * -1 * dampen;
                        }
                        direc(0) = 1;
                        objBottom = objects.at(i).getFarthestPointInDirection(direc);
                        objInnerDist = objects.at(i).getPosition() - objBottom;
                        newPos = boundaries.at(j).getBoundary() + objInnerDist(0);
                        if (newPos < pos(0)) pos(0) = newPos;
                        break;
                    case WALL_FRONT:
                        if (newSpeed(2) >= 0) {
                            newSpeed(2) = newSpeed(2) * -1 * dampen;
                        }
                        direc(2) = 1;
                        objBottom = objects.at(i).getFarthestPointInDirection(direc);
                        objInnerDist = objects.at(i).getPosition() - objBottom;
                        newPos = boundaries.at(j).getBoundary() + objInnerDist(2);
                        if (newPos < pos(2)) pos(2) = newPos;
                        break;
                    case WALL_BACK:
                        if (newSpeed(2) <= 0) {
                            newSpeed(2) = newSpeed(2) * -1 * dampen;
                        }
                        direc(2) = -1;
                        objBottom = objects.at(i).getFarthestPointInDirection(direc);
                        objInnerDist = objects.at(i).getPosition() - objBottom;
                        newPos = boundaries.at(j).getBoundary() + objInnerDist(2);
                        if (newPos > pos(2)) pos(2) = newPos;
                        break;
                    default:
                        break;
                }
            }
        }
        
        if(applyGravity) {
            // Apply gravity
            newSpeed(1) = newSpeed(1) - G;
        }
    
        objects.at(i).setVelocity(newSpeed);
        
        objects.at(i).setPosition(pos + newSpeed);
    }
}

// Common video-game collision physics algorith implementation
void verletIntegration()
{
    
}

// Simple function to convert an angle in degrees to radians
double deg2rad(double angle)
{
    return angle * M_PI / 180.0;
}
// Simple function to convert an angle in radians to degrees
double rad2deg(double angle)
{
    return angle * (180.0 / M_PI);
}

// Function to handle mouse click events
void onMouse(int button, int state, int x, int y)
{
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        arcball_on = true;
        if (trackPlayer) {
            tplast_mx = tpcur_mx = x;
            tplast_my = tpcur_my = y;
        }
        else {
            last_mx = cur_mx = x;
            last_my = cur_my = y;
        }
    }
    else
    {
        arcball_on = false;
    }
}
// Function to handle mouse movement
void onMotion(int x, int y)
{
    if (trackPlayer) {
        if (arcball_on) tpcur_mx = x;
        if (tpcur_mx != tplast_mx)
        {
            yAngle = yAngle + (tpcur_mx - tplast_mx)*ANGLEMULT;
            // Make sure that the angle is within [0, 360)
            while (yAngle < 0) yAngle += 360;
            while (yAngle >= 360) yAngle -= 360;
        }
    }
    else {
        if(arcball_on)
        {
            cur_mx = x;
            cur_my = y;
        }
        if (cur_mx != last_mx || cur_my != last_my)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            Vector3d va = get_arcball_vector(last_mx, last_my);
            Vector3d vb = get_arcball_vector(cur_mx, cur_my);

            double value = 1.0f < va.dot(vb) ? 1.0f : va.dot(vb);
            double angle = acos(value);

            rotAxis = va.cross(vb);
            MatrixXd rotateMat = get_rotate_mat(rotAxis(0), rotAxis(1), rotAxis(2), angle);
            total_rotate = total_rotate * rotateMat;

            glutPostRedisplay();

            last_mx = cur_mx;
            last_my = cur_my;
            
        }
    }
}
// Function to calculate the arcball surface point
Vector3d get_arcball_vector(int x, int y)
{
    Vector3d P((double) x / (double) glutGet(GLUT_WINDOW_WIDTH) * 2 - 1.0,
               (double) y / (double) glutGet(GLUT_WINDOW_HEIGHT) * 2 - 1.0,
               0.0);
    P(1) = -P(1);
    double OP_squared = (P(0) * P(0)) + (P(1) * P(1));
    
    if (OP_squared <= 1*1)
    {
        P(2) = sqrt(1*1 - OP_squared);
    }
    else
    {  
        P.normalize();
    }
    return P;
}

// Function to handle key-press events.
void key_pressed(unsigned char key, int x, int y)
{
    if (key == 'q' || key == 'Q') // Quit
    {
        exit(0);
    }
    // Only do all of the player-move calculations if the button press
    // corresponds to a player move
    if (key == 'w' || key == 'W' || key == 'a' || key == 'A' || key == 's' ||
        key == 'S' || key == 'd' || key == 'D') {
        // Find a way to make all movement relative to the camera. (Don't 
        // actually change the player's position here, only their velocity. The 
        // physics() function handles changing their position)
        Vector3d newVel = objects.at(player).getVelocity();
        // Get the total camera transform, so we can project down to the x- and
        // z-axis to get the "forward" direction
        Vector3d camDirection = get_camera_direction();
        if (key == 'w' || key == 'W') // Player move forward
        {
            // Nothing to do here
        }
        if (key == 'a' || key == 'A') // Player move left
        {
            // Rotate the direction vector 90 degrees left
            camDirection = matrix4to3(get_rotate_mat(0, 1, 0, deg2rad(90)))
                           * camDirection;
        }
        if (key == 's' || key == 'S') // Player move backward
        {
            // negate the direction vector
            camDirection = camDirection * -1;
        }
        if (key == 'd' || key == 'd') // Player move right
        {
            // Rotate the direction vector 90 degrees right
            camDirection = matrix4to3(get_rotate_mat(0, 1, 0, deg2rad(270)))
                           * camDirection;
        }
        camDirection(1) = 0; // get rid of the y-component
        camDirection.normalize();
        // We can do this outside the if statements since all the if statements
        // take care of is changing the direction of the vector accordingly
        Vector3d vel = newVel + MOVEMENTACCEL * camDirection;
        // Only add the speed if the total velocity does not exceed the
        // maximum
        if (vel.norm() < MOVEMENTSPEEDMAX) {
            newVel = vel;
        }
        
        objects.at(player).setVelocity(newVel);
    }
    if (key == 32 && !jumped) // Spacebar
    {
        Vector3d newVel = objects.at(player).getVelocity();
        // Need a more sophisticated check to make sure that the player can't
        // jump in midair (unless we want to call it a feature instead of a bug)
        if(newVel(1) < 1 && objects.at(player).get_grounded()) {
            newVel(1) = 2;
            objects.at(player).set_grounded(false);
        }
        objects.at(player).setVelocity(newVel);
    }
    if (key == 'p' || key == 'P')
    {
        trackPlayer = !trackPlayer;
    }
    glutPostRedisplay();
}

void specialInput(int key, int x, int y)
{
    if (key == GLUT_KEY_LEFT && trackPlayer)
    {
        yAngle += 20 * ANGLEMULT;
    }
    if (key == GLUT_KEY_RIGHT && trackPlayer)
    {
        yAngle -= 20 * ANGLEMULT;
    }
    
    glutPostRedisplay();
}

Vector3d get_camera_direction()
{
    Vector3d direction(0, 0, 0);
    if (trackPlayer) {
        Vector3d viewPoint = objects.at(player).getPosition();
        
        Vector4d camPos(TRACKDISTX, TRACKDISTY, TRACKDISTZ, 1);
        MatrixXd rot = get_rotate_mat(0, 1, 0, yAngle);
        MatrixXd trans = get_translate_mat(viewPoint(0), viewPoint(1), viewPoint(2));
        camPos = trans * rot * camPos;
        
        direction = objects.at(player).getPosition() - vector4to3(camPos);
    }
    else {
        Vector4d directionTmp(0, 0, 0, 1);

        MatrixXd cameraT = get_translate_mat(-cam_position[0],
                                             -cam_position[1],
                                             -cam_position[2]);
        MatrixXd cameraR = get_rotate_mat(cam_orientation_axis[0],
                                          cam_orientation_axis[1],
                                          cam_orientation_axis[2],
                                          deg2rad(-cam_orientation_angle));
        MatrixXd cameraC = cameraR * cameraT * total_rotate;
        
        directionTmp = cameraC * directionTmp;
        
        direction = vector4to3(directionTmp);
    }
    
    return direction;
}

void create_lights()
{
    ////////////////////////////////////////////////////////////////////////////
    // Light 1 Below
    ////////////////////////////////////////////////////////////////////////////
    
    Point_Light light1;
    
    light1.position[0] = -1;
    light1.position[1] = 0;
    light1.position[2] = 1;
    light1.position[3] = 1;
    
    light1.color[0] = 1;
    light1.color[1] = 1;
    light1.color[2] = 1;
    light1.attenuation_k = 0;
    
    lights.push_back(light1);
}

void continuousAnimation()
{
    drawState++;
    if (drawState >= 5000)
    {
        drawState = 0;
        glutPostRedisplay();
    }
}

int main(int argc, char* argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0,0);
    glutCreateWindow("game");

    init();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(onMouse);
    glutMotionFunc(onMotion);
    glutIdleFunc(continuousAnimation);
    glutKeyboardFunc(key_pressed);
    glutSpecialFunc(specialInput);

    glutMainLoop();
}


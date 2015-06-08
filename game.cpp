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

#include "object.h"
#include "physicalObject.h"
#include "util.h"
#include "halfEdge.h"

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
void reshape(int width, int height);
void display(void);

void init_lights();
void set_lights();
void draw_objects();

void physics();

void onMouse(int button, int state, int x, int y);
void onMotion(int x, int y);
Vector3d get_arcball_vector(int x, int y);
void key_pressed(unsigned char key, int x, int y);

double rad2deg(double);
double deg2rad(double);

void create_lights();

void parseArguments(int argc, char* argv[]);

/******************************************************************************/
// Global variables
int xres = 500, yres = 500;

double e = 1.0, n = 1.0; // Shape exponents
double a = 1.0, b = 1.0, c = 1.0; // Scale factors
double theta = 0.0, r1 = 0.0, r2 = 0.0, r3 = 1.0; // Rotation
double xt = 0.0, yt = 0.0, zt = 0.0; // Translation
double xmin = -1.2, xmax = 1.2, ymin = -1.2, ymax = 1.2, zmin = -1.2, zmax = 1.2;
int Nu = 80, Nv = 40;

// Tolerance value for the Newton's Method update rule
double epsilon = 0.00001;

// Value for whether or not the parametric shape should be drawn.
bool parametricToggle = true;
// Value for whether the parametric shape should be drawn as a wireframe or a
// solid (transparent) object.
bool wireframe = false;

// Toggle for printing debug values. Set at program initiation.
bool printDebug = false;

// Matrix to store total rotations from the arcball
MatrixXd total_rotate = MatrixXd::Identity(4,4);

// Vector to store the objects in the game
vector<PhysicalObject> objects;
// Vector to store the objects that represent the world boundaries
// (ground, walls, etc.)
vector<Object> boundaries;

/*----- Arcball Globals -----*/
int last_mx = 0, last_my = 0, cur_mx = 0, cur_my = 0;
bool arcball_on = false;
double mouse_scale_x, mouse_scale_y;

const double step_size = 0.2;
const double x_view_step = 90.0, y_view_step = 90.0;
double x_view_angle = 0, y_view_angle = 0;

/*----- Camera globals -----*/
double cam_position[] = {0, 0, 2};

double cam_orientation_axis[] = {1, 1, 1};

double cam_orientation_angle = 0; // degrees

double near_param = 1, far_param = 10,
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

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    glEnable(GL_DEPTH_TEST);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
   
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(left_param, right_param,
              bottom_param, top_param,
              near_param, far_param);
    glMatrixMode(GL_MODELVIEW);
    
    create_parametric();
    create_ray_objects();
    create_lights();

    cam_position[2] = zmax + 1;
    
    init_lights();
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
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

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
    
    // Handle all of the position and velocity updates
    physics();
    
    // Draw the scene
    set_lights();
    draw_objects();
    
    glutSwapBuffers();
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

// Function to actually render objects to the screen
void draw_objects()
{
    // Set background to white
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
        
        for (int i = 0; i < halfFaces->size(); i++)
        {
            int ind1 = halfFaces->at(i)->a;
            int ind2 = halfFaces->at(i)->b;
            int ind3 = halfFaces->at(i)->c;
        
            glPushMatrix();
            
            // Draw the face
            glColor4f(1.0, 0, 0, 0.2);
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
    vector<Vector3d> netNewSpeeds = new vector<Vector3d>;
    for (int i = 0; i < objects.size(); i++) {
        Vector3d zeros(0,0,0);
        netNewSpeeds.push_back(zeros);
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
            if(objects.at(i).collidedWith(objects.at(j))) {
                float mI = objects.at(i).getMass();
                float mJ = objects.at(j).getMass();
                Vector3d vI = objects.at(i).getVelocity();
                Vector3d vJ = objects.at(j).getVelocity();
                
                Vector3d newVI = (mI - mJ) / (mI + mJ) * (vI - vJ) + vJ;
                Vector3d newVJ = (2 * mI) / (mI + mJ) * (vI - vJ) + vJ;
                
                netNewSpeeds.at(i) = netNewSpeeds.at(i) + newVI;
                netNewSpeeds.at(j) = netNewSpeeds.at(j) + newVJ;
            }
        }
    }
    
    // After all collisions have been checked, apply the "net new speed" to
    // every object (but only if the netNewSpeed vector is non-zero; if it /is/
    // zero then the object did not collide with anything).
    // This would also be a good place to add a slowdown factor
    for (int i = 0; i < objects.size(); i++) {
        if(!vecIsZero(netNewSpeeds.at(i))) {
            objects.at(i).setVelocity(netNewSpeeds.at(i));
        }
    }
    
    // Calculate the objects' new positions based on this speed, checking all
    // the way to make sure that the objects do not clip out of the world
    // (This loop could probably be merged with the one previous...)
    for (int i = 0; i < objects.size(); i++) {
        Vector3d pos = objects.at(i).getPosition();
        Vector3d vel = objects.at(i).getVelocity();
        objects.at(i).setPosition(pos + vel);
        // TODO: ensure the objects don't move past any of the world boundaries
         
    }
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
        last_mx = cur_mx = x;
        last_my = cur_my = y;
    }
    else
    {
        arcball_on = false;
    }
}
// Function to handle mouse movement
void onMotion(int x, int y)
{
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

        Vector3d axis = va.cross(vb);

        MatrixXd rotateMat = get_rotate_mat(axis(0), axis(1), axis(2), angle);
        total_rotate = total_rotate * rotateMat;

        glutPostRedisplay();

        last_mx = cur_mx;
        last_my = cur_my;
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
    if (key == 'q' || key == 'Q')
    {
        exit(0);
    }
    if (key == 't' || key == 'T')
    {
        parametricToggle = !parametricToggle;
        glutPostRedisplay();
    }
    if ((key == 'w' || key == 'W') && parametricToggle)
    {
        wireframe = !wireframe;
        glutPostRedisplay();
    }
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

// Function to parse the command line arguments
void parseArguments(int argc, char* argv[])
{
    int inInd = 1;
    
    // Command line triggers to respond to.
    const char* res = "-res"; // the next two values are the xres and yres of the display
    const char* exponents = "-e"; // the next two values are e and n
    const char* scaling = "-s"; // the next three values are the x,y,z scaling factors
    const char* rotation = "-r"; // the next four values are the angle of rotation and the rotation axis
    const char* translation = "-t"; // the next three values are the x,y,z translation components
    const char* display = "-dr"; // the next six values are the min/max display ranges
    const char* displayAll = "-as"; // if directly after display, the next value is 
                              //the absolute value to apply to all range values
    const char* parameters = "-p"; // the next two values are Nu and Nv
    const char* rayStart = "-b"; // the next three values are the x,y,z starting point for the ray
    const char* ray = "-a"; // the next three values are the vector direction of the ray
    const char* length = "-l"; // the next two values are the ray length and the intersection normal length
    const char* epsilonC = "-ep"; // the epsilon-close-to-zero value for the update rule
    const char* debugP = "-debug"; // tells the program to print debugging values

    // Temporary values to store the in parameters. These only get assigned to
    // the actual program values if no errors are encountered while parsing the
    // arguments.
    int txres = 500, tyres = 500;
    double te = 1.0, tn = 1.0; // Shape exponents
    double ta = 1.0, tb = 1.0, tc = 1.0; // Scale factors
    double ttheta = 0.0, tr1 = 0.0, tr2 = 0.0, tr3 = 1.0; // Rotation
    double txt = 0.0, tyt = 0.0, tzt = 0.0; // Translation
    double txmin = -1.2, txmax = 1.2, tymin = -1.2, tymax = 1.2, tzmin = -1.2, tzmax = 1.2;
    int tNu = 80, tNv = 40;
    Vector3d tbVec(5.0, 0.1, 0.1);
    Vector3d taVec(-1.0, 0.0, 0.0);
    double trayLength = 0.5, tnormalLength = 0.5;
    double tepsilon = .00001;
    bool tdebug = false;

    try
    {
        while (inInd < argc)
        {
            if (strcmp(argv[inInd], exponents) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: e");
                te = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: n");
                tn = atof(argv[inInd]);
            }
            else if (strcmp(argv[inInd], res) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: xres");
                txres = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: yres");
                tyres = atof(argv[inInd]);
            }
            else if (strcmp(argv[inInd], scaling) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: a");
                ta = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: b");
                tb = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: c");
                tc = atof(argv[inInd]);
            }
            else if (strcmp(argv[inInd], rotation) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: theta");
                ttheta = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: r1");
                tr1 = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: r2");
                tr2 = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: r3");
                tr3 = atof(argv[inInd]);
            }
            else if (strcmp(argv[inInd], translation) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: xt");
                txt = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: yt");
                tyt = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: zt");
                tzt = atof(argv[inInd]);
            }
            else if (strcmp(argv[inInd], display) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument");
                if (strcmp(argv[inInd], displayAll) == 0)
                {
                    inInd++;
                    if (inInd >= argc) throw out_of_range("Missing argument: val");
                    double val = atof(argv[inInd]);
                    txmin = tymin = tzmin = -val;
                    txmax = tymax = tzmax = val;
                }
                else
                {
                    inInd++;
                    if (inInd >= argc) throw out_of_range("Missing argument: xmin");
                    txmin = atof(argv[inInd]);
                    inInd++;
                    if (inInd >= argc) throw out_of_range("Missing argument: xmax");
                    txmax = atof(argv[inInd]);
                    inInd++;
                    if (inInd >= argc) throw out_of_range("Missing argument: ymin");
                    tymin = atof(argv[inInd]);
                    inInd++;
                    if (inInd >= argc) throw out_of_range("Missing argument: ymax");
                    tymax = atof(argv[inInd]);
                    inInd++;
                    if (inInd >= argc) throw out_of_range("Missing argument: zmin");
                    tzmin = atof(argv[inInd]);
                    inInd++;
                    if (inInd >= argc) throw out_of_range("Missing argument: zmax");
                    tzmax = atof(argv[inInd]);
                }
            }
            else if (strcmp(argv[inInd], epsilonC) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: epsilon");
                tepsilon = atof(argv[inInd]);
            }
            else if (strcmp(argv[inInd], debugP) == 0)
            {
                tdebug = true;
            }
            else if (strcmp(argv[inInd], parameters) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: Nu");
                tNu = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: Nv");
                tNv = atof(argv[inInd]);
            }
            else if (strcmp(argv[inInd], rayStart) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: bx");
                tbVec(0) = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: by");
                tbVec(1) = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: bz");
                tbVec(2) = atof(argv[inInd]);
            }
            else if (strcmp(argv[inInd], ray) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: ax");
                taVec(0) = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: ay");
                taVec(1) = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: az");
                taVec(2) = atof(argv[inInd]);
            }
            else if (strcmp(argv[inInd], length) == 0)
            {
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: rayLength");
                trayLength = atof(argv[inInd]);
                inInd++;
                if (inInd >= argc) throw out_of_range("Missing argument: normalLength");
                tnormalLength = atof(argv[inInd]);
            }

            inInd++;
        }

        xres = txres, yres = tyres;
        e = te, n = tn;
        a = ta, b = tb, c = tc;
        theta = ttheta * M_PI / 180.0, r1 = tr1, r2 = tr2, r3 = tr3;
        xt = txt, yt = tyt, zt = tzt;
        xmin = txmin, xmax = txmax, ymin = tymin, ymax = tymax, zmin = tzmin, zmax = tzmax;
        Nu = tNu, Nv = tNv;
        bVec = tbVec;
        aVec = taVec;
        rayLength = trayLength, normalLength = tnormalLength;
        epsilon = tepsilon;
        printDebug = tdebug;
    }
    catch (exception& e)
    {
        cout << "Error at input argument " << inInd << ":" << endl;
        cout << e.what() << endl;
        cout << "Program will execute with default values." << endl;
    }
}

int main(int argc, char* argv[])
{
    // extract the command line arguments
    parseArguments(argc, argv);

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
    glutKeyboardFunc(key_pressed);

    glutMainLoop();
}


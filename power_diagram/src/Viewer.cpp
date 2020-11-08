#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef MAC_OS
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif // MAC_OS

#include "ConvexHull.h"
#include "ConvexHullMesh.h"
#include "PowerDiagram.h"

#include "Viewer/Arcball.h" /* Arc Ball Interface */

using namespace DartLib;
using namespace ConvexHull;
using namespace PowerDiagram;

/* window width and height */
int g_win_width, g_win_height;
int g_button;
int g_startx, g_starty;

bool g_show_lift_points      = false;
bool g_show_points           = true;
bool g_show_lift_delaunay    = false;
bool g_show_delaunay         = true;
bool g_show_lift_dual_points = false;
bool g_show_dual_points      = true;
bool g_show_lift_dual_mesh   = false;
bool g_show_dual_mesh        = true;

bool g_flat_shading = true;

/* rotation quaternion and translation vector for the object */
CQrot g_obj_rot(0, 0, 1, 0);
CPoint g_obj_trans(0, 0, 0);

/* arcball object */
CArcball g_arcball;

/* global convexhull object */

CPowerDiagram g_pd;
CMesh & g_mesh = g_pd.mesh();


/*! setup the object, transform from the world to the object coordinate system */
void setupObject(void)
{
    double rot[16];

    glTranslated(g_obj_trans[0], g_obj_trans[1], g_obj_trans[2]);
    g_obj_rot.convert(rot);
    glMultMatrixd((GLdouble*) rot);
}

/*! the eye is always fixed at world z = +5 */
void setupEye(void)
{
    glLoadIdentity();
    gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);
}

/*! setup light */
void setupLight()
{
    GLfloat lightOnePosition[4] = {0, 0, 1, 0};
    GLfloat lightTwoPosition[4] = {0, 0, -1, 0};
    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT2, GL_POSITION, lightTwoPosition);
}

/*! draw the sites */
void drawPoints(bool isPlane = false)
{
    glDisable(GL_LIGHTING);
    glPointSize(2.0);
    glColor4f(1.0f, 1.0f, 0.0f, 0.6f);

    glBegin(GL_POINTS);
    for (const auto * p : g_pd.points())
    {
        const CPoint& pt = *p;
        if (isPlane)
            glVertex3d(pt[0], pt[1], -1e-3);
        else
            glVertex3d(pt[0], pt[1], pt[2]);
    }
    glEnd();
}

/*! draw mesh */
void drawMesh(bool isPlane = false)
{
    if (g_flat_shading) glEnable (GL_LIGHTING);
    else                glDisable(GL_LIGHTING);

    glLineWidth(1.0);
    glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
    //glColor3f(229.0f / 255, 162.0f / 255, 141.0f / 255);
    for (CMesh::MeshFaceIterator fiter(&g_mesh); !fiter.end(); ++fiter)
    {
        glBegin(GL_POLYGON);
        CMesh::CFace* pF = *fiter;
        for (CMesh::FaceVertexIterator fviter(pF); !fviter.end(); ++fviter)
        {
            CMesh::CVertex* pV = *fviter;
            CPoint& p = pV->point();
            CPoint& n = pF->normal();
            glNormal3d(n[0], n[1], n[2]);
            if (isPlane)
                glVertex3d(p[0], p[1], -1e-3);
            else
                glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
    }
}

void drawDualPoints(bool isPlane = false) 
{
    glDisable(GL_LIGHTING);
    glPointSize(2.0);
    glColor4f(1.0f, 0.0f, 0.0f, 0.6f);

    glBegin(GL_POINTS);
    for (CMesh::FaceIterator fiter(&g_mesh); !fiter.end(); ++fiter)
    {
        CMesh::CFace* pF = *fiter;
        CPoint& p = pF->dual_point();

        if (isPlane)
            glVertex3d(p[0], p[1], 0.0);
        else
            glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();
}

void drawBoundaryDualEdge(bool isPlane = false) 
{
    CMesh::CBoundary b(&g_mesh);
    auto & loops = b.loops();
    if(loops.size() != 1) return;

    auto & halfedges = loops[0]->halfedges();
    for (auto pD : halfedges)
    {
        CPoint& p1 = g_mesh.dart_source(pD)->point();
        CPoint& p2 = g_mesh.dart_target(pD)->point();
        CPoint& p3 = g_mesh.dart_face(pD)->dual_point();
        CPoint  p4;        
        p4 = (p2 - p1) ^ CPoint(0, 0, 1);
        p4 = p4 * 10 + p3;
        //insert your code here,

        glColor4f(1.0f, 1.0f, 0.0f, 0.6f);
        glBegin(GL_LINES);
        if (isPlane)
        {
            glVertex3d(p3[0], p3[1], 0.0);
            glVertex3d(p4[0], p4[1], 0.0);
        }
        else
        {
            glVertex3d(p3[0], p3[1], p3[2]);
            glVertex3d(p4[0], p4[1], p4[2]);
        }
        glEnd();
    }
}

void drawDualMesh(bool isPlane = false)
{
    if (g_flat_shading) glEnable (GL_LIGHTING);
    else                glDisable(GL_LIGHTING);

    glLineWidth(1.0);
    glColor4f(1.0f, 1.0f, 0.0f, 0.6f);
    for (CMesh::MeshVertexIterator viter(&g_mesh); !viter.end(); ++viter)
    {
        CMesh::CVertex* pV = *viter;
        CPoint n(0, 0, 0);
        for (CMesh::VertexFaceIterator vfiter(pV); !vfiter.end(); ++vfiter)
        {
            CMesh::CFace* pF = *vfiter;
            n += pF->normal();
        }

        glBegin(GL_POLYGON);
        for (CMesh::VertexFaceIterator vfiter(pV); !vfiter.end(); ++vfiter)
        {
            CMesh::CFace* pF = *vfiter;
            CPoint& p = pF->dual_point();
            glNormal3d(n[0], n[1], n[2]);
            if (isPlane)
                glVertex3d(p[0], p[1], 0.0);
            else
                glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
    }

    drawBoundaryDualEdge(isPlane);
}

/*! display call back function
 */
void display()
{
    /* clear frame buffer */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    setupLight();
    /* transform from the eye coordinate system to the world system */
    setupEye();
    glPushMatrix();
    /* transform from the world to the ojbect coordinate system */
    setupObject();

    if (g_show_lift_points)      drawPoints(false);
    if (g_show_points)           drawPoints(true);
    
    if (g_show_lift_dual_points) drawDualPoints(false);
    if (g_show_dual_points)      drawDualPoints(true);

    if (g_show_lift_delaunay)    drawMesh(false);
    if (g_show_delaunay)         drawMesh(true);

    if (g_show_lift_dual_mesh)   drawDualMesh(false);
    if (g_show_dual_mesh)        drawDualMesh(true);
    

    glPopMatrix();
    glutSwapBuffers();
}

/*! Called when a "resize" event is received by the window. */
void reshape(int w, int h)
{
    float ar;

    g_win_width = w;
    g_win_height = h;

    ar = (float) (w) / h;
    glViewport(0, 0, w, h); /* Set Viewport */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(40.0, /* field of view in degrees */
                   ar,   /* aspect ratio */
                   0.1,  /* Z near */
                   100.0 /* Z far */);

    glMatrixMode(GL_MODELVIEW);

    glutPostRedisplay();
}

/*! helper function to remind the user about commands, hot keys */
void help()
{
    printf("Usage: PowerDiagram [num_of_sites]\n");
    printf("   ex: PowerDiagram 600\n\n");

    printf("d  -  Compute the delaunay triangulation\n");
    printf("v  -  Compute the voronoi diagram\n");
    
    printf("\n");

    printf("1  -  Show or hide the lifting points\n");
    printf("2  -  Show or hide the 2d points\n");

    printf("3  -  Show or hide the 3d convex hull (downward)\n");
    printf("4  -  Show or hide the 2d delaunay triangulation\n");

    printf("5  -  Show or hide the 3d dual points\n");
    printf("6  -  Show or hide the 2d dual points\n");

    printf("7  -  Show or hide the upper envelope\n");
    printf("8  -  Show or hide the 2d dual mesh\n");

    printf("\n");

    printf("w  -  Wireframe Display\n");
    printf("f  -  Flat Shading \n");

    printf("\n");
    
    printf("?  -  Help Information\n");
    printf("esc - Quit\n\n");
}

/*! Keyboard call back function */
void keyBoard(unsigned char key, int x, int y)
{
    static int site_index = 3; // the first three point has been inserted.
    CPoint* site = NULL;

    switch (key)
    {
        case '1':
            // Show or hide the sites
            g_show_lift_points = !g_show_lift_points;            
            break;
        case '2':
            // Show or hide the convex hull
            g_show_points = !g_show_points;
            break;
        case '3':
            // Show or hide the sites
            g_show_lift_delaunay = !g_show_lift_delaunay;
            break;
        case '4':
            // Show or hide the sites
            g_show_delaunay = !g_show_delaunay;
            break;
        case '5':
            // Show or hide the sites
            g_show_lift_dual_points = !g_show_lift_dual_points;
            break;
        case '6':
            // Show or hide the sites
            g_show_dual_points = !g_show_dual_points;
            break;
        case '7':
            // Show or hide the sites
            g_show_lift_dual_mesh = !g_show_lift_dual_mesh;
            break;
        case '8':
            // Show or hide the sites
            g_show_dual_mesh = !g_show_dual_mesh;
            break;
        case 'd':
            // Compute the delaunay triangulation
            g_pd.calc_delaunay();
            break;
        case 'v':
            // Compute the voronoi diagram
            g_pd.calc_voronoi();
            break;
        case 'f':
            // Flat Shading
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            g_flat_shading = true;
            break;
        case 'w':
            // Wireframe mode
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            g_flat_shading = false;
            break;
        case '?':
            help();
            break;
        case 27:
            exit(0);
            break;
    }
    glutPostRedisplay();
}

/*! setup GL states */
void setupGLstate()
{
    GLfloat lightOneColor[] = {1, 1, 1, 1.0};
    GLfloat globalAmb[] = {.1f, .1f, .1f, 1.f};
    GLfloat lightOnePosition[] = {.0, 0.0, 1.0, 1.0};
    GLfloat lightTwoPosition[] = {.0, 0.0, -1.0, 1.0};

    //glEnable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.35f, 0.53f, 0.70f, 0.0f);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHT2);
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    // Enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
    glLightfv(GL_LIGHT2, GL_DIFFUSE, lightOneColor);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmb);
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

    glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
    glLightfv(GL_LIGHT2, GL_POSITION, lightTwoPosition);

    const GLfloat specular[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 64.0f);

    GLfloat mat_ambient[] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat mat_diffuse[] = {0.01f, 0.01f, 0.01f, 1.0f};
    GLfloat mat_specular[] = {0.5f, 0.5f, 0.5f, 1.0f};
    GLfloat mat_shininess[] = {32};

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
}

/*! mouse click call back function */
void mouseClick(int button, int state, int x, int y)
{
    /* set up an arcball around the Eye's center
    switch y coordinates to right handed system  */

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        g_button = GLUT_LEFT_BUTTON;
        g_arcball = CArcball(g_win_width, g_win_height, x - g_win_width / 2, g_win_height - y - g_win_height / 2);
    }

    if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN)
    {
        g_startx = x;
        g_starty = y;
        g_button = GLUT_MIDDLE_BUTTON;
    }

    if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
    {
        g_startx = x;
        g_starty = y;
        g_button = GLUT_RIGHT_BUTTON;
    }
    return;
}

/*! mouse motion call back function */
void mouseMove(int x, int y)
{
    CPoint trans;
    CQrot rot;

    /* rotation, call g_arcball */
    if (g_button == GLUT_LEFT_BUTTON)
    {
        rot = g_arcball.update(x - g_win_width / 2, g_win_height - y - g_win_height / 2);
        g_obj_rot = rot * g_obj_rot;
        glutPostRedisplay();
    }

    /*xy translation */
    if (g_button == GLUT_MIDDLE_BUTTON)
    {
        double scale = 10. / g_win_height;
        trans = CPoint(scale * (x - g_startx), scale * (g_starty - y), 0);
        g_startx = x;
        g_starty = y;
        g_obj_trans = g_obj_trans + trans;
        glutPostRedisplay();
    }

    /* zoom in and out */
    if (g_button == GLUT_RIGHT_BUTTON)
    {
        double scale = 10. / g_win_height;
        trans = CPoint(0, 0, scale * (g_starty - y));
        g_startx = x;
        g_starty = y;
        g_obj_trans = g_obj_trans + trans;
        glutPostRedisplay();
    }
}

void initOpenGL(int argc, char* argv[])
{
    /* glut stuff */
    glutInit(&argc, argv); /* Initialize GLUT */
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(600, 600);
    glutCreateWindow("2D Power Diagram"); /* Create window with given title */
    glViewport(0, 0, 600, 600);

    glutDisplayFunc(display); /* Set-up callback functions */
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseClick);
    glutMotionFunc(mouseMove);
    glutKeyboardFunc(keyBoard);
    setupGLstate();

    glutMainLoop(); /* Start GLUT event-processing loop */
}

/*! main function for viewer
 */
int main(int argc, char* argv[])
{
    int num_sites = 100;

    if (argc >= 2)
        num_sites = atoi(argv[1]);

    g_pd.init(num_sites);

    help();

    initOpenGL(argc, argv);
    return EXIT_SUCCESS;
}

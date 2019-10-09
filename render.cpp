/****************************************************************************
  Based on GLUI example2 :A simple GLUT program using the GLUI User Interface Library

                        :9/9/98 Paul Rademacher (rademach@cs.unc.edu)

****************************************************************************/

#include <string.h>
#include <glui.h>
#include <GLUT/glut.h>
#include "Extrusion.h"
#include "main.h"

static float xyAspect;
static int   lastX, lastY;
static float rotationX = 0.0, rotationY = 0.0;


// Using a std::string as a live variable is safe.
static std::string text = "KEYS: [s] [Delete] [SHIFT+Delete]";
static double eyeX = 0, eyeY = 0, eyeZ = 0;
static double eyeDelta = 0.5;

using namespace std;
list<list<PlanData::Vertex_handle> >output;
extern void draw_point(int, int);
extern float pointSize;
MyPolyhedron shell;
void render();

/***************************************** for debuging  ****************/
void display_test_sw3()
{
    glClearColor( .9f, .9f, .9f, 1.0f );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, stateSw3.w, 0, stateSw3.h);
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    //polys = extrusion.triangulate(polys);
    float oldPointSize = pointSize;
    for (list<list<PlanData::Vertex_handle> >::iterator it = output.begin(); it !=output.end(); ++it)
    {
        glColor3f(1,1,1);
        glBegin(GL_LINE_STRIP);
        list<PlanData::Vertex_handle> ply2 = *it;

        for (list<PlanData::Vertex_handle>::iterator it1 = ply2.begin(); it1 != ply2.end(); ++it1)
        {
            glVertex2f((*it1)->point.x(), (*it1)->point.y());
            //printf("%f %f\n", it1->x(), it1->y());
        }
        glEnd();
        pointSize = 1;
        for (list<PlanData::Vertex_handle>::iterator it1 = ply2.begin(); it1 != ply2.end(); ++it1)
        {
            if (pointSize < 6) {
                pointSize = pointSize + 1;
            }
            draw_point((*it1)->point.x(), (*it1)->point.y());

        }
        pointSize = oldPointSize;
    }
    //glutSwapBuffers();

    render();
}

/***************************************** myGlutSpecialFunc() **********/


void myGlutSpecialFunc(int key, int x, int y)
{
    switch(key)
    {
    case GLUT_KEY_UP:
        eyeZ -= eyeDelta;
        break;
    case GLUT_KEY_DOWN:
        eyeZ += eyeDelta;
        break;
    case GLUT_KEY_RIGHT:
        extrusion.build_polyhedron(stateSw1.edit, stateSw2.edit, shell);
        break;
    }
    glutPostRedisplay();

}

void preview()
{

    int oldWindow = glutGetWindow();
    glutSetWindow(stateSw3.window);
    extrusion.build_polyhedron(stateSw1.edit, stateSw2.edit, shell);
    glutPostRedisplay();
    glutSetWindow(oldWindow);
}

/***************************************** myGlutMouse() **********/

void myGlutMouse(int button, int button_state, int x, int y )
{
  if ( button == GLUT_LEFT_BUTTON && button_state == GLUT_DOWN )
  {
    lastX = x;
    lastY = y;

    glutPostRedisplay();
  }

}


/***************************************** myGlutMotion() **********/

void myGlutMotion(int x, int y )
{
  rotationX += (float) (y - lastY);
  rotationY += (float) (x - lastX);

  lastX = x;
  lastY = y;

  glutPostRedisplay();
}

/**************************************** myGlutReshape() *************/
static void update_max(float &max, float a) {
    if(a > max) {
        max = a;
    }
}
static void update_min(float &min, float a) {
    if(a < min) {
        min = a;
    }
}

void myGlutReshape( int x, int y )
{
  xyAspect = (float)x / (float)y;
  glViewport( 0, 0, x, y );

  glutPostRedisplay();
}



//assume input coords are all positive
void render() {
    //MyPolyhedron phdrn = extrusion.build_polyhedron(stateSw1.edit, stateSw2.edit);

    float max = -1;


    for (MyPolyhedron::iterator it = shell.begin(); it != shell.end(); ++it)
    {
        Triangle_3 tri = *it;

        for (int i=0; i<3; ++i)
        {
            Point_3 p = tri[i];

            update_max(max, p.x());
            update_max(max, p.y());
            update_max(max, p.z());
        }
    }

    float minX=2, maxX = -1, minY = 2, maxY = -1, minZ = 2, maxZ = -1;
    for (MyPolyhedron::iterator it = shell.begin(); it != shell.end(); ++it)
    {
        Triangle_3 tri = *it;

        for (int i=0; i<3; ++i)
        {
            Point_3 p = tri[i];
            update_max(maxX, p.x()/max);
            update_max(maxY, p.y()/max);
            update_max(maxZ, p.z()/max);
            update_min(minX, p.x()/max);
            update_min(minY, p.y()/max);
            update_min(minZ, p.z()/max);
        }
    }

    float centerX = (minX + maxX)/2, centerY = (minY + maxY)/2, centerZ = (minZ +maxZ)/2;
    //printf("center=%f, %f, %f\n", centerX, centerY, centerZ);

    eyeX = centerX;
    eyeY = centerY;

    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective(65.0, stateSw3.w/(float)stateSw3.h, 1, 1000);
    gluLookAt(eyeX, eyeY, eyeZ, eyeX, eyeY, eyeZ-1, eyeX, 1, eyeZ);

    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    glTranslatef( 0.0f, 0.0f, -2.6f );
    glTranslatef(centerX, centerY, centerZ);
    glRotatef( rotationY, 0.0, 1.0, 0.0 );
    glRotatef( rotationX, 1.0, 0.0, 0.0 );
    glTranslatef(-centerX, -centerY, -centerZ);

    for (MyPolyhedron::iterator it = shell.begin(); it != shell.end(); ++it)
    {
        glColor3f(1.0, 1.0, 1.0);
        glBegin(GL_TRIANGLES);

        if(CGAL::collinear((*it)[0], (*it)[1], (*it)[2]))
            continue;

        Vector_3 normalV = CGAL::normal((*it)[0], (*it)[1], (*it)[2]);
        //printf("normalV.() = %lf, %lf, %lf\n",normalV.x(), normalV.y(), normalV.z() );
        glNormal3f(normalV.x(), normalV.y(), normalV.z());

        for (int i=0; i<3; ++i)
        {
            glVertex3f((*it)[i].x()/max, (*it)[i].y()/max, (*it)[i].z()/max);
        }
        glEnd();
    }

    glDisable( GL_LIGHTING );
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluOrtho2D( 0.0, 100.0, 0.0, 100.0  );
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    glColor3ub( 0, 0, 0 );
    glRasterPos2i( 10, 10 );

    /*** Render the live character array 'text' ***/
    for (unsigned int i=0; i<text.length(); ++i)
      glutBitmapCharacter( GLUT_BITMAP_HELVETICA_18, text[i] );

    glEnable( GL_LIGHTING );

    glutSwapBuffers();
}

/***************************************** myGlutDisplay() *****************/

void myGlutDisplay( void )
{
  glClearColor( .9f, .9f, .9f, 1.0f );
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );



  render();
  //glutSolidSphere( .6, 8, 8 );
  //display_test_sw3();

}


/**************************************** main() ********************/

void setup()
{
  glutDisplayFunc( myGlutDisplay );
  glutReshapeFunc( myGlutReshape );
  glutMotionFunc( myGlutMotion );
  glutMouseFunc( myGlutMouse );
  glutSpecialFunc(myGlutSpecialFunc);

  /****************************************/
  /*       Set up OpenGL lights           */
  /****************************************/

  GLfloat light0_ambient[] =  {0.1f, 0.1f, 0.3f, 1.0f};
  GLfloat light0_diffuse[] =  {.6f, .6f, 1.0f, 1.0f};
  GLfloat light0_position[] = {1.0f, 1.0f, 10.0f, 0.0f};

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
  glEnable(GL_NORMALIZE);
  /****************************************/
  /*          Enable z-buferring          */
  /****************************************/

  glEnable(GL_DEPTH_TEST);



}


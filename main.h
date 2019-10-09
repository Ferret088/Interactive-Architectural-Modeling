#ifndef MAIN_H
#define MAIN_H
#include <stdio.h>
#include <stdlib.h>
#include <GLUT/glut.h>

#include "point.h"
#include "edge.h"
#include "edit.h"
#include "extrusion.h"

#define MODE_LINES 1
#define MODE_SELECT 2
#define MODE_STRIPS 3
#define MODE_POLYGONS 4
#define MAGNET_DIST 50

//Stores the state of a window
struct State{

    //Window width and height
    int w;
    int h;

    //Window handle
    int window;

    //When draw a line, we need to know if the user is adding the first point or the second point of the line
    bool pointPending;
    Point lastPoint;

    //could be MODE_LINES, MODE_SELECT, etc.
    int mode;

    //Used to treat points close enough as same point
    bool magneted;
    Point magnet;

    State():lastPoint(0,0), pointPending(false), mode(MODE_LINES), magneted(false),magnet(Point(0,0)){}

    //Corresponding data model, similar to controller+model in MVC
    Editor edit;

};


extern State stateSw1, stateSw2, stateSw3;
extern int mainWindow, subWindow1, subWindow2, subWindow3;

void setup();
void preview();
#endif // MAIN_H

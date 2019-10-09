/*
 *This program reads in 2D data and use it to generate 3D building.
 */
#include "main.h"

// subWindow1 is plan edit, subWindow2 is profile edit, subWindow3 is Output
int mainWindow, subWindow1, subWindow2, subWindow3;
State stateSw1, stateSw2, stateSw3;

static bool leftButton = false;
//border between subwindows
static const int border = 6;

static float lineWidth = 2.0;
float pointSize = 3.0;
static const int bgColor[3] = {143, 199, 135};
static const int pointColor[3] = {122, 148, 111};
static const int lineColor[3] = {213, 205, 182};
//const int selectColor[3] = {133, 52, 49};
static const float white[3] = {1, 1, 1};
static const int selectColor[3] = {0, 0, 0};



// -----------------------------------
//             Display
// -----------------------------------

//Set Projection to Orthogonal
void set_ortho(const State& state)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, state.w, 0, state.h);
    glMatrixMode(GL_MODELVIEW);
}


// Display func for main window
void display_main()
{
    glutSetWindow(mainWindow);
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();
}

// Draw a Point, big enough to see :)
void draw_point(int x, int y)
{
    glColor3f(pointColor[0]/255.0, pointColor[1]/255.0, pointColor[2]/255.0);
    glBegin(GL_POLYGON);
    glVertex2f(x-pointSize, y-pointSize);
    glVertex2f(x-pointSize, y+pointSize);
    glVertex2f(x+pointSize, y+pointSize);
    glVertex2f(x+pointSize, y-pointSize);
    glEnd();
}

//Draw an edge, which is two points and a line
void draw_edge(const Edge &edge)
{
    //glColor3f(1, 1, 1);
    glLineWidth(lineWidth);
    draw_point(edge.p1.x, edge.p1.y);
    draw_point(edge.p2.x, edge.p2.y);
    glColor3f(lineColor[0]/255.0, lineColor[1]/255.0, lineColor[2]/255.0);
    glBegin(GL_LINES);
    glVertex2f(edge.p1.x, edge.p1.y);
    glVertex2f(edge.p2.x, edge.p2.y);
    glEnd();
}

//A slightly different edge, for better UI
void draw_edge_without_endpoint(const Edge &edge)
{
    glLineWidth(lineWidth);
    draw_point(edge.p1.x, edge.p1.y);
    glBegin(GL_LINES);
    glVertex2f(edge.p1.x, edge.p1.y);
    glVertex2f(edge.p2.x, edge.p2.y);
    glEnd();
}

//Draw the magnet
void draw_magnet(Point p)
{
    //printf("drawMagnet: %f %f:", p.x, p.y);
    float x = p.x, y = p.y;
    glLineWidth(1.0f);
    glColor3f(selectColor[0]/255.0, selectColor[1]/255.0, selectColor[2]/255.0);
    glBegin(GL_LINE_LOOP);
    glVertex2f(x-pointSize-2, y-pointSize-2);
    glVertex2f(x-pointSize-2, y+pointSize+2);
    glVertex2f(x+pointSize+2, y+pointSize+2);
    glVertex2f(x+pointSize+2, y-pointSize-2);
    glEnd();
}

//Draw the scene using the editor data
void draw_editor(const Editor &edit, const State& windowState)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    std::set<Edge> edges = edit.get_edges();
    for (std::set<Edge>::iterator it = edges.begin(); it != edges.end(); ++it)
    {
        draw_edge(*it);
    }
    if (windowState.pointPending)
    {
        draw_point(windowState.lastPoint.x, windowState.lastPoint.y);
    }
    if (windowState.magneted)
    {
        draw_magnet(windowState.magnet);
    }
}

void display_editor(const State& windowState)
{
    draw_editor(windowState.edit, windowState);
    glutSwapBuffers();
}

// Display func for sub window 1
void display_sw1()
{
    display_editor(stateSw1);
   // preview();
}

// Display func for sub window 2
void display_sw2()
{
    display_editor(stateSw2);
   // preview();
}

// -----------------------------------
//             KEYBOARD
// -----------------------------------
void display_test_sw1();
void display_test_sw2();

void process_normal_keys(unsigned char key, int xx, int yy, State & windowState)
{
    if (key == 27)
    {
        //Esc key
        glutDestroyWindow(subWindow3);
        glutDestroyWindow(subWindow2);
        glutDestroyWindow(subWindow1);
        glutDestroyWindow(mainWindow);
        exit(0);
    }else if(key == 127)
    {
        //Delete key
        if(!windowState.magneted)
        {
            if(glutGetModifiers() == GLUT_ACTIVE_SHIFT)
            {
                windowState.edit = Editor();

            }
        }
        else
        {
            windowState.edit.remove(windowState.magnet);
        }
        windowState.pointPending = false;
        windowState.magneted = false;
        glutPostRedisplay();
    }else if(key == 's')
    {
        //printf("s\n");
        if(windowState.window == subWindow2)
        {
            display_test_sw2();
        }else if(windowState.window == subWindow1)
        {
            display_test_sw1();
        }
        windowState.pointPending = false;
        windowState.magneted = false;
    }
}

void process_normal_keys_sw1(unsigned char key, int xx, int yy)
{
    process_normal_keys(key, xx, yy, stateSw1);
}
void process_normal_keys_sw2(unsigned char key, int xx, int yy)
{
    process_normal_keys(key, xx, yy, stateSw2);
}
void process_normal_keys_sw3(unsigned char key, int xx, int yy)
{
    process_normal_keys(key, xx, yy, stateSw3);
}

void make_line_straight_if_shift_active(int &x, int &y, State& windowState)
{
    //draw straight lines
    //if(glutGetModifiers() == GLUT_ACTIVE_SHIFT)
    {
        int deltaX = x - windowState.lastPoint.x;
        int deltaY = y - windowState.lastPoint.y;
        if(abs(deltaX) < 3)
        {
                x = windowState.lastPoint.x;

        }else if(abs(deltaY) < 3)
        {
            if(windowState.window == stateSw1.window)

                y = windowState.lastPoint.y;
        }
    }

}
//Check if there is already a point close enough
bool do_magnet(int &x, int &y, State &windowState)
{
    y = windowState.h - y;
    Magnet m = windowState.edit.magnet(Point(x, y));
    if (m.dist != -1 && m.dist < MAGNET_DIST)
    {
        x = m.p.x;
        y = m.p.y;
        windowState.magneted = true;
        windowState.magnet = m.p;
        return true;
        //glutWarpPointer(x, y);
    }
    windowState.magneted = false;
    make_line_straight_if_shift_active(x, y, windowState);
    return false;
}



// -----------------------------------
//             MOUSE
// -----------------------------------

void mouse_move_edit(int x, int y, State &windowState)
{
//    printf("move: %d %d\n", x, y);
    do_magnet(x, y, windowState);
    if (leftButton)
    {
        if (windowState.mode == MODE_LINES)
        {

            draw_editor(windowState.edit, windowState);
            if(windowState.pointPending)
            {
                //make_line_straight_if_shift_active(x, y, windowState);
                draw_edge_without_endpoint(Edge(windowState.lastPoint, Point(x,y)));
            }else
            {
                draw_point(x, y);
            }
            glutSwapBuffers();
        }
    }
}

void mouse_move_Sw1(int x, int y)
{
    mouse_move_edit(x, y, stateSw1);
}

void mouse_move_sw2(int x, int y)
{
    mouse_move_edit(x, y, stateSw2);
}


void mouse_passive_edit(int x, int y, State &windowState)
{
    do_magnet(x, y, windowState);
    if (windowState.mode == MODE_LINES)
    {
        draw_editor(windowState.edit, windowState);
        if (windowState.pointPending)
        {
            //make_line_straight_if_shift_active(x, y, windowState);

            draw_edge_without_endpoint(Edge(windowState.lastPoint, Point(x,y)));
        }
        glutSwapBuffers();
    }
}

void mouse_passive_sw1(int x, int y)
{
    mouse_passive_edit(x, y, stateSw1);
}

void mouse_passive_sw2(int x, int y)
{
    mouse_passive_edit(x, y, stateSw2);
}

void mouse_button_edit(int button, int state, int x, int y, State &windowState)
{
    do_magnet(x, y, windowState);
    if (button == GLUT_LEFT_BUTTON)
    {
        leftButton = true;
        if (state == GLUT_UP) {\
            // when the button is released
            if (windowState.mode == MODE_LINES)
            {
                if(windowState.pointPending)
                {
                    windowState.edit.add_edge(windowState.lastPoint, Point(x, y));
                    windowState.pointPending = false;
                } else
                {
                    windowState.pointPending = true;
                }
                windowState.lastPoint = Point(x, y);
                glutPostRedisplay();
            }
        }
        else
        {// state = GLUT_DOWN
            if (windowState.mode == MODE_LINES)
            {
                draw_editor(windowState.edit, windowState);
                if(windowState.pointPending)
                {
                    draw_edge(Edge(windowState.lastPoint, Point(x,y)));
                }else
                {
                    draw_point(x, y);
                }
                glutSwapBuffers();
            }
        }

    } else
    {
        leftButton = false;
    }
}
void mouse_button_sw1(int button, int state, int x, int y)
{
    mouse_button_edit(button, state, x, y, stateSw1);
}
void mouse_button_sw2(int button, int state, int x, int y)
{
    mouse_button_edit(button, state, x, y, stateSw2);
}


//just for debugging
void display_test_sw2()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();


    std::vector<std::vector<Point> > polys ;//= stateSw2.edit.get_profiles();
    polys.push_back(stateSw2.edit.get_profile(0));
    if(stateSw2.edit.get_overhanging_profiles(0).size() >= 2)
    {
    polys.push_back(stateSw2.edit.get_overhanging_profiles(0)[0]);
    polys.push_back(stateSw2.edit.get_overhanging_profiles(0)[1]);
    }
    //polys = extrusion.test(polys);
    float oldPointSize = pointSize;
    for (std::vector<std::vector<Point> >::iterator it = polys.begin(); it !=polys.end(); ++it)
    {
        glColor3fv(white);
        glBegin(GL_LINE_STRIP);
        std::vector<Point> ply2 = *it;

        for (std::vector<Point>::iterator it1 = ply2.begin(); it1 != ply2.end(); ++it1)
        {
            glVertex2f(it1->x, it1->y);
            //printf("%f %f\n", it1->x(), it1->y());
        }
        glEnd();
        pointSize = 1;
        for (std::vector<Point>::iterator it1 = ply2.begin(); it1 != ply2.end(); ++it1)
        {
            if (pointSize < 6)
            {
                pointSize = pointSize + 1;
            }
            draw_point(it1->x, it1->y);

        }
    }
    pointSize = oldPointSize;
    glutSwapBuffers();
}

void display_test_sw1()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    std::vector<Polygon_2> polys = stateSw1.edit.get_polygons();
    //polys = extrusion.triangulate(polys);
    float oldPointSize = pointSize;
    for (std::vector<Polygon_2 >::iterator it = polys.begin(); it !=polys.end(); ++it)
    {
        glColor3fv(white);
        glBegin(GL_LINE_LOOP);
        Polygon_2 ply2 = *it;

        for (Polygon_2::Vertex_iterator it1 = ply2.vertices_begin(); it1 != ply2.vertices_end(); ++it1)
        {
            glVertex2f(it1->x(), it1->y());
            //printf("%f %f\n", it1->x(), it1->y());
        }
        glEnd();
        pointSize = 1;
        for (Polygon_2::Vertex_iterator it1 = ply2.vertices_begin(); it1 != ply2.vertices_end(); ++it1)
        {
            if (pointSize < 6) {
                pointSize = pointSize + 1;
            }
            draw_point(it1->x(), it1->y());

        }
    }
    pointSize = oldPointSize;
    glutSwapBuffers();
}

//reshape for mainwindow
void reshape(int w, int h)
{
    if(w == 0) w = 1;
    if(h == 0) h = 1;

    //for sub
    glutSetWindow(stateSw1.window);
    stateSw1.w = w/3 - border;
    stateSw1.h = h - 2*border;
    glutPositionWindow(border, border);
    glutReshapeWindow(stateSw1.w, stateSw1.h);
    set_ortho(stateSw1);

    glutSetWindow(stateSw2.window);
    stateSw2.w = w/3 - border;
    stateSw2.h = h - 2*border;
    glutPositionWindow(stateSw1.w + 2*border, border);
    glutReshapeWindow(stateSw2.w, stateSw2.h);
    set_ortho(stateSw2);

    glutSetWindow(stateSw3.window);
    stateSw3.w = w - stateSw1.w - stateSw2.w - 4*border;
    stateSw3.h = h - 2*border;
    glutPositionWindow(stateSw1.w + stateSw2.w + 3*border, border);
    glutReshapeWindow(stateSw3.w, stateSw3.h);
}

int main(int argc, char **argv)
{
    //disable printf buffering
    setbuf(stdout, NULL);

    // init GLUT and create main window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowPosition(10,10);
    glutInitWindowSize(1200, 600);
    mainWindow = glutCreateWindow(argv[0]);


    // callbacks for main window
    glutDisplayFunc(display_main);
    //glutReshapeFunc(reshapeMain);
    //glutKeyboardFunc(process_normal_keys);
    glClearColor(1.0,1.0,1.0,1.0);
    glutReshapeFunc(reshape);

    // sub windows
    stateSw1.w = 400 - border;
    stateSw1.h = 600 - 2*border;
    stateSw1.window = subWindow1 = glutCreateSubWindow(mainWindow, border,border, stateSw1.w, stateSw1.h);
    glutDisplayFunc(display_sw1);
    //glutDisplayFunc(display_test);
    glutKeyboardFunc(process_normal_keys_sw1);
    glutMouseFunc(mouse_button_sw1);
    glutMotionFunc(mouse_move_Sw1);
    glutPassiveMotionFunc(mouse_passive_sw1);
    glClearColor (bgColor[0]/255.0, bgColor[1]/255.0, bgColor[2]/255.0, 1.0);
    set_ortho(stateSw1);

    stateSw2.w = 400 - border;
    stateSw2.h = 600 - 2*border;

    stateSw2.window = subWindow2 = glutCreateSubWindow(mainWindow, stateSw1.w + 2* border, border, stateSw2.w, stateSw2.h);
    glutDisplayFunc(display_sw2);
    glutKeyboardFunc(process_normal_keys_sw2);
    glutMouseFunc(mouse_button_sw2);
    glutMotionFunc(mouse_move_sw2);
    glutPassiveMotionFunc(mouse_passive_sw2);
    glClearColor (bgColor[0]/255.0, bgColor[1]/255.0, bgColor[2]/255.0, 1.0);
    set_ortho(stateSw2);


    stateSw3.w = 1200 - stateSw1.w - stateSw2.w - 4*border;
    stateSw3.h = 600 - 2*border;

    stateSw3.window = subWindow3 = glutCreateSubWindow(mainWindow, stateSw1.w + stateSw2.w + 3*border, border, stateSw3.w, stateSw3.h);

    /*
    glClearColor (bgColor[0]/255.0, bgColor[1]/255.0, bgColor[2]/255.0, 1.0);
    setOrtho(stateSw3);
    glutKeyboardFunc(processNormalKeys);
    glutDisplayFunc(displayTest1);
    glutMouseFunc(mouseTest);

    */
    //Call function in render, Seperate the code for clarity
    setup();
    glutKeyboardFunc(process_normal_keys_sw3);
    // enter GLUT event processing cycle
    glutMainLoop();

    return 1;
}


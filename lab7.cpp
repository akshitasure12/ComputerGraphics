// - Left click: add vertex (grid snapped)
// - Right click: close polygon (if >= 3 vertices)
// - 1: Scanline fill
// - 2: Flood fill (4-connected) -> then left click inside polygon to pick seed
// - 3: Flood fill (8-connected) -> then left click inside polygon to pick seed
// - 4: Boundary fill            -> then left click inside polygon to pick seed
// - c: Clear polygon and fills
// - ESC: Exit

#include <GLUT/glut.h>
#include <vector>
#include <set>
#include <queue>
#include <utility>
#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

using namespace std;

int GRID_SIZE = 20;
int SCREEN_WIDTH = 1200;
int SCREEN_HEIGHT = 800;

struct Point { int x, y; };
vector<Point> polygonPts;
bool polygonClosed = false;

set<pair<int,int>> filledCells;
set<pair<int,int>> boundaryCells;

enum FillMode { NONE, SCANLINE, FLOOD4_WAIT, FLOOD8_WAIT, BOUNDARY_WAIT, FLOOD4, FLOOD8, BOUNDARY };
FillMode currentMode = NONE;

int DELAY_MS = 0;

inline void sleep_ms(int ms) {
    if (ms > 0) this_thread::sleep_for(chrono::milliseconds(ms));
}

void screenToGrid(int sx, int sy, int &gx, int &gy) {
    gx = sx / GRID_SIZE;
    gy = sy / GRID_SIZE;
}

void gridToScreen(int gx, int gy, float &sx, float &sy) {
    sx = gx * GRID_SIZE;
    sy = gy * GRID_SIZE;
}

void drawQuadAtGrid(int gx, int gy, float r, float g, float b) {
    float x0 = gx * GRID_SIZE;
    float y0 = gy * GRID_SIZE;
    float x1 = x0 + GRID_SIZE;
    float y1 = y0 + GRID_SIZE;
    glColor3f(r, g, b);
    glBegin(GL_QUADS);
      glVertex2f(x0, y0);
      glVertex2f(x1, y0);
      glVertex2f(x1, y1);
      glVertex2f(x0, y1);
    glEnd();
}

void drawGrid() {
    glColor3f(0.2f, 0.2f, 0.2f);
    glBegin(GL_LINES);
    for (int x = 0; x <= SCREEN_WIDTH; x += GRID_SIZE) {
        glVertex2f((float)x, 0.0f);
        glVertex2f((float)x, (float)SCREEN_HEIGHT);
    }
    for (int y = 0; y <= SCREEN_HEIGHT; y += GRID_SIZE) {
        glVertex2f(0.0f, (float)y);
        glVertex2f((float)SCREEN_WIDTH, (float)y);
    }
    glEnd();
}

void drawPolygonOutline() {
    if (polygonPts.empty()) return;
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(2.0f);
    if (polygonClosed) {
        glBegin(GL_LINE_LOOP);
        for (auto &p : polygonPts) {
            float sx = p.x * GRID_SIZE + GRID_SIZE/2.0f;
            float sy = p.y * GRID_SIZE + GRID_SIZE/2.0f;
            glVertex2f(sx, sy);
        }
        glEnd();
    } else {
        glBegin(GL_LINE_STRIP);
        for (auto &p : polygonPts) {
            float sx = p.x * GRID_SIZE + GRID_SIZE/2.0f;
            float sy = p.y * GRID_SIZE + GRID_SIZE/2.0f;
            glVertex2f(sx, sy);
        }
        glEnd();
    }
}

void drawVertices() {
    for (auto &p : polygonPts) {
        drawQuadAtGrid(p.x, p.y, 1.0f, 1.0f, 0.0f);
    }
}

bool pointEquals(const Point &a, const Point &b) {
    return a.x == b.x && a.y == b.y;
}

// Bresenham integer line points (grid coordinates)
vector<pair<int,int>> getLinePoints(int x0, int y0, int x1, int y1) {
    vector<pair<int,int>> pts;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    while (true) {
        pts.emplace_back(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }
    return pts;
}

// Ray casting point-in-polygon for grid coordinates
bool isInsidePolygon(int gx, int gy) {
    if (polygonPts.size() < 3) return false;
    bool inside = false;
    size_t n = polygonPts.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        int xi = polygonPts[i].x, yi = polygonPts[i].y;
        int xj = polygonPts[j].x, yj = polygonPts[j].y;
        bool intersect = ((yi > gy) != (yj > gy)) &&
            (gx < (xj - xi) * (double)(gy - yi) / (double)(yj - yi + 0.0) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

// Build boundary cells set for boundary fill algorithm
void computeBoundaryCells() {
    boundaryCells.clear();
    if (polygonPts.size() < 2) return;
    size_t n = polygonPts.size();
    for (size_t i = 0; i < n; ++i) {
        Point p1 = polygonPts[i];
        Point p2 = polygonPts[(i+1) % n];
        auto pts = getLinePoints(p1.x, p1.y, p2.x, p2.y);
        for (auto &pp : pts) boundaryCells.insert(pp);
    }
}

// SCANLINE FILL (grid-based)
void scanlineFill() {
    if (polygonPts.size() < 3) return;
    currentMode = SCANLINE;
    filledCells.clear();
    computeBoundaryCells();

    int minY = polygonPts[0].y, maxY = polygonPts[0].y;
    for (auto &p : polygonPts) {
        minY = min(minY, p.y);
        maxY = max(maxY, p.y);
    }

    struct Edge { int x0, y0, x1, y1; };
    vector<Edge> edges;
    size_t n = polygonPts.size();
    for (size_t i = 0; i < n; ++i) {
        Point a = polygonPts[i];
        Point b = polygonPts[(i+1) % n];
        if (a.y == b.y) continue;
        if (a.y < b.y) edges.push_back({a.x, a.y, b.x, b.y});
        else edges.push_back({b.x, b.y, a.x, a.y});
    }

    for (int y = minY; y <= maxY; ++y) {
        vector<double> inters;
        for (auto &e : edges) {
            if (e.y0 <= y && y < e.y1) {
                double x = e.x0 + (double)(y - e.y0) * (double)(e.x1 - e.x0) / (double)(e.y1 - e.y0);
                inters.push_back(x);
            }
        }
        sort(inters.begin(), inters.end());
        for (size_t i = 0; i + 1 < inters.size(); i += 2) {
            int xs = (int)ceil(inters[i]);
            int xe = (int)floor(inters[i+1]);
            for (int x = xs; x <= xe; ++x) {
                filledCells.insert({x, y});
                glutPostRedisplay();
                sleep_ms(DELAY_MS);
            }
        }
    }
    currentMode = NONE;
    glutPostRedisplay();
}

// FLOOD FILL (4-connected)
void floodFill4(int sx, int sy) {
    if (!isInsidePolygon(sx, sy)) return;
    currentMode = FLOOD4;
    filledCells.clear();
    computeBoundaryCells();

    int maxGX = SCREEN_WIDTH / GRID_SIZE;
    int maxGY = SCREEN_HEIGHT / GRID_SIZE;
    queue<pair<int,int>> q;
    set<pair<int,int>> visited;
    q.push({sx, sy});

    while (!q.empty()) {
        auto [x, y] = q.front(); q.pop();
        if (visited.count({x,y})) continue;
        if (x < 0 || x >= maxGX || y < 0 || y >= maxGY) continue;
        if (!isInsidePolygon(x, y)) continue;

        visited.insert({x,y});
        filledCells.insert({x,y});
        glutPostRedisplay();
        sleep_ms(DELAY_MS);

        vector<pair<int,int>> neigh = {{x+1,y},{x-1,y},{x,y+1},{x,y-1}};
        for (auto &nb : neigh) if (!visited.count(nb)) q.push(nb);
    }
    currentMode = NONE;
    glutPostRedisplay();
}

// FLOOD FILL (8-connected)
void floodFill8(int sx, int sy) {
    if (!isInsidePolygon(sx, sy)) return;
    currentMode = FLOOD8;
    filledCells.clear();
    computeBoundaryCells();

    int maxGX = SCREEN_WIDTH / GRID_SIZE;
    int maxGY = SCREEN_HEIGHT / GRID_SIZE;
    queue<pair<int,int>> q;
    set<pair<int,int>> visited;
    q.push({sx, sy});

    while (!q.empty()) {
        auto [x, y] = q.front(); q.pop();
        if (visited.count({x,y})) continue;
        if (x < 0 || x >= maxGX || y < 0 || y >= maxGY) continue;
        if (!isInsidePolygon(x, y)) continue;

        visited.insert({x,y});
        filledCells.insert({x,y});
        glutPostRedisplay();
        sleep_ms(DELAY_MS);

        for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) {
            if (dx==0 && dy==0) continue;
            pair<int,int> nb = {x+dx, y+dy};
            if (!visited.count(nb)) q.push(nb);
        }
    }
    currentMode = NONE;
    glutPostRedisplay();
}

// BOUNDARY FILL
void boundaryFillSeed(int sx, int sy) {
    if (!isInsidePolygon(sx, sy)) return;
    currentMode = BOUNDARY;
    filledCells.clear();
    computeBoundaryCells();

    int maxGX = SCREEN_WIDTH / GRID_SIZE;
    int maxGY = SCREEN_HEIGHT / GRID_SIZE;
    queue<pair<int,int>> q;
    set<pair<int,int>> visited;
    q.push({sx, sy});

    while (!q.empty()) {
        auto [x, y] = q.front(); q.pop();
        if (visited.count({x,y})) continue;
        if (x < 0 || x >= maxGX || y < 0 || y >= maxGY) continue;
        if (boundaryCells.count({x,y})) continue;

        visited.insert({x,y});
        filledCells.insert({x,y});
        glutPostRedisplay();
        sleep_ms(DELAY_MS);

        vector<pair<int,int>> neigh = {{x+1,y},{x-1,y},{x,y+1},{x,y-1}};
        for (auto &nb : neigh) if (!visited.count(nb) && !boundaryCells.count(nb)) q.push(nb);
    }
    currentMode = NONE;
    glutPostRedisplay();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);
    for (auto &c : filledCells) drawQuadAtGrid(c.first, c.second, 0.0f, 1.0f, 0.4f);
    drawGrid();
    drawPolygonOutline();
    drawVertices();
    glutSwapBuffers();
}

void reshape(int w, int h) {
    SCREEN_WIDTH = w;
    SCREEN_HEIGHT = h;
    glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y) {
    if (key == 27) exit(0);
    else if (key == 'c' || key == 'C') {
        polygonPts.clear();
        polygonClosed = false;
        filledCells.clear();
        boundaryCells.clear();
        currentMode = NONE;
    } else if (key == '1') {
        if (polygonClosed) thread([](){ scanlineFill(); }).detach();
        else cout << "Close polygon first.\n";
    } else if (key == '2') { if (polygonClosed) currentMode = FLOOD4_WAIT; }
    else if (key == '3') { if (polygonClosed) currentMode = FLOOD8_WAIT; }
    else if (key == '4') { if (polygonClosed) { currentMode = BOUNDARY_WAIT; computeBoundaryCells(); } }
    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    if (state != GLUT_DOWN) return;
    int gx, gy; screenToGrid(x, y, gx, gy);
    if (button == GLUT_LEFT_BUTTON) {
        if (currentMode == FLOOD4_WAIT) { if(isInsidePolygon(gx,gy)) { thread([gx,gy](){ floodFill4(gx,gy); }).detach(); currentMode=NONE; } }
        else if (currentMode == FLOOD8_WAIT) { if(isInsidePolygon(gx,gy)) { thread([gx,gy](){ floodFill8(gx,gy); }).detach(); currentMode=NONE; } }
        else if (currentMode == BOUNDARY_WAIT) { if(isInsidePolygon(gx,gy)) { thread([gx,gy](){ boundaryFillSeed(gx,gy); }).detach(); currentMode=NONE; } }
        else if (!polygonClosed) {
            Point p{gx, gy};
            if (polygonPts.empty() || !pointEquals(p, polygonPts.back())) polygonPts.push_back(p);
        }
    } else if (button == GLUT_RIGHT_BUTTON && !polygonClosed && polygonPts.size() >= 3) {
        polygonClosed = true;
        computeBoundaryCells();
    }
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(SCREEN_WIDTH, SCREEN_HEIGHT);
    glutCreateWindow("Polygon Fill Algorithms (macOS GLUT)");
    glClearColor(0.08f, 0.08f, 0.08f, 1.0f);

    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluOrtho2D(0, SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);

    cout << "Polygon Fill Visualizer (macOS GLUT)\n"
         << "Left click: add point (grid snapped)\n"
         << "Right click: close polygon\n"
         << "1: Scanline fill\n2: Flood fill (4-connected)\n3: Flood fill (8-connected)\n4: Boundary fill\nc: Clear\nESC: Exit\n";

    glutMainLoop();
    return 0;
}

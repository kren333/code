#include <math.h>
#include <set>
#include<iostream>
#include<queue>
using namespace std;

class Node {
    
    int x;
    int y; // x, y coords of the node; used for identification of the node
    double g; // g(x); what's the shortest dist to this point computed thus far?
    double h; // h(x); how far from the goal do we think this will be?
    // note about nbors: LENGTH MUST BE 8 IN THIS IMPLEMENTATION (verify with asserts below)
    public:
        // declaration and get functions to access node properties
        // TODO: delete nbors? calculate the nbors as you go along rather than getting the entire graph befor eyous tart
        Node (int x, int y, double g, double h) {
            this->x = x;
            this->y = y;
            this->g = g;
            this->h = h;
        }

        /* GET FUNCTIONS */

        double getG () const {return g;}
        double getH () const {return h;}
        double getF () const {return g + h;}
        int getX () const {return x;}
        int getY () const {return y;}

        /* UTILITY FUNCTIONS */

        bool cmpNode (Node a, Node b) {
            return a.x == b.x && a.y == b.y;
        }
};

/* helper class to compare two nodes based on their estimated distance to the goal */ 
struct compareFx {
    bool operator () (const Node& a, const Node& b) {
        return a.getF() > b.getF();
    }
};

// priority queue, sorted desc by heuristic, of nodes to traverse
priority_queue <Node, vector<Node>, compareFx > OPEN;
// set of previously opened nodes
set<Node>CLOSED;

int main( int argc, char *argv[], char *envp[] )
{
    Node* bob = new Node (1, 1, 1, 1);
    Node * joe = new Node (10,110,0,0);
    cout << (*bob).getF() << "\n";
    cout << (*joe).getF() << "\n";
    OPEN.push(*joe);
    OPEN.push(*bob);
    Node newBob = OPEN.top();
    OPEN.pop();
    cout << OPEN.empty() << "\n";
    OPEN.pop();
    cout << newBob.getF() << "\n";
    cout << newBob.getX() << "\n";
    cout << OPEN.empty() << "\n"; // returns 1 when empty, 0 when not empty
}


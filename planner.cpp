/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include <set>
#include<iostream>
#include<queue>
#include<tuple>
using namespace std;

/* MOVES */
// saves an indefinite number of moves 
// we will call each move the x, y coordinate pair that the bot should move to next
// the back (e.g. nth entry) of the vector will represent the next move, with the front representing coords at the final point
typedef vector< tuple<int, int> > moveVector;
moveVector moves;


/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // if empty then compute the plan
    // else just return the next point in the vector and pop the element

    /* NODE CLASS: describes one node in the 8-connected grid */
    class Node {
        
        tuple<int, int> coords; // x, y coords of the node; used for identification of the node
        double g; // g(x); what's the shortest dist to this point computed thus far?
        double h; // h(x); how far from the goal do we think this will be?
        Node nbors[NUMOFDIRS]; // the set of neighbors that will be put into OPEN; ith entry represents moving dX[i] and dY[i]
        // note about nbors: LENGTH MUST BE 8 IN THIS IMPLEMENTATION (verify with asserts below)
        
        public:
            // declaration and get functions to access node properties
            // TODO: delete nbors? calculate the nbors as you go along rather than getting the entire graph befor eyous tart
            Node (tuple <int, int> coords, double g, double h, Node nbors[NUMOFDIRS]) {
                this->coords = coords;
                this->g = g;
                this->h = h;
                this->nbors = nbors;
            }

            /* GET FUNCTIONS */

            double getG () const {return g;}
            double getH () const {return h;}
            double getF () const {return g + h;}
            double getCoords () const {return coords;}
            double getNbors () const {return nbors;}

            /* UTILITY FUNCTIONS */
    };

    /* helper class to compare two nodes based on their estimated distance to the goal */ 
    class compareFx {
        public:
            bool compareHx (const Node& a, const Node& b) {
                return a.getF() > b.getF();
            }
    };

    // priority queue, sorted desc by heuristic, of nodes to traverse
    priority_queue <Node, vector<Node>, compareFx > OPEN;
    // set of previously opened nodes
    set<Node>CLOSED;

    // TODO: give the nodes its heuristic values,
    // add the starting point to the queue, 
    // open its neighbors, 
    // then move it to closed, 
    // update all the queue/set information with regards to g(x) and h(x)

    // QUESTIONS
    /*
    - how does the heuristic work when you have a moving target? 
    - what would i want to find the euclidean distance to?
    - how does the planning function work when run? does it run once every x ms? does it run a single time at the start of the program? 
    - do i need to plan out the entire path in the call below or just a single step?
    - also basic syntax question: is it OK to specify length of arr must be NUMOFDIRS in the init function?
    - am i supposed to insert the current node (with coords <robotposeX, robotposeY>) into OPEN and go from there?
    */

    /* ANSWERS
    - starting heuristic would be euclidean distance to the last location (a bit naive but good enough for now)
    - the planning function runs in 'ticks' in parallel with the movement of both bots
    - plan out the entire path beforehand; a lot at the start and then you just move ur bot based off global var
    */

    // if we have not yet put in all of the moves, insert start into the queue and update the moves
    if moves.empty() {
        // TODO: use push front to incrementally insert the newest move at the front of the vector of moves
        // basically here is where we run A* to populate moves :)
    }
    // otherwise, just return the next move and pop
    else {
        int n = moves.size();
        tuple<int, int> nextMove = moves[n-1];
        // update coords and remove from moves vector
        action_ptr[0] = get<0>(nextMove);
        action_ptr[1] = get<1>(nextMove);
        moves.pop_back();
        return;
    }

    /* OUTDATED CODE BELOW: GREEDY/NAIVE SOLUTION */

    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);

    int bestX = 0, bestY = 0; // robot will not move if greedy action leads to collision
    double olddisttotarget = (double)sqrt(((robotposeX-goalposeX)*(robotposeX-goalposeX) + (robotposeY-goalposeY)*(robotposeY-goalposeY)));
    double disttotarget;
    for(int dir = 0; dir < NUMOFDIRS; dir++)
    {
        int newx = robotposeX + dX[dir];
        int newy = robotposeY + dY[dir];

        if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)
        {
            if (((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh))  //if free
            {
                disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                if(disttotarget < olddisttotarget)
                {
                    olddisttotarget = disttotarget;
                    bestX = dX[dir];
                    bestY = dY[dir];
                }
            }
        }
    }
    robotposeX = robotposeX + bestX;
    robotposeY = robotposeY + bestY;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}
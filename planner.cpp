/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <iostream>
#include <assert.h>

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]


/* Output Arguments */
#define	ACTION_OUT	plhs[0]

/*access to the map is shifted to account for 0-based indexing in the map, whereas
1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)*/
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

/* Primitives Information */
#define NUMOFDIRS 8
#define NUMOFPRIMS 5
#define NUMOFINTERSTATES 10
#define NUMOFDIM 3

#define RES 0.1

typedef float PrimArray[NUMOFDIRS][NUMOFPRIMS][NUMOFINTERSTATES][NUMOFDIM];

int temp = 0;

bool need_new_plan = true;

// Initialize CLOSED list (storing parent_id and action_id of states)
std::unordered_map<int, std::pair<int, int>> closed_list;

// Map to store the waypoints and actions in the generated path
std::unordered_map<int, int> curr_plan;

// Create data structure to keep track of visited states
std::unordered_set<int> visited_states;

// NOTE: OPEN list will be initialized in the planner function instead. 

float prior_pose_x;
float prior_pose_y;

struct state 
{
    int index;
    float x;
    float y;
    float theta;
    float cost;
    float value;
    int parent_id;
    int action_id;
};

struct compare
{
    bool operator()(const state& s1, const state& s2)
    {
        return s1.value > s2.value;
    } 
};

bool applyaction(double *map, int x_size, int y_size, float robotposeX, float robotposeY, float robotposeTheta,
                 float *newx, float *newy, float *newtheta, PrimArray mprim, int dir, int prim)
{
    int i;
    for (i = 0; i < NUMOFINTERSTATES; i++) {
        *newx = robotposeX + mprim[dir][prim][i][0];
        *newy = robotposeY + mprim[dir][prim][i][1];
        *newtheta = mprim[dir][prim][i][2];
        
        int gridposx = (int)(*newx / RES + 0.5);
        int gridposy = (int)(*newy / RES + 0.5);

        if (!need_new_plan)
        	std::cout << "newx= " << *newx << ", newy= " << *newy << ", gridx= " << gridposx << ", gridy= " << gridposy << std::endl;
        

        /* check validity */
        if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size){
            return false;
        }
        if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
            return false;
        }
    }


    return true;
}

int getPrimitiveDirectionforRobotPose(float angle)
{
    /* returns the direction index with respect to the PrimArray */
    /* normalize bw 0 to 2pi */
    if (angle < 0.0) {
        angle += 2 * M_PI;
    }
    int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
    if (dir == 8) {
        dir = 0;
    }
    return dir;
}

inline double getHeuristics(float x_curr, float y_curr, float x_goal, float y_goal)
{
    return (double)sqrt(((x_curr-x_goal)*(x_curr-x_goal) + (y_curr-y_goal)*(y_curr-y_goal)));
}

inline int pose2Index(float robotposeX, float robotposeY, int x_size, int y_size) 
{
    int grid_x = (int)(robotposeX / RES + 0.5f);
    int grid_y = (int)(robotposeY / RES + 0.5f);

    return GETMAPINDEX(grid_x, grid_y, x_size, y_size);
}

void planner(double* map,
            int x_size,
            int y_size,
            float robotposeX,
            float robotposeY,
            float robotposeTheta,
            float goalposeX,
            float goalposeY,
            PrimArray mprim,
            int *prim_id) 
{
    if (need_new_plan)
    {
    	// std::cout << "Generating new plan..." << std::endl;
        // Initialize OPEN list (storing index of states and their values f=c+h)
        std::priority_queue<state, std::vector<state>, compare> open_list;

        closed_list.clear();
        curr_plan.clear();
        visited_states.clear();

        //Assign weight to A-star algorithm
        int epsilon = 5; //1.5 
        
        //Initialization
        //int cost = 0;
        int initIndex;
        
        state waypoint;
        waypoint.x = robotposeX;
        waypoint.y = robotposeY;
        waypoint.theta = robotposeTheta;
        waypoint.index = pose2Index(robotposeX, robotposeY, x_size, y_size);
        //Record initial pose index
        initIndex = waypoint.index;
        waypoint.cost = 0;
        waypoint.value = 0 + epsilon * getHeuristics(robotposeX, robotposeY, goalposeX, goalposeY);
        //Create "fake" parent and action for start pose
        waypoint.parent_id = -1;
        waypoint.action_id = -1;
        open_list.push(waypoint);
        visited_states.insert(initIndex);

        int num_expansion = 0;
        //Create pair to store parent_id and action_id
        std::pair<int, int> p;
        //Index of last visited state
        int state_id;

        bool found_goal = false;

        while (!open_list.empty() && !found_goal) //&& num_expansion < 10000)
        {
            //Remove state with smallest value from OPEN
            state min_state = open_list.top();
            open_list.pop();
            //Insert state info (index, parent, prior action) into CLOSED 
            p.first = min_state.parent_id;
            p.second = min_state.action_id;
            closed_list[min_state.index] = p;
            //Update state_id as the index of the latest min_state
            state_id = min_state.index;

            //Get current orientation of robot
            int dir = getPrimitiveDirectionforRobotPose(min_state.theta);

            for (int prim = 0; prim < NUMOFPRIMS; prim++)
            {
                float newx, newy, newtheta;
                bool path_valid = applyaction(map, x_size, y_size, min_state.x, min_state.y, min_state.theta, 
                    &newx, &newy, &newtheta, mprim, dir, prim);
                
                int new_index = pose2Index(newx, newy, x_size, y_size);
                if (path_valid && closed_list.find(new_index) == closed_list.end())
                {
                    waypoint.x = newx;
                    waypoint.y = newy;
                    waypoint.theta = newtheta;
                    waypoint.index = new_index;
                    double newh = getHeuristics(newx, newy, goalposeX, goalposeY);
                    waypoint.cost = min_state.cost + 1;
                    waypoint.value = waypoint.cost + epsilon * newh;
                    waypoint.parent_id = min_state.index;
                    waypoint.action_id = prim;
                    //Break if reaching the goal
                    if (newh < 0.25)
                    {
                        p.first = min_state.index;
                        p.second = prim;
                        //Insert final state into CLOSED
                        closed_list[waypoint.index] = p;
                        //Update state_id as the index of the goal state 
                        state_id = waypoint.index;
                        found_goal = true;
                        std::cout << "Found solution!" << std::endl;
                        break;
                    }
                    //Insert new state into OPEN iff it hasn't been visited
                    else if (visited_states.find(new_index) == visited_states.end())
                    {
                        open_list.push(waypoint);
                        //Mark the new (unvisited) state as visited 
                        visited_states.insert(new_index);
                    }
                }
            }
            num_expansion++;
            //cost = cost + 0.1;
        }

        std::cout << "Num of expansions: " << num_expansion << std::endl;

        //Back-traversal of the path
        while (state_id != initIndex)
        {
            int prior_action = closed_list[state_id].second;

            //Move state to its parent state
            state_id = closed_list[state_id].first;

            //Store the state and action in curr_plan
            curr_plan[state_id] = prior_action;
        }

        *prim_id = curr_plan.find(initIndex)->second;

        
        //Use this plan until getting closer to the goal 
        if (found_goal) need_new_plan = false;
    } 
    else 
    {
        int new_index = pose2Index(robotposeX, robotposeY, x_size, y_size);

        assert(curr_plan.find(new_index) != curr_plan.end());
        assert(closed_list.find(new_index) != closed_list.end());

        //Return the value (action_id) corresponded to the key (index)
        //"*prim_id = curr_plan.find(new_index)->second;" doesn't work somehow...      
        *prim_id = curr_plan[new_index];

        //Check heuristics at updated location 
        double newh = getHeuristics(robotposeX, robotposeY, goalposeX, goalposeY);
        if (newh < 5)
            need_new_plan = true;
        if (getHeuristics(prior_pose_x, prior_pose_y, robotposeX, robotposeY) <= 0.11)
            //Getting stuck...Need re-plan!!
            need_new_plan = true;
        
        /*
        //FOR DEBUGGING ...
        curr_plan.erase(new_index);
        std::cout << "Map size:" << curr_plan.size() << std::endl;
        assert(curr_plan.size() > 0);

        float newx, newy, newtheta;
        int dir = getPrimitiveDirectionforRobotPose(robotposeTheta);
        bool valid = applyaction(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, 
                    &newx, &newy, &newtheta, mprim, dir, *prim_id);
        assert(valid);
        std::cout << "About to head to: " << newx << ", " << newy << " with dir= " << dir << std::endl;
        */
        
    }
    //Record current robot pose as prior to be used in the next step
    prior_pose_x = robotposeX;
    prior_pose_y = robotposeY;
}

static void greedyPlanner(
		   double*	map,
		   int x_size,
 		   int y_size,
           float robotposeX,
           float robotposeY,
           float robotposeTheta,
           float goalposeX,
           float goalposeY,
           PrimArray mprim,
           int *prim_id)
{   
    printf("temp=%d\n", temp);
    temp = temp+1;

    *prim_id = 0; /* arbitrary action */
    
    /*printf("robot: %d %d; ", robotposeX, robotposeY);*/
    /*printf("goal: %d %d;", goalposeX, goalposeY);*/
    
	/*for now greedily move towards the target, */
	/*but this is where you can put your planner */
	double mindisttotarget = 1000000;
    int dir;
    int prim;
	dir = getPrimitiveDirectionforRobotPose(robotposeTheta);
    
    for (prim = 0; prim < NUMOFPRIMS; prim++) {
        float newx, newy, newtheta;
        bool ret;
        ret = applyaction(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, &newx, &newy, &newtheta, mprim, dir, prim);
            /* skip action that leads to collision */
        if (ret) {
            double disttotarget = (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
            if(disttotarget < mindisttotarget){
                mindisttotarget = disttotarget;
                
                *prim_id = prim;
            }            
        }

    }
    printf("action %d\n", *prim_id);
    return;
}

/*prhs contains input parameters (3): 
1st is matrix with all the obstacles
2nd is a row vector <x,y> for the robot pose
3rd is a row vector <x,y> for the target pose
plhs should contain output parameters (1): 
1st is a row vector <dx,dy> which corresponds to the action that the robot should make*/

void parseMotionPrimitives(PrimArray mprim)
{
    FILE * fp;
    fp = fopen ("unicycle_8angles.mprim", "r+");
    char skip_c[100];
    int skip_f;
    float resolution;
    int num_angles;
    int num_mprims;
    fscanf(fp, "%s %f", skip_c, &resolution);
    fscanf(fp, "%s %d", skip_c, &num_angles);
    fscanf(fp, "%s %d", skip_c, &num_mprims);

    int i, j, k;
    for (i = 0; i < NUMOFDIRS; ++i) {
        for (j = 0; j < NUMOFPRIMS; ++j) {
            fscanf(fp, "%s %d", skip_c, &skip_f);
            for (k = 0; k < NUMOFINTERSTATES; ++k) {
                fscanf(fp, "%f %f %f", &mprim[i][j][k][0], &mprim[i][j][k][1], &mprim[i][j][k][2]);
            }

        }
    }
    fclose(fp);
}

void mexFunction( int nlhs, mxArray *plhs[], 
		  int nrhs, const mxArray*prhs[] )
     
{

    /* Read motion primtives */
    PrimArray motion_primitives;
    parseMotionPrimitives(motion_primitives);

    /* Check for proper number of arguments */    
    if (nrhs != 3) { 
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Three input arguments required."); 
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
    if(robotpose_M != 1 || robotpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 3.");         
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    float robotposeX = (float)robotposeV[0];
    float robotposeY = (float)robotposeV[1];
    float robotposeTheta = (float)robotposeV[2];
    
    /* get the dimensions of the goalpose and the goalpose itself*/     
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 3){
	    mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
                "goalpose vector should be 1 by 3.");         
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    float goalposeX = (float)goalposeV[0];
    float goalposeY = (float)goalposeV[1];
        
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( 1, 1, mxINT8_CLASS, mxREAL); 
    int* action_ptr = (int*) mxGetData(ACTION_OUT);

    //need_new_plan = true;

    /* Do the actual planning in a subroutine */
    planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, motion_primitives, &action_ptr[0]);

    return;
    
}






/* demo1 - main.cpp
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>
#include <vector>
#include <algorithm>

#include "model/ModelInterface.h"
#include "graphics/GraphicsInterface.h"
#include "graphics/ChaiGraphics.h"
#include "simulation/Sai2Simulation.h"
#include "chai3d.h"

#include "timer/LoopTimer.h"

// #include "force_sensor/ForceSensorSim.h"
// #include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

#include "widgets/ToggleButton.h"
#include "widgets/ToggleValve.h"
#include "widgets/ToggleLever.h"

using namespace std;
using namespace Eigen;
using namespace chai3d;

const string world_fname = "resources/demo1/world.urdf";
const string robot_fname = "resources/demo1/robot.urdf";
const string robot_name = "Robot1";
string camera_name = "camera_front";
const string end_effector_name = "link4";
bool target_reached = false;

//target list
double targets [][6] = {{1.0, 1.0, 1.0}, {0.7, 0.7, 0.55}, {1.3, 0.7, 0.6}, {1.3,0.62,0.5}, {1.6, 0.62, 0.5}, {1.0, 1.0, 1.0}};

// global variables
Eigen::VectorXd q_home;
Eigen::Vector3d target_pos;
// ForceSensorSim* tool_force_sensor;
// ForceSensorDisplay* tool_force_display;
ToggleButton* button;
ToggleValve* valve;
ToggleLever* lever;

// simulation loop
bool fSimulationRunning = false;
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);
void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);

bool f_global_sim_pause = false; // use with caution!
// bool f_global_sim_pause = true; // use with caution!

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics_ext = new Graphics::GraphicsInterface(world_fname, Graphics::chai, Graphics::urdf, false);
	Graphics::ChaiGraphics* graphics = dynamic_cast<Graphics::ChaiGraphics*> (graphics_ext->_graphics_internal);

	// load robots
	auto robot = new Model::ModelInterface(robot_fname, Model::rbdl, Model::urdf, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, Simulation::urdf, false);

	// set initial condition
    target_pos = Eigen::Vector3d(1.04, 1.0, 0.5);
	q_home.setZero(robot->dof());
	q_home << 1.0,
                90.0/180.0*M_PI,
                -90.0/180.0*M_PI,
                45.0/180.0*M_PI,
                90.0/180.0*M_PI;

	robot->_q = q_home;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();

	// togglewidgits
	button = new ToggleButton("Button1", Vector3d(1.0, 1.0, 0.475), Matrix3d::Identity(), sim);
	graphics->_world->addChild(button);
    lever = new ToggleLever("Lever", Vector3d(1.0, 0.7, 0.475), Matrix3d::Identity(), sim);
    graphics->_world->addChild(lever);
    valve = new ToggleValve("Valve", Vector3d(1.4, 0.7, 0.475), Matrix3d::Identity(), sim);
    graphics->_world->addChild(valve);
	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot, sim);

	// next start the control thread

	thread ctrl_thread(control, robot, sim);


    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		button->updateGraphics();
        valve->updateGraphics();
        lever->updateGraphics();
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}
//------------------------------------------------------------------------------
//change to target n+1 after target n has been reached
void getTarget(int index){
    if(index > sizeof(targets)/sizeof(targets[0])){
        target_pos = Eigen::Vector3d(1.0, 1.0, 1.0);
    }
    else{
        double x = targets[index][0];
        double y = targets[index][1];
        double z = targets[index][2];
        cout << "target pos: " << x << " " << y << " " << z<< endl;
        target_pos = Eigen::Vector3d(x, y, z);
    }
}


//------------------------------------------------------------------------------
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(800); //800Hz timer
	double last_time = timer.elapsedTime(); //secs

	// control variables
	Eigen::VectorXd tau(robot->dof());
	tau.setZero();
	Eigen::VectorXd tau_grav(robot->dof());
	Eigen::MatrixXd Jv, Lambda_v, N_v;
	Eigen::Vector3d vel_control, ee_pos;
    Eigen::Vector3d vel_control2;
    double target_threshold = 0.02; //how much error allowed before target is reached
    int target_num = 0;
	Eigen::Vector3d ee_link_pos(0.1, 0.0, 0.0);

	bool fTimerDidSleep = true;
	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// check if paused
		if (f_global_sim_pause) { continue;}

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);

		// update kinematic models
		robot->updateModel();

		robot->gravityVector(tau_grav);

		robot->Jv(Jv, end_effector_name, ee_link_pos);
		robot->position(ee_pos, end_effector_name, ee_link_pos);

		Lambda_v = (Jv*robot->_M*Jv.transpose()).inverse();
		N_v = (Eigen::MatrixXd::Identity(robot->dof(), robot->dof()) - robot->_M*Jv.transpose()*Lambda_v*Jv);

		vel_control = (target_pos - ee_pos)*30.0/30.0;
		double speed_control = vel_control.norm();
		if(speed_control > 0.1) {
			vel_control = 0.1*vel_control/speed_control;
		}

		tau = (-Jv.transpose() *Lambda_v* 30.0*(Jv*robot->_dq - vel_control)) + N_v.transpose()*robot->_M*(-20.0*(robot->_q - q_home) - 10.0*robot->_dq);

		sim->setJointTorques(robot_name, tau+tau_grav);
        if(vel_control.norm() < target_threshold){
            target_reached = true;
            target_reached = false;
            cout << "target changed" << endl;
            getTarget(target_num);
            target_num = target_num + 1;
        }

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1500); //1.5kHz timer
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;
		if (!f_global_sim_pause) {
			button->updateDynamics();
            valve->updateDynamics();
            lever->updateDynamics();
			sim->integrate(loop_dt);
		}

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - Panel Manipulation Project", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------


void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
    if ((key == 'P' || key == 'p') && action == GLFW_PRESS)
    {
        // pause simulation
        f_global_sim_pause = !f_global_sim_pause;
    }
    if ((key == '1') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_front";
    }
    if ((key == '2') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_top";
    }
    if ((key == '3') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_side";
    }
    if ((key == '4') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_oppside";
    }
    if ((key == '5') && action == GLFW_PRESS)
    {
        // change camera
        camera_name = "camera_frontside";
    }
}

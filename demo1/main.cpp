/* demo1 - main.cpp
*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "model/ModelInterface.h"
#include "graphics/GraphicsInterface.h"
#include "graphics/ChaiGraphics.h"
#include "simulation/SimulationInterface.h"
#include "chai3d.h"

#include "timer/LoopTimer.h"

// #include "force_sensor/ForceSensorSim.h"
// #include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;
using namespace Eigen;
using namespace chai3d;

const string world_fname = "resources/demo1/world.urdf";
const string robot_fname = "resources/demo1/robot.urdf";
const string robot_name = "Robot1";
string camera_name = "camera_front";
const string end_effector_name = "link4";


// global variables
Eigen::VectorXd q_home;
// ForceSensorSim* tool_force_sensor;
// ForceSensorDisplay* tool_force_display;

// simulation loop
bool fSimulationRunning = false;
void control(Model::ModelInterface* robot, Simulation::SimulationInterface* sim);
void simulation(Model::ModelInterface* robot, Simulation::SimulationInterface* sim);

// bool f_global_sim_pause = false; // use with caution!
bool f_global_sim_pause = true; // use with caution!

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
	auto sim = new Simulation::SimulationInterface(world_fname, Simulation::sai2simulation, Simulation::urdf, false);

	// set initial condition
	q_home.setZero(robot->dof());
	q_home << 1.0,
				90.0/180.0*M_PI,
				-90.0/180.0*M_PI,
				45.0/180.0*M_PI,
				90.0/180.0*M_PI;
	robot->_q = q_home;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();

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
void control(Model::ModelInterface* robot, Simulation::SimulationInterface* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
	double last_time = timer.elapsedTime(); //secs

	// control variables
	Eigen::VectorXd tau(robot->dof());
	tau.setZero();
	Eigen::VectorXd tau_grav(robot->dof());

	Eigen::Vector3d ee_link_pos(0.0, 0.0, 0.4);

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

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot, Simulation::SimulationInterface* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1kHz timer
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;
		if (!f_global_sim_pause) {
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
}

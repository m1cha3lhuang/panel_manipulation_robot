/* ToggleValve.h: Class representing a toggle valve
*/
#ifndef TOGGLE_VALVE_H
#define TOGGLE_VALVE_H

#include "simulation/Sai2Simulation.h"
#include <chai3d.h>
#include <dynamics3d.h>
#include <Eigen/Dense>

struct ToggleValveProperties {
	// geometry. cannot be changed after valve object has been constructed.
	chai3d::cVector3d bottom_part_extents; // length, width, height of valve base part
	chai3d::cVector3d top_part_extents; // length, width, height of valve moving part
	chai3d::cVector3d top_part_relpos; // x, y, z position of top part wrt bottom part

	// graphics. can be changed after valve object has been constructed.
	chai3d::cColorf bottom_part_color;
	chai3d::cColorf top_part_color;

	// dynamics. can be changed after valve object has been constructed.
	double top_part_mass; // mass of top part in kg
	chai3d::cVector3d top_part_inertia; // Ixx, Iyy and Izz for the top part at COM
	double k_spring; // valve spring constant
	double k_damper; // valve damping constant

	// constructor with default properties
	ToggleValveProperties()
	: bottom_part_extents(chai3d::cVector3d(0.04, 0.06, 0.06)),
	top_part_extents(chai3d::cVector3d(0.037, 0.25, 0.015)),
	top_part_relpos(chai3d::cVector3d(0.0,0.0,0.075)),
	bottom_part_color(chai3d::cColorf(0.62, 0.52, 0.0, 1.0)),
	top_part_color(chai3d::cColorf(0.92, 0.82, 0.13, 1.0)),
	top_part_mass(0.03),
	top_part_inertia(chai3d::cVector3d(0.00005, 0.0001, 0.0001)),
    k_spring(0.00),
	k_damper(0.002)
	{ // done
	}
};


class ToggleValve: public chai3d::cGenericObject {
public:
	// states def
	enum State {Off, On};

public:
	// ctor with default properties
	// NOTE: valve is created fixed to the simulation world at the given location.
	// Only the top part of it moves.
	ToggleValve(const std::string& name, const Eigen::Vector3d& world_pos, const Eigen::Matrix3d& world_ori, const Simulation::Sai2Simulation* simulation);

	// ctor with custom properties
	// NOTE: valve is created fixed to the simulation world at the given location.
	// Only the top part of it moves.
	ToggleValve(const ToggleValveProperties& properties, const std::string& name, const Eigen::Vector3d& world_pos, const Eigen::Matrix3d& world_ori, const Simulation::Sai2Simulation* simulation);

	// dtor
	~ToggleValve();

	// update graphics. needs to be called from the application graphics loop.
	void updateGraphics();

	// update dynamics. needs to be called from the application simulation loop.
	void updateDynamics();

	// get current state
	// default state is Off when valve is created
	State getState() const;

	// set valve state.
	/* Note: Leads to undefined behavior if called when valve is in contact
		 with another object */
	void setState(State state);

	// set valve angle in range [-1.0, 1.0]. -1.0 is off, 1.0 is on.
	/* Note: Leads to undefined behavior if called when valve is in contact
		 with another object */
	void setAngle(double angle);

	// get location in world
	Eigen::Vector3d getPosInWorld() const;

	// get orientation in world
	Eigen::Matrix3d getRotInWorld() const;

private:
	// hide default constructor
	ToggleValve() {};

private:
	// properties
	ToggleValveProperties _prop;

	// world position
	Eigen::Affine3d _world_pose;

	// joint extent
	double _joint_extent;

	// graphic representations
	chai3d::cMultiMesh* _mesh_valve;
	chai3d::cMesh* _mesh_valve_top;

	// dynamics representation
	cDynamicBase* _dyn_valve;
	cDynamicJoint* _joint_valve;
};

#endif //TOGGLE_VALVE_H

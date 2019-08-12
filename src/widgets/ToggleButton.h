/* ToggleButton.h: Class representing a toggle button
*/
#ifndef TOGGLE_BUTTON_H
#define TOGGLE_BUTTON_H

#include "simulation/Sai2Simulation.h"
#include <chai3d.h>
#include <dynamics3d.h>
#include <Eigen/Dense>

class ToggleButton: public chai3d::cGenericObject {
public:
	// states def
	enum State {Off, On};

	// ctor
	ToggleButton(std::string name, const chai3d::cVector3d& world_pos, const chai3d::cMatrix3d& world_ori, const Simulation::Sai2Simulation* simulation);

	// dtor
	~ToggleButton();

	// update graphics
	void updateGraphics();

	// update dynamics
	void updateDynamics();

	// // get current state
	// State getState() const;

	// // inspect internal force
	// double getTorque() const;

	// // get contacts
	// void getContacts(chai3d::cDynamicContactList& ret_contact_list) const;

private:
	// hide default constructor
	ToggleButton() {};

public:
	// mechanics parameters of button
	double _k_spring;
	double _b_spring;

	// world position
	Eigen::Affine3d _world_pose;

	// state
	State _curr_state;

	// graphic representations
	chai3d::cMultiMesh* _mesh_button;
	chai3d::cMesh* _mesh_button_top;
	// chai3d::cMesh* _mesh_button_bottom;

	// dynamics representation
	cDynamicBase* _dyn_button;
	cDynamicJoint* _joint_button;
};

#endif //TOGGLE_BUTTON_H
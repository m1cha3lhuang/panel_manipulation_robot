/* ToggleButton.cpp */

#include "widgets/ToggleButton.h"

using namespace Eigen;
using namespace chai3d;

// ctor
ToggleButton::ToggleButton(std::string name, const cVector3d& world_pos, const cMatrix3d& world_ori, const Simulation::Sai2Simulation* simulation):
_k_spring(0.01),
_b_spring(0.002)
{
	m_name = name;

	// set default state
	_curr_state = ToggleButton::Off;

	// create graphics
	_mesh_button = new cMultiMesh();
	_mesh_button->m_name = "graphics";
	_mesh_button_top = new cMesh();
	_mesh_button_top->m_name = "top";
	cCreateBox(_mesh_button_top, 0.06, 0.037, 0.015);
	cColorf top_color(0.92, 0.82, 0.13, 1.0);
	_mesh_button_top->m_material->setColor(top_color);
	_mesh_button_top->setLocalPos(cVector3d(0.0,0.0,0.04));
	_mesh_button_top->setLocalRot(cIdentity3d());
	_mesh_button->addMesh(_mesh_button_top);
	auto mesh_button_bottom = new cMesh();
	mesh_button_bottom->m_name = "bottom";
	cCreateBox(mesh_button_bottom, 0.08, 0.049, 0.02);
	cColorf bottom_color(0.62, 0.52, 0.0, 1.0);
	mesh_button_bottom->m_material->setColor(bottom_color);
	mesh_button_bottom->setLocalPos(cVector3d(0.0,0.0,0.01));
	mesh_button_bottom->setLocalRot(cIdentity3d());
	_mesh_button->addMesh(mesh_button_bottom);
	_mesh_button->setLocalPos(cVector3d(0.0, 0.0, 0.0));
	_mesh_button->setLocalRot(cIdentity3d());
	addChild(_mesh_button);

	// create dynamics button
	_dyn_button = simulation->_world->newBaseObject(world_pos, world_ori);
	_dyn_button->m_name = name;
	auto dyn_mat = new cDynamicMaterial();
	auto dyn_button_bottom = _dyn_button->newLink(dyn_mat);
	dyn_button_bottom->m_name = "bottom";
	dyn_button_bottom->setMassProperties(0.01, cIdentity3d(), cVector3d(0.0, 0.0, 0.0));
	_dyn_button->linkChild(dyn_button_bottom, cVector3d(0.0, 0.0, 0.0), cIdentity3d());
	// TODO: set collision mesh for bottom
	auto dyn_button_top = _dyn_button->newLink(dyn_mat);
	dyn_button_top->m_name = "top";
	cMatrix3d button_inertia(
		0.00005, 0, 0,
		0, 0.0001, 0,
		0, 0, 0.0001
	);
	dyn_button_top->setMassProperties(0.03, button_inertia, cVector3d(0.0, 0.0, 0.0));
	_joint_button = dyn_button_top->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Y);
	_joint_button->m_name = "j0";
	_joint_button->setPos(0.0);
	dyn_button_bottom->linkChild(dyn_button_top, cVector3d(0.0, 0.0, 0.04), cIdentity3d());

	// enable dynamics
	_dyn_button->enableDynamics(true);

	// set button to OFF
	_joint_button->setPos(-M_PI/7.0);

	// set local position of the button in the graphic world
	setLocalPos(world_pos);
	setLocalRot(world_ori);
}

ToggleButton::~ToggleButton() {
	delete _mesh_button; //auto deletes child meshes
	delete _dyn_button; // this does not auto delete links and joints, oh well.
}

ToggleButton::State ToggleButton::getState() const {
	ToggleButton::State ret_state;
	ret_state = (_joint_button->getPos() > M_PI/10.0)? ToggleButton::On: ToggleButton::Off;
	return ret_state;
}

void ToggleButton::updateGraphics() {
	// get joint angle
	double joint_val = _joint_button->getPos();

	// compute orientation
	cMatrix3d rot(cVector3d(0.0, 1.0, 0.0), joint_val);
	_mesh_button_top->setLocalRot(rot);
}

void ToggleButton::updateDynamics() {
	// compute button torque and set torque
	// _joint_button->setForce(0.0);
	double curr_pos = _joint_button->getPos();
	double set_pos = M_PI/7.0;
	if (curr_pos < 0.0) {
		set_pos = -M_PI/7.0;
	}
	_joint_button->setForce(-_k_spring*(curr_pos - set_pos) - _b_spring*_joint_button->getVel());
}


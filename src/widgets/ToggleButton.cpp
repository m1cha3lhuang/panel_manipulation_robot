/* ToggleButton.cpp */

#include <cmath>
#include "widgets/ToggleButton.h"

using namespace Eigen;
using namespace chai3d;

ToggleButton::ToggleButton(
	const ToggleButtonProperties& properties,
	const std::string& name,
	const Eigen::Vector3d& world_pos,
	const Eigen::Matrix3d& world_ori,
	const Simulation::Sai2Simulation* simulation
) {
	// copy properties
	_prop = properties;

	// call default constructor
	ToggleButton(name, world_pos, world_ori, simulation);
}

ToggleButton::ToggleButton(
	const std::string& name,
	const Eigen::Vector3d& world_pos,
	const Eigen::Matrix3d& world_ori,
	const Simulation::Sai2Simulation* simulation
) {
	// copy world position
	_world_pose.translation() = world_pos;
	_world_pose.linear() = world_ori;

	// compute joint extent. button top part rotates about the button y axis.
	double temp1 = 0.5 * sqrt(pow(_prop.top_part_extents.x(), 2) + pow(_prop.top_part_extents.z(), 2));
	double temp2 = asin((_prop.top_part_relpos.z() - _prop.bottom_part_extents.z())/temp1);
	double temp3 = atan(_prop.top_part_extents.z()/_prop.top_part_extents.x());
	_joint_extent = 0.95 * (temp2 - temp3);

	// save name
	m_name = name;

	// create graphics
	_mesh_button = new cMultiMesh();
	_mesh_button->m_name = "graphics";
	// - create and add graphic for bottom part
	auto mesh_button_bottom = new cMesh();
	mesh_button_bottom->m_name = "bottom";
	cCreateBox(mesh_button_bottom, _prop.bottom_part_extents.x(), _prop.bottom_part_extents.y(), _prop.bottom_part_extents.z());
	mesh_button_bottom->m_material->setColor(_prop.bottom_part_color);
	mesh_button_bottom->setLocalPos(cVector3d(0.0,0.0,0.5*_prop.bottom_part_extents.z()));
	//^offset z by half the height of the base part
	_mesh_button->addMesh(mesh_button_bottom);
	// - create and add graphic for top part
	_mesh_button_top = new cMesh();
	_mesh_button_top->m_name = "top";
	cCreateBox(_mesh_button_top, _prop.top_part_extents.x(), _prop.top_part_extents.y(), _prop.top_part_extents.z());
	_mesh_button_top->m_material->setColor(_prop.top_part_color);
	_mesh_button_top->setLocalPos(_prop.top_part_relpos);
	_mesh_button->addMesh(_mesh_button_top);
	addChild(_mesh_button);

	// create dynamics
	// - create a new base object in the simulation world
	_dyn_button = simulation->_world->newBaseObject(world_pos, world_ori);
	_dyn_button->m_name = name;
	// - create a default type material to apply
	auto dyn_mat = new cDynamicMaterial();
	// - create bottom link
	auto dyn_button_bottom = _dyn_button->newLink(dyn_mat);
	dyn_button_bottom->m_name = "bottom";
	dyn_button_bottom->setMassProperties(0.01, cIdentity3d(), cVector3d(0.0, 0.0, 0.0));
	//^this is a fixed link, so these are just good default mass properties
	// - create collision mesh from graphics mesh
	auto mesh_coll_bottom = mesh_button_bottom->copy(true, true, true, true);
	auto mmesh_coll_bottom = new cMultiMesh();
	mmesh_coll_bottom->addMesh(mesh_coll_bottom);
	mmesh_coll_bottom->setLocalPos(cVector3d(0.0,0.0,0.5*_prop.bottom_part_extents.z())); 
	//^offset the mesh by half the height of the bottom part
	dyn_button_bottom->setCollisionModel(mmesh_coll_bottom);
	dyn_button_bottom->buildCollisionHull(0.0004, 0.0004); // set 0.0004 meters as mesh conversion error
	// - add bottom link to base. position it at the center
	_dyn_button->linkChild(dyn_button_bottom, cVector3d(0.0, 0.0, 0.0), cIdentity3d());
	// - create top link with same material
	auto dyn_button_top = _dyn_button->newLink(dyn_mat);
	dyn_button_top->m_name = "top";
	// - set mass properties
	cMatrix3d button_inertia(
		_prop.top_part_inertia.x(), 0, 0,
		0, _prop.top_part_inertia.y(), 0,
		0, 0, _prop.top_part_inertia.z()
	);
	dyn_button_top->setMassProperties(_prop.top_part_mass, button_inertia, cVector3d(0.0, 0.0, 0.0));
	// - create collision mesh from graphics mesh
	auto mesh_coll_top = _mesh_button_top->copy(true, true, true, true);
	auto mmesh_coll_top = new cMultiMesh(); mmesh_coll_top->addMesh(mesh_coll_top);
	dyn_button_top->setCollisionModel(mmesh_coll_top);
	dyn_button_top->buildCollisionHull(0.0004, 0.0004); // set 0.0004 meters as mesh conversion error
	// - create button joint about the y-axis passing through the center of the top part
	_joint_button = dyn_button_top->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Y);
	_joint_button->m_name = "j0";
	_joint_button->setPos(0.0);
	// - add top link to base.
	dyn_button_bottom->linkChild(dyn_button_top, _prop.top_part_relpos, cIdentity3d());

	// enable dynamics
	_dyn_button->enableDynamics(true);

	// set button to Off
	setState(ToggleButton::Off);

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
	ret_state = (_joint_button->getPos() > 0.75*_joint_extent)? ToggleButton::On: ToggleButton::Off;
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
	double curr_pos = _joint_button->getPos();
	double set_pos = (curr_pos < 0.0)? -_joint_extent: _joint_extent;
	_joint_button->setForce(-_prop.k_spring*(curr_pos - set_pos) - _prop.k_damper*_joint_button->getVel());
}

void ToggleButton::setState(State state) {
	if (state == ToggleButton::On) {
		setAngle(1.0);
	} else {
		setAngle(-1.0);
	}
}

// set button angle in range [-1.0, 1.0]. -1.0 is off, 1.0 is on.
void ToggleButton::setAngle(double angle) {
	_dyn_button->enableDynamics(false);
	_joint_button->setPos(_joint_extent * angle);
	_dyn_button->enableDynamics(true);
}

// get location in world
Eigen::Vector3d ToggleButton::getPosInWorld() const {
	return _world_pose.translation();
}

// get orientation in world
Eigen::Matrix3d ToggleButton::getRotInWorld() const {
	return _world_pose.linear();
}

/* ToggleLever.cpp */

#include <cmath>
#include "widgets/ToggleLever.h"

using namespace Eigen;
using namespace chai3d;

ToggleLever::ToggleLever(
	const ToggleLeverProperties& properties,
	const std::string& name,
	const Eigen::Vector3d& world_pos,
	const Eigen::Matrix3d& world_ori,
	const Simulation::Sai2Simulation* simulation
) {
	// copy properties
	_prop = properties;

	// call default constructor
	ToggleLever(name, world_pos, world_ori, simulation);
}

ToggleLever::ToggleLever(
	const std::string& name,
	const Eigen::Vector3d& world_pos,
	const Eigen::Matrix3d& world_ori,
	const Simulation::Sai2Simulation* simulation
) {
	// copy world position
	_world_pose.translation() = world_pos;
	_world_pose.linear() = world_ori;

	// compute joint extent. lever top part rotates about the lever y axis.
	double temp1 = 0.5 * sqrt(pow(_prop.top_part_extents.x(), 2) + pow(_prop.top_part_extents.z(), 2));
	double temp2 = asin((_prop.top_part_relpos.z() - _prop.bottom_part_extents.z())/temp1);
	double temp3 = atan(_prop.top_part_extents.z()/_prop.top_part_extents.x());
	_joint_extent = 0.95 * (temp2 - temp3);

	// save name
	m_name = name;

	// create graphics
	_mesh_lever = new cMultiMesh();
	_mesh_lever->m_name = "graphics";
	// - create and add graphic for bottom part
	auto mesh_lever_bottom = new cMesh();
	mesh_lever_bottom->m_name = "bottom";
	cCreateBox(mesh_lever_bottom, _prop.bottom_part_extents.x(), _prop.bottom_part_extents.y(), _prop.bottom_part_extents.z());
	mesh_lever_bottom->m_material->setColor(_prop.bottom_part_color);
	mesh_lever_bottom->setLocalPos(cVector3d(0.0,0.0,0.5*_prop.bottom_part_extents.z()));
	//^offset z by half the height of the base part
	_mesh_lever->addMesh(mesh_lever_bottom);
	// - create and add graphic for top part
	_mesh_lever_top = new cMesh();
	_mesh_lever_top->m_name = "top";
	cCreateCylinder(_mesh_lever_top, _prop.top_part_extents.x(), _prop.top_part_extents.z());
	_mesh_lever_top->m_material->setColor(_prop.top_part_color);
	_mesh_lever_top->setLocalPos(_prop.top_part_relpos);
	_mesh_lever->addMesh(_mesh_lever_top);
	addChild(_mesh_lever);

	// create dynamics
	// - create a new base object in the simulation world
	_dyn_lever = simulation->_world->newBaseObject(world_pos, world_ori);
	_dyn_lever->m_name = name;
	// - create a default type material to apply
	auto dyn_mat = new cDynamicMaterial();
	// - create bottom link
	auto dyn_lever_bottom = _dyn_lever->newLink(dyn_mat);
	dyn_lever_bottom->m_name = "bottom";
	dyn_lever_bottom->setMassProperties(0.01, cIdentity3d(), cVector3d(0.0, 0.0, 0.0));
	//^this is a fixed link, so these are just good default mass properties
	// - create collision mesh from graphics mesh
	auto mesh_coll_bottom = mesh_lever_bottom->copy(true, true, true, true);
	auto mmesh_coll_bottom = new cMultiMesh();
	mmesh_coll_bottom->addMesh(mesh_coll_bottom);
	mmesh_coll_bottom->setLocalPos(cVector3d(0.0,0.0,0.5*_prop.bottom_part_extents.z()));
	//^offset the mesh by half the height of the bottom part
	dyn_lever_bottom->setCollisionModel(mmesh_coll_bottom);
	dyn_lever_bottom->buildCollisionHull(0.0004, 0.0004); // set 0.0004 meters as mesh conversion error
	// - add bottom link to base. position it at the center
	_dyn_lever->linkChild(dyn_lever_bottom, cVector3d(0.0, 0.0, 0.0), cIdentity3d());
	// - create top link with same material
	auto dyn_lever_top = _dyn_lever->newLink(dyn_mat);
	dyn_lever_top->m_name = "top";
	// - set mass properties
	cMatrix3d lever_inertia(
		_prop.top_part_inertia.x(), 0, 0,
		0, _prop.top_part_inertia.y(), 0,
		0, 0, _prop.top_part_inertia.z()
	);
	dyn_lever_top->setMassProperties(_prop.top_part_mass, lever_inertia, cVector3d(0.0, 0.0, 0.0));
	// - create collision mesh from graphics mesh
	auto mesh_coll_top = _mesh_lever_top->copy(true, true, true, true);
	auto mmesh_coll_top = new cMultiMesh(); mmesh_coll_top->addMesh(mesh_coll_top);
	dyn_lever_top->setCollisionModel(mmesh_coll_top);
	dyn_lever_top->buildCollisionHull(0.0004, 0.0004); // set 0.0004 meters as mesh conversion error
	// - create lever joint about the y-axis passing through the center of the bottom part
	_joint_lever = dyn_lever_top->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Y);
	_joint_lever->m_name = "j0";
	_joint_lever->setPos(0.0);
	// - add top link to base.
	dyn_lever_bottom->linkChild(dyn_lever_top, _prop.top_part_relpos, cIdentity3d());

	// enable dynamics
	_dyn_lever->enableDynamics(true);

	// set lever to Off
	setState(ToggleLever::Off);

	// set local position of the Lever in the graphic world
	setLocalPos(world_pos);
	setLocalRot(world_ori);
}

ToggleLever::~ToggleLever() {
	delete _mesh_lever; //auto deletes child meshes
	delete _dyn_lever; // this does not auto delete links and joints, oh well.
}

ToggleLever::State ToggleLever::getState() const {
	ToggleLever::State ret_state;
	ret_state = (_joint_lever->getPos() > 0.75*_joint_extent)? ToggleLever::On: ToggleLever::Off;
	return ret_state;
}

void ToggleLever::updateGraphics() {
	// get joint angle
	double joint_val = _joint_lever->getPos();

	// compute orientation
	cMatrix3d rot(cVector3d(0.0, 1.0, 0.0), joint_val);
	_mesh_lever_top->setLocalRot(rot);
}

void ToggleLever::updateDynamics() {
	// compute lever torque and set torque
	double curr_pos = _joint_lever->getPos();
	double set_pos = (curr_pos < 0.0)? -_joint_extent: _joint_extent;
	_joint_lever->setForce(-_prop.k_spring*(curr_pos - set_pos) - _prop.k_damper*_joint_lever->getVel());
}

void ToggleLever::setState(State state) {
	if (state == ToggleLever::On) {
		setAngle(1.0);
	} else {
		setAngle(-1.0);
	}
}

// set lever angle in range [-1.0, 1.0]. -1.0 is off, 1.0 is on.
void ToggleLever::setAngle(double angle) {
	_dyn_lever->enableDynamics(false);
	_joint_lever->setPos(_joint_extent * angle);
	_dyn_lever->enableDynamics(true);
}

// get location in world
Eigen::Vector3d ToggleLever::getPosInWorld() const {
	return _world_pose.translation();
}

// get orientation in world
Eigen::Matrix3d ToggleLever::getRotInWorld() const {
	return _world_pose.linear();
}

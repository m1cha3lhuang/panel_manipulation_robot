/* ToggleValve.cpp */

#include <cmath>
#include "widgets/ToggleValve.h"

using namespace Eigen;
using namespace chai3d;

ToggleValve::ToggleValve(
	const ToggleValveProperties& properties,
	const std::string& name,
	const Eigen::Vector3d& world_pos,
	const Eigen::Matrix3d& world_ori,
	const Simulation::Sai2Simulation* simulation
) {
	// copy properties
	_prop = properties;

	// call default constructor
	ToggleValve(name, world_pos, world_ori, simulation);
}

ToggleValve::ToggleValve(
	const std::string& name,
	const Eigen::Vector3d& world_pos,
	const Eigen::Matrix3d& world_ori,
	const Simulation::Sai2Simulation* simulation
) {
	// copy world position
	_world_pose.translation() = world_pos;
	_world_pose.linear() = world_ori;

	// compute joint extent. valve top part rotates about the valve y axis.
	double temp1 = 0.5 * sqrt(pow(_prop.top_part_extents.x(), 2) + pow(_prop.top_part_extents.z(), 2));
	double temp2 = asin((_prop.top_part_relpos.z() - _prop.bottom_part_extents.z())/temp1);
	double temp3 = atan(_prop.top_part_extents.z()/_prop.top_part_extents.x());
	_joint_extent = 0.95 * (temp2 - temp3);

	// save name
	m_name = name;

	// create graphics
	_mesh_valve = new cMultiMesh();
	_mesh_valve->m_name = "graphics";
	// - create and add graphic for bottom part
	auto mesh_valve_bottom = new cMesh();
	mesh_valve_bottom->m_name = "bottom";
	cCreateBox(mesh_valve_bottom, _prop.bottom_part_extents.x(), _prop.bottom_part_extents.y(), _prop.bottom_part_extents.z());
	mesh_valve_bottom->m_material->setColor(_prop.bottom_part_color);
	mesh_valve_bottom->setLocalPos(cVector3d(0.0,0.0,0.5*_prop.bottom_part_extents.z()));
	//^offset z by half the height of the base part
	_mesh_valve->addMesh(mesh_valve_bottom);
	// - create and add graphic for top part
	_mesh_valve_top = new cMesh();
	_mesh_valve_top->m_name = "top";
	cCreateBox(_mesh_valve_top, _prop.top_part_extents.x(), _prop.top_part_extents.y(), _prop.top_part_extents.z());
	_mesh_valve_top->m_material->setColor(_prop.top_part_color);
	_mesh_valve_top->setLocalPos(_prop.top_part_relpos);
	_mesh_valve->addMesh(_mesh_valve_top);
	addChild(_mesh_valve);

	// create dynamics
	// - create a new base object in the simulation world
	_dyn_valve = simulation->_world->newBaseObject(world_pos, world_ori);
	_dyn_valve->m_name = name;
	// - create a default type material to apply
	auto dyn_mat = new cDynamicMaterial();
	// - create bottom link
	auto dyn_valve_bottom = _dyn_valve->newLink(dyn_mat);
	dyn_valve_bottom->m_name = "bottom";
	dyn_valve_bottom->setMassProperties(0.01, cIdentity3d(), cVector3d(0.0, 0.0, 0.0));
	//^this is a fixed link, so these are just good default mass properties
	// - create collision mesh from graphics mesh
	auto mesh_coll_bottom = mesh_valve_bottom->copy(true, true, true, true);
	auto mmesh_coll_bottom = new cMultiMesh();
    mmesh_coll_bottom->addMesh(mesh_coll_bottom);
    mmesh_coll_bottom->setLocalPos(cVector3d(0.0,0.0,0.5*_prop.bottom_part_extents.z()));
	//^offset the mesh by half the height of the bottom part
	dyn_valve_bottom->setCollisionModel(mmesh_coll_bottom);
	dyn_valve_bottom->buildCollisionHull(0.0004, 0.0004); // set 0.0004 meters as mesh conversion error
	// - add bottom link to base. position it at the center
	_dyn_valve->linkChild(dyn_valve_bottom, cVector3d(0.0, 0.0, 0.0), cIdentity3d());
	// - create top link with same material
	auto dyn_valve_top = _dyn_valve->newLink(dyn_mat);
	dyn_valve_top->m_name = "top";
	// - set mass properties
	cMatrix3d valve_inertia(
		_prop.top_part_inertia.x(), 0, 0,
		0, _prop.top_part_inertia.y(), 0,
		0, 0, _prop.top_part_inertia.z()
	);
	dyn_valve_top->setMassProperties(_prop.top_part_mass, valve_inertia, cVector3d(0.0, 0.0, 0.0));
	// - create collision mesh from graphics mesh
    auto mesh_coll_top = _mesh_valve_top->copy(true, true, true, true);
    auto mmesh_coll_top = new cMultiMesh(); mmesh_coll_top->addMesh(mesh_coll_top);
    dyn_valve_top->setCollisionModel(mmesh_coll_top);
    dyn_valve_top->buildCollisionHull(0.0004, 0.0004); // set 0.0004 meters as mesh conversion error
	// - create valve joint about the z-axis passing through the center of the bottom part
	_joint_valve = dyn_valve_top->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Z);
	_joint_valve->m_name = "j0";
	_joint_valve->setPos(0.0);
	// - add top link to base.
	dyn_valve_bottom->linkChild(dyn_valve_top, _prop.top_part_relpos, cIdentity3d());

	// enable dynamics
	_dyn_valve->enableDynamics(true);

	// set valve to Off
	setState(ToggleValve::Off);

	// set local position of the valve in the graphic world
	setLocalPos(world_pos);
	setLocalRot(world_ori);
}

ToggleValve::~ToggleValve() {
	delete _mesh_valve; //auto deletes child meshes
	delete _dyn_valve; // this does not auto delete links and joints, oh well.
}

ToggleValve::State ToggleValve::getState() const {
	ToggleValve::State ret_state;
	ret_state = (_joint_valve->getPos() > 0.75*_joint_extent)? ToggleValve::On: ToggleValve::Off;
	return ret_state;
}

void ToggleValve::updateGraphics() {
	// get joint angle
	double joint_val = _joint_valve->getPos();

	// compute orientation
	cMatrix3d rot(cVector3d(0.0, 0.0, 1.0), joint_val);
	_mesh_valve_top->setLocalRot(rot);
}

void ToggleValve::updateDynamics() {
	// compute valve torque and set torque
	double curr_pos = _joint_valve->getPos();
	double set_pos = (curr_pos < 0.0)? -_joint_extent: _joint_extent;
	_joint_valve->setForce(-_prop.k_spring*(curr_pos - set_pos) - _prop.k_damper*_joint_valve->getVel());
}

void ToggleValve::setState(State state) {
	if (state == ToggleValve::On) {
		setAngle(1.0);
	} else {
		setAngle(-1.0);
	}
}

// set valve angle in range [-1.0, 1.0]. -1.0 is off, 1.0 is on.
void ToggleValve::setAngle(double angle) {
	_dyn_valve->enableDynamics(false);
	_joint_valve->setPos(_joint_extent * angle);
	_dyn_valve->enableDynamics(true);
}

// get location in world
Eigen::Vector3d ToggleValve::getPosInWorld() const {
	return _world_pose.translation();
}

// get orientation in world
Eigen::Matrix3d ToggleValve::getRotInWorld() const {
	return _world_pose.linear();
}

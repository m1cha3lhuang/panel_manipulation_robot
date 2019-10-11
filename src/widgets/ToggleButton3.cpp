/* ToggleButton3.cpp */

#include <cmath>
#include "widgets/ToggleButton3.h"

using namespace Eigen;
using namespace chai3d;

ToggleButton3::ToggleButton3(
	const ToggleButton3Properties& properties,
	const std::string& name,
	const Eigen::Vector3d& world_pos,
	const Eigen::Matrix3d& world_ori,
	const Simulation::Sai2Simulation* simulation
) {
	// copy properties
	_prop = properties;

	// call default constructor
	ToggleButton3(name, world_pos, world_ori, simulation);
}

ToggleButton3::ToggleButton3(
	const std::string& name,
	const Eigen::Vector3d& world_pos,
	const Eigen::Matrix3d& world_ori,
	const Simulation::Sai2Simulation* simulation
) {
	// copy world position
	_world_pose.translation() = world_pos;
	_world_pose.linear() = world_ori;

	// compute joint extent. button3 top part rotates about the button3 y axis.
	double temp1 = 0.5 * sqrt(pow(_prop.top_part_extents.x(), 2) + pow(_prop.top_part_extents.z(), 2));
	double temp2 = asin((_prop.top_part_relpos.z() - _prop.bottom_part_extents.z())/temp1);
	double temp3 = atan(_prop.top_part_extents.z()/_prop.top_part_extents.x());
	_joint_extent = 0.95 * (temp2 - temp3);

	// save name
	m_name = name;

	// create graphics
	_mesh_button3 = new cMultiMesh();
	_mesh_button3->m_name = "graphics";
	// - create and add graphic for bottom part
	auto mesh_button3_bottom = new cMesh();
	mesh_button3_bottom->m_name = "bottom";
	cCreateBox(mesh_button3_bottom, _prop.bottom_part_extents.x(), _prop.bottom_part_extents.y(), _prop.bottom_part_extents.z());
	mesh_button3_bottom->m_material->setColor(_prop.bottom_part_color);
	mesh_button3_bottom->setLocalPos(cVector3d(0.0,0.0,0.5*_prop.bottom_part_extents.z()));
	//^offset z by half the height of the base part
	_mesh_button3->addMesh(mesh_button3_bottom);
	// - create and add graphic for top part
	_mesh_button3_top = new cMesh();
	_mesh_button3_top->m_name = "top";
	cCreateBox(_mesh_button3_top, _prop.top_part_extents.x(), _prop.top_part_extents.y(), _prop.top_part_extents.z());
	_mesh_button3_top->m_material->setColor(_prop.top_part_color);
	_mesh_button3_top->setLocalPos(_prop.top_part_relpos);
	_mesh_button3->addMesh(_mesh_button3_top);
	addChild(_mesh_button3);

	// create dynamics
	// - create a new base object in the simulation world
	_dyn_button3 = simulation->_world->newBaseObject(world_pos, world_ori);
	_dyn_button3->m_name = name;
	// - create a default type material to apply
	auto dyn_mat = new cDynamicMaterial();
	// - create bottom link
	auto dyn_button3_bottom = _dyn_button3->newLink(dyn_mat);
	dyn_button3_bottom->m_name = "bottom";
	dyn_button3_bottom->setMassProperties(0.01, cIdentity3d(), cVector3d(0.0, 0.0, 0.0));
	//^this is a fixed link, so these are just good default mass properties
	// - create collision mesh from graphics mesh
	auto mesh_coll_bottom = mesh_button3_bottom->copy(true, true, true, true);
	auto mmesh_coll_bottom = new cMultiMesh();
    mmesh_coll_bottom->addMesh(mesh_coll_bottom);
    mmesh_coll_bottom->setLocalPos(cVector3d(0.0,0.0,0.5*_prop.bottom_part_extents.z()));
	//^offset the mesh by half the height of the bottom part
	dyn_button3_bottom->setCollisionModel(mmesh_coll_bottom);
	dyn_button3_bottom->buildCollisionHull(0.0004, 0.0004); // set 0.0004 meters as mesh conversion error
	// - add bottom link to base. position it at the center
	_dyn_button3->linkChild(dyn_button3_bottom, cVector3d(0.0, 0.0, 0.0), cIdentity3d());
	// - create top link with same material
	auto dyn_button3_top = _dyn_button3->newLink(dyn_mat);
	dyn_button3_top->m_name = "top";
	// - set mass properties
	cMatrix3d button3_inertia(
		_prop.top_part_inertia.x(), 0, 0,
		0, _prop.top_part_inertia.y(), 0,
		0, 0, _prop.top_part_inertia.z()
	);
	dyn_button3_top->setMassProperties(_prop.top_part_mass, button3_inertia, cVector3d(0.0, 0.0, 0.0));
	// - create collision mesh from graphics mesh
    auto mesh_coll_top = _mesh_button3_top->copy(true, true, true, true);
    auto mmesh_coll_top = new cMultiMesh(); mmesh_coll_top->addMesh(mesh_coll_top);
    dyn_button3_top->setCollisionModel(mmesh_coll_top);
    dyn_button3_top->buildCollisionHull(0.0004, 0.0004); // set 0.0004 meters as mesh conversion error
	// - create button3 joint about the y-axis passing through the center of the top part
	_joint_button3 = dyn_button3_top->newJoint(DYN_JOINT_REVOLUTE, DYN_AXIS_Z);
	_joint_button3->m_name = "j0";
	_joint_button3->setPos(0.0);
	// - add top link to base.
	dyn_button3_bottom->linkChild(dyn_button3_top, _prop.top_part_relpos, cIdentity3d());

	// enable dynamics
	_dyn_button3->enableDynamics(true);

	// set button3 to Off
	setState(ToggleButton3::Off);

	// set local position of the button3 in the graphic world
	setLocalPos(world_pos);
	setLocalRot(world_ori);
}

ToggleButton3::~ToggleButton3() {
	delete _mesh_button3; //auto deletes child meshes
	delete _dyn_button3; // this does not auto delete links and joints, oh well.
}

ToggleButton3::State ToggleButton3::getState() const {
	ToggleButton3::State ret_state;
	ret_state = (_joint_button3->getPos() > 0.75*_joint_extent)? ToggleButton3::On: ToggleButton3::Off;
	return ret_state;
}

void ToggleButton3::updateGraphics() {
	// get joint angle
	double joint_val = _joint_button3->getPos();

	// compute orientation
	cMatrix3d rot(cVector3d(0.0, 0.0, 1.0), joint_val);
	_mesh_button3_top->setLocalRot(rot);
}

void ToggleButton3::updateDynamics() {
	// compute button3 torque and set torque
	double curr_pos = _joint_button3->getPos();
	double set_pos = (curr_pos < 0.0)? -_joint_extent: _joint_extent;
	_joint_button3->setForce(-_prop.k_spring*(curr_pos - set_pos) - _prop.k_damper*_joint_button3->getVel());
}

void ToggleButton3::setState(State state) {
	if (state == ToggleButton3::On) {
		setAngle(1.0);
	} else {
		setAngle(-1.0);
	}
}

// set button3 angle in range [-1.0, 1.0]. -1.0 is off, 1.0 is on.
void ToggleButton3::setAngle(double angle) {
	_dyn_button3->enableDynamics(false);
	_joint_button3->setPos(_joint_extent * angle);
	_dyn_button3->enableDynamics(true);
}

// get location in world
Eigen::Vector3d ToggleButton3::getPosInWorld() const {
	return _world_pose.translation();
}

// get orientation in world
Eigen::Matrix3d ToggleButton3::getRotInWorld() const {
	return _world_pose.linear();
}

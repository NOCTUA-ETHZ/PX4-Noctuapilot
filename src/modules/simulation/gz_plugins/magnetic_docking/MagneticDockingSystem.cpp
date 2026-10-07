/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MagneticDockingSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <sdf/Element.hh>

#include <algorithm>
#include <cmath>
#include <stdexcept>

using custom::MagneticDockingSystem;

GZ_ADD_PLUGIN(
	MagneticDockingSystem,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(MagneticDockingSystem, "custom::MagneticDockingSystem")

void MagneticDockingSystem::Configure(const gz::sim::Entity &entity,
				      const std::shared_ptr<const sdf::Element> &sdf,
				      gz::sim::EntityComponentManager &ecm,
				      gz::sim::EventManager &event_mgr)
{
	(void)event_mgr;

	_world = gz::sim::World(entity);

	if (!_world.Valid(ecm)) {
		throw std::runtime_error("MagneticDockingSystem must be attached to a world");
	}

	const char *required_elements[] = {"model_a", "link_a", "model_b", "link_b"};

	for (const char *element : required_elements) {
		if (!sdf->HasElement(element)) {
			throw std::runtime_error(std::string("MagneticDockingSystem is missing required <") + element + "> element");
		}
	}

	_magnet_a.model_name = sdf->Get<std::string>("model_a");
	_magnet_a.link_name = sdf->Get<std::string>("link_a");
	_magnet_b.model_name = sdf->Get<std::string>("model_b");
	_magnet_b.link_name = sdf->Get<std::string>("link_b");

	if (sdf->HasElement("offset_a")) {
		_magnet_a.offset = sdf->Get<gz::math::Vector3d>("offset_a");
	}

	if (sdf->HasElement("offset_b")) {
		_magnet_b.offset = sdf->Get<gz::math::Vector3d>("offset_b");
	}

	if (sdf->HasElement("axis_a")) {
		_magnet_a.axis = sdf->Get<gz::math::Vector3d>("axis_a");
	}

	if (sdf->HasElement("axis_b")) {
		_magnet_b.axis = sdf->Get<gz::math::Vector3d>("axis_b");
	}

	if (sdf->HasElement("capture_distance")) {
		_capture_distance = sdf->Get<double>("capture_distance");
	}

	if (sdf->HasElement("magnet_radius")) {
		_magnet_radius = sdf->Get<double>("magnet_radius");
	}

	if (sdf->HasElement("magnet_length")) {
		_magnet_length = sdf->Get<double>("magnet_length");
	}

	if (sdf->HasElement("remanence")) {
		_remanence = sdf->Get<double>("remanence");
	}

	if (sdf->HasElement("force_scale")) {
		_force_scale = sdf->Get<double>("force_scale");
	}

	if (sdf->HasElement("damping")) {
		_damping = sdf->Get<double>("damping");
	}

	if (sdf->HasElement("enabled")) {
		_enabled.store(sdf->Get<bool>("enabled"));
	}

	if (sdf->HasElement("control_topic")) {
		_control_topic = sdf->Get<std::string>("control_topic");
	}

	if (_capture_distance <= 0.0) {
		throw std::runtime_error("MagneticDockingSystem <capture_distance> must be greater than zero");
	}

	if (_magnet_radius <= 0.0 || _magnet_length <= 0.0) {
		throw std::runtime_error("MagneticDockingSystem magnet dimensions must be greater than zero");
	}

	if (_remanence < 0.0 || _force_scale < 0.0) {
		throw std::runtime_error("MagneticDockingSystem remanence and force scale must not be negative");
	}

	if (_damping < 0.0) {
		throw std::runtime_error("MagneticDockingSystem <damping> must not be negative");
	}

	if (_magnet_a.axis.Length() <= 1e-9 || _magnet_b.axis.Length() <= 1e-9) {
		throw std::runtime_error("MagneticDockingSystem magnet axes must not be zero vectors");
	}

	_magnet_a.axis.Normalize();
	_magnet_b.axis.Normalize();
	_contact_force = AttractionForce(0.0);

	if (!_node.Subscribe(_control_topic, &MagneticDockingSystem::EnableCallback, this)) {
		throw std::runtime_error("MagneticDockingSystem failed to subscribe to control topic " + _control_topic);
	}

	ResolveLinks(ecm);

	gzmsg << "MagneticDockingSystem configured for "
	      << _magnet_a.model_name << "::" << _magnet_a.link_name << " and "
	      << _magnet_b.model_name << "::" << _magnet_b.link_name
	      << " (Zurek cylindrical model: radius " << _magnet_radius << " m, length "
	      << _magnet_length << " m, remanence " << _remanence << " T, contact force "
	      << _contact_force << " N, damping " << _damping << " N s/m, initially "
	      << (_enabled.load() ? "enabled" : "disabled") << ")" << std::endl;
	gzmsg << "MagneticDockingSystem control topic: " << _control_topic << std::endl;
}

double MagneticDockingSystem::AttractionForce(double surface_gap) const
{
	// Zurek (2022), Eq. (6): finite-at-contact closed-form approximation
	// for two identical, coaxial, axially magnetized cylindrical magnets.
	constexpr double permeability_free_space = 4.0 * M_PI * 1e-7;
	const double corrected_gap = std::max(0.0, surface_gap) + 0.8 * _magnet_radius;
	const double middle_distance = corrected_gap + _magnet_length;
	const double far_distance = corrected_gap + 2.0 * _magnet_length;
	const double geometric_term = 1.0 / (corrected_gap * corrected_gap)
				      + 1.0 / (far_distance * far_distance)
				      - 2.0 / (middle_distance * middle_distance);
	const double prefactor = M_PI * _remanence * _remanence
				 * std::pow(_magnet_radius, 4.0) / (4.0 * permeability_free_space);
	return _force_scale * prefactor * geometric_term;
}

void MagneticDockingSystem::EnableCallback(const gz::msgs::Boolean &message)
{
	const bool enabled = message.data();
	const bool was_enabled = _enabled.exchange(enabled);

	if (enabled != was_enabled) {
		gzmsg << "MagneticDockingSystem " << (enabled ? "enabled" : "disabled") << std::endl;
	}
}

bool MagneticDockingSystem::ResolveEndpoint(MagnetEndpoint &endpoint,
		gz::sim::EntityComponentManager &ecm)
{
	const gz::sim::Entity model_entity = _world.ModelByName(ecm, endpoint.model_name);

	if (model_entity == gz::sim::kNullEntity) {
		return false;
	}

	const gz::sim::Model model(model_entity);
	const gz::sim::Entity link_entity = model.LinkByName(ecm, endpoint.link_name);

	if (link_entity == gz::sim::kNullEntity) {
		return false;
	}

	endpoint.link = gz::sim::Link(link_entity);
	endpoint.link.EnableVelocityChecks(ecm, true);
	return true;
}

bool MagneticDockingSystem::ResolveLinks(gz::sim::EntityComponentManager &ecm)
{
	const bool resolved_a = ResolveEndpoint(_magnet_a, ecm);
	const bool resolved_b = ResolveEndpoint(_magnet_b, ecm);
	_links_resolved = resolved_a && resolved_b;

	if (_links_resolved && _magnet_a.link.Entity() == _magnet_b.link.Entity()) {
		gzerr << "MagneticDockingSystem endpoints resolve to the same link; disabling plugin" << std::endl;
		_configuration_valid = false;
		_links_resolved = false;
	}

	if (!_links_resolved && _configuration_valid && !_missing_links_reported) {
		gzwarn << "MagneticDockingSystem is waiting for models and links "
		       << _magnet_a.model_name << "::" << _magnet_a.link_name << " and "
		       << _magnet_b.model_name << "::" << _magnet_b.link_name << std::endl;
		_missing_links_reported = true;
	}

	return _links_resolved;
}

void MagneticDockingSystem::PreUpdate(const gz::sim::UpdateInfo &info,
				      gz::sim::EntityComponentManager &ecm)
{
	if (!_configuration_valid) {
		return;
	}

	if (!_enabled.load()) {
		return;
	}

	if (!_links_resolved || !_magnet_a.link.Valid(ecm) || !_magnet_b.link.Valid(ecm)) {
		_links_resolved = false;

		if (!ResolveLinks(ecm)) {
			return;
		}
	}

	if (info.paused) {
		return;
	}

	const auto pose_a = _magnet_a.link.WorldPose(ecm);
	const auto pose_b = _magnet_b.link.WorldPose(ecm);
	const auto velocity_a = _magnet_a.link.WorldLinearVelocity(ecm, _magnet_a.offset);
	const auto velocity_b = _magnet_b.link.WorldLinearVelocity(ecm, _magnet_b.offset);

	if (!pose_a || !pose_b || !velocity_a || !velocity_b) {
		return;
	}

	const gz::math::Vector3d point_a = pose_a->Pos() + pose_a->Rot().RotateVector(_magnet_a.offset);
	const gz::math::Vector3d point_b = pose_b->Pos() + pose_b->Rot().RotateVector(_magnet_b.offset);
	const gz::math::Vector3d separation = point_b - point_a;
	const double distance = separation.Length();

	if (distance >= _capture_distance) {
		return;
	}

	const gz::math::Vector3d axis_a = pose_a->Rot().RotateVector(_magnet_a.axis);
	const gz::math::Vector3d axis_b = pose_b->Rot().RotateVector(_magnet_b.axis);
	const double alignment = std::max(0.0, axis_a.Dot(axis_b));

	if (alignment <= 1e-6) {
		return;
	}

	gz::math::Vector3d direction;

	if (distance > 1e-9) {
		direction = separation / distance;

	} else {
		// The face points coincide at contact. Their cylinder centres still
		// define the attraction direction and avoid a zero-force singularity.
		const gz::math::Vector3d center_a = point_a + 0.5 * _magnet_length * axis_a;
		const gz::math::Vector3d center_b = point_b - 0.5 * _magnet_length * axis_b;
		direction = (center_b - center_a).Normalized();
	}

	const double attraction = AttractionForce(distance);
	const double damping_weight = _contact_force > 0.0 ? attraction / _contact_force : 0.0;
	const double relative_axial_velocity = (*velocity_b - *velocity_a).Dot(direction);
	const double force_magnitude = alignment * std::max(0.0,
				       attraction + _damping * damping_weight * relative_axial_velocity);
	const gz::math::Vector3d force_on_a = force_magnitude * direction;

	// Apply the forces at the link-local magnet offsets. Gazebo therefore
	// produces moments from off-centre forces without an artificial torque.
	_magnet_a.link.AddWorldForce(ecm, force_on_a, _magnet_a.offset);
	_magnet_b.link.AddWorldForce(ecm, -force_on_a, _magnet_b.offset);
}

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

#pragma once

#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/transport/Node.hh>

#include <atomic>
#include <memory>
#include <string>

namespace custom
{

class MagneticDockingSystem final :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &event_mgr) final;

	void PreUpdate(const gz::sim::UpdateInfo &info,
		       gz::sim::EntityComponentManager &ecm) final;

private:
	struct MagnetEndpoint {
		std::string model_name;
		std::string link_name;
		gz::math::Vector3d offset{0.0, 0.0, 0.0};
		gz::math::Vector3d axis{0.0, 0.0, 1.0};
		gz::sim::Link link{gz::sim::kNullEntity};
	};

	bool ResolveLinks(gz::sim::EntityComponentManager &ecm);
	bool ResolveEndpoint(MagnetEndpoint &endpoint,
			     gz::sim::EntityComponentManager &ecm);
	double AttractionForce(double surface_gap) const;
	void EnableCallback(const gz::msgs::Boolean &message);

	gz::sim::World _world{gz::sim::kNullEntity};
	gz::transport::Node _node;
	MagnetEndpoint _magnet_a;
	MagnetEndpoint _magnet_b;

	std::string _control_topic{"/magnetic_docking/enable"};
	std::atomic<bool> _enabled{true};
	double _capture_distance{0.15};
	double _magnet_radius{0.015};
	double _magnet_length{0.005};
	double _remanence{0.8};
	double _force_scale{1.0};
	double _damping{3.0};
	double _contact_force{0.0};

	bool _links_resolved{false};
	bool _configuration_valid{true};
	bool _missing_links_reported{false};
};

} // namespace custom

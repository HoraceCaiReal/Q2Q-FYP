// Explicit kinematic approximation: hold commanded world pose and zero momentum.
// No joint, suction force, or friction grasp is simulated. A wall lease releases
// the model if the relay dies. Only the experimental 50 mm world loads this.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <chrono>
#include <mutex>

namespace q2q {
class KinematicCarry : public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate {
  using Steady = std::chrono::steady_clock;
  ignition::gazebo::Entity model{};
  ignition::transport::Node node;
  std::mutex mutex;
  ignition::math::Pose3d pose;
  bool active{false}, applied{false};
  Steady::time_point received;
 public:
  void Configure(const ignition::gazebo::Entity &entity,
      const std::shared_ptr<const sdf::Element> &,
      ignition::gazebo::EntityComponentManager &,
      ignition::gazebo::EventManager &) override {
    model = entity;
    node.Advertise("/q2q/kinematic_carry/set_pose", &KinematicCarry::Command, this);
  }
  bool Command(const ignition::msgs::Pose &request, ignition::msgs::Boolean &reply) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!request.name().empty() && request.name() != "pick_bin") {
      reply.set_data(false); return true;
    }
    active = !request.name().empty();
    pose = ignition::math::Pose3d(request.position().x(), request.position().y(),
                                 request.position().z(), 0., 0., 0.);
    received = Steady::now();
    reply.set_data(true);
    return true;
  }
  void PreUpdate(const ignition::gazebo::UpdateInfo &,
                 ignition::gazebo::EntityComponentManager &ecm) override {
    std::lock_guard<std::mutex> lock(mutex);
    namespace c = ignition::gazebo::components;
    if (active && Steady::now()-received < std::chrono::milliseconds(500)) {
      ecm.SetComponentData<c::WorldPoseCmd>(model, pose);
      ecm.SetComponentData<c::LinearVelocityCmd>(model, ignition::math::Vector3d::Zero);
      ecm.SetComponentData<c::AngularVelocityCmd>(model, ignition::math::Vector3d::Zero);
      applied = true;
    } else if (applied) {
      ecm.RemoveComponent<c::WorldPoseCmd>(model);
      ecm.RemoveComponent<c::LinearVelocityCmd>(model);
      ecm.RemoveComponent<c::AngularVelocityCmd>(model);
      applied = false;
    }
  }
};
}
IGNITION_ADD_PLUGIN(q2q::KinematicCarry, ignition::gazebo::System,
    q2q::KinematicCarry::ISystemConfigure, q2q::KinematicCarry::ISystemPreUpdate)

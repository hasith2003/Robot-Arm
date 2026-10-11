// Fixed-joint grasp abstraction: no attachment until the task explicitly asks.
#include <atomic>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>

namespace gp7_grasp {
class ValidatedGrasp : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
  gz::transport::Node node;
  gz::transport::Node::Publisher status;
  gz::sim::Entity parent{gz::sim::kNullEntity}, joint{gz::sim::kNullEntity};
  std::atomic<bool> attach{false}, detach{false};
  unsigned ticks{0};
  void Publish() {
    gz::msgs::StringMsg msg;
    msg.set_data(joint == gz::sim::kNullEntity ? "detached" : "attached");
    status.Publish(msg);
  }
public:
  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    parent = gz::sim::Model(entity).LinkByName(ecm, "tool0");
    node.Subscribe("/box_b/attach", &ValidatedGrasp::Attach, this);
    node.Subscribe("/box_b/detach", &ValidatedGrasp::Detach, this);
    status = node.Advertise<gz::msgs::StringMsg>("/box_b/grasp_state");
    Publish();
  }
  void Attach(const gz::msgs::Empty &) { attach.store(true); }
  void Detach(const gz::msgs::Empty &) { detach.store(true); }
  void PreUpdate(const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    if (detach.exchange(false)) {
      attach.store(false);
      if (joint != gz::sim::kNullEntity) ecm.RequestRemoveEntity(joint);
      joint = gz::sim::kNullEntity;
      Publish();
    }
    if (attach.exchange(false) && joint == gz::sim::kNullEntity && parent != gz::sim::kNullEntity) {
      auto model = ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name("box_b"));
      auto child = ecm.EntityByComponents(gz::sim::components::Link(),
        gz::sim::components::ParentEntity(model), gz::sim::components::Name("link"));
      if (child != gz::sim::kNullEntity) {
        joint = ecm.CreateEntity();
        ecm.CreateComponent(joint, gz::sim::components::DetachableJoint({parent, child, "fixed"}));
        Publish();
      }
    }
    if (++ticks % 100 == 0) Publish();
  }
};
}
GZ_ADD_PLUGIN(gp7_grasp::ValidatedGrasp, gz::sim::System,
  gp7_grasp::ValidatedGrasp::ISystemConfigure, gp7_grasp::ValidatedGrasp::ISystemPreUpdate)

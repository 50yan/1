#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

namespace gazebo {
// One server-side simulation clock drives every light and both renderers.
class TrafficLightSync : public WorldPlugin {
 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    red_ = sdf->Get<double>("red_seconds");
    green_ = sdf->Get<double>("green_seconds");
    yellow_ = sdf->Get<double>("yellow_seconds");
    material_uri_ = sdf->Get<std::string>("material_uri");
    if (red_ <= 0 || green_ <= 0 || yellow_ <= 0) {
      gzerr << "[TrafficLightSync] Durations must be positive.\n";
      return;
    }
    for (auto target = sdf->GetElement("visual"); target;
         target = target->GetNextElement("visual")) {
      names_.push_back(target->Get<std::string>());
    }
    node_.reset(new transport::Node());
    node_->Init(world_->Name());
    publisher_ = node_->Advertise<msgs::Visual>("~/visual", 100);
    connection_ = event::Events::ConnectWorldUpdateEnd(
        std::bind(&TrafficLightSync::Update, this));
    gzmsg << "[TrafficLightSync] red=" << red_ << " green=" << green_
          << " yellow=" << yellow_ << " (simulation seconds); visuals="
          << names_.size() << "\n";
  }

 private:
  void Update() {
    const double now = world_->SimTime().Double();
    const double phase = std::fmod(now, red_ + green_ + yellow_);
    const std::string state = phase < red_ ? "Red" :
        (phase < red_ + green_ ? "Green" : "Yellow");
    // Refresh for renderers that connect late, as well as at every transition.
    if (state == last_state_ && now >= last_publish_ &&
        now - last_publish_ < 0.2) return;
    for (const auto &name : names_) {
      const auto split_visual = name.rfind("::");
      const auto split_link = name.rfind("::", split_visual - 1);
      if (split_visual == std::string::npos || split_link == std::string::npos) continue;
      const std::string model_name = name.substr(0, split_link);
      const std::string link_name = name.substr(split_link + 2,
                                                 split_visual - split_link - 2);
      const std::string visual_name = name.substr(split_visual + 2);
      auto model = world_->ModelByName(model_name);
      auto link = model ? model->GetLink(link_name) : physics::LinkPtr();
      if (!link) continue;
      uint32_t id = 0;
      bool found = false;
      for (const auto &entry : link->Visuals()) {
        if (entry.second.name() == name || entry.second.name() == visual_name) {
          id = entry.first;
          found = true;
          break;
        }
      }
      if (!found) continue;
      msgs::Visual visual;
      visual.set_name(name);
      visual.set_id(id);
      visual.set_parent_name(model_name + "::" + link_name);
      visual.set_parent_id(link->GetId());
      auto *material = visual.mutable_material();
      material->set_lighting(false);
      auto *script = material->mutable_script();
      script->set_name("Ceshi3Traffic/" + state);
      script->add_uri(material_uri_);
      publisher_->Publish(visual);
    }
    if (state != last_state_) {
      gzmsg << "[TrafficLightSync] sim_time=" << now << " state="
            << state << "\n";
    }
    last_state_ = state;
    last_publish_ = now;
  }
  physics::WorldPtr world_;
  transport::NodePtr node_;
  transport::PublisherPtr publisher_;
  event::ConnectionPtr connection_;
  std::vector<std::string> names_;
  std::string material_uri_, last_state_;
  double red_ = 10.0, green_ = 15.0, yellow_ = 5.0;
  double last_publish_ = -1.0;
};
GZ_REGISTER_WORLD_PLUGIN(TrafficLightSync)
}  // namespace gazebo

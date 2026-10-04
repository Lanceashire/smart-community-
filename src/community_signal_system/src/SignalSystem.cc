#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

class CommunitySignalSystem : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate {
  struct Lamp {gz::sim::Entity entity; gz::math::Pose3d pose; std::string colour; bool on;};
  std::vector<Lamp> lamps;
  gz::transport::Node transport;
  gz::transport::Node::Publisher health;
  std::string wanted;
  double red=10.,green=15.,yellow=3.,requestedAt=0.,lastPublish=-1.;
  unsigned updates=0,applied=0,mismatches=0;
  double maxDelay=0.;
 public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override {
    if(sdf->HasElement("red_seconds"))red=sdf->Get<double>("red_seconds");
    if(sdf->HasElement("green_seconds"))green=sdf->Get<double>("green_seconds");
    if(sdf->HasElement("yellow_seconds"))yellow=sdf->Get<double>("yellow_seconds");
    health=transport.Advertise<gz::msgs::StringMsg>("/simulation/traffic_light_engine");
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if(lamps.empty())ecm.Each<gz::sim::components::Model,gz::sim::components::Name,gz::sim::components::Pose>(
      [&](const gz::sim::Entity &e,const auto *,const auto *name,const auto *pose){
        const std::string n=name->Data();
        if(n.rfind("light_top_",0)==0 || n.rfind("light_bottom_",0)==0){
          const bool on=n.size()>3 && n.substr(n.size()-3)=="_on";
          const bool off=n.size()>4 && n.substr(n.size()-4)=="_off";
          if(on || off)for(const auto &c:{std::string("red"),std::string("green"),std::string("yellow")})
            if(n.find("_"+c+"_")!=std::string::npos)lamps.push_back({e,pose->Data(),c,on});
        }
        return true;
      });
    if(info.paused || lamps.size()!=12)return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    const double phase=std::fmod(now,red+green+yellow);
    const std::string next=phase<red?"red":phase<red+green?"green":"yellow";
    if(next==wanted)return;
    wanted=next;requestedAt=now;++updates;
    // All twelve commands are created in one simulation update, with no
    // network discovery, external process or service acknowledgement involved.
    for(const auto &lamp:lamps){
      auto pose=lamp.pose;
      pose.Pos().Z((lamp.on==(lamp.colour==wanted))?.35:-1.);
      gz::sim::Model(lamp.entity).SetWorldPoseCmd(ecm,pose);
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
    if(info.paused)return;
    const double now=std::chrono::duration<double>(info.simTime).count();
    bool matching=lamps.size()==12 && !wanted.empty();
    for(const auto &lamp:lamps){
      const auto *pose=ecm.Component<gz::sim::components::Pose>(lamp.entity);
      const double expected=(lamp.on==(lamp.colour==wanted))?.35:-1.;
      matching=matching && pose && std::abs(pose->Data().Pos().Z()-expected)<.002;
    }
    if(matching && applied<updates){++applied;maxDelay=std::max(maxDelay,now-requestedAt);}
    if(!matching && now-requestedAt>.01)++mismatches;
    if(now-lastPublish<.1)return;
    lastPublish=now;
    std::ostringstream json;
    json<<"{\"engine\":\"Gazebo simulation update\",\"updates\":"<<updates
        <<",\"applied\":"<<applied<<",\"entities\":"<<lamps.size()
        <<",\"phase\":\""<<wanted<<"\",\"poses_match\":"<<(matching?"true":"false")
        <<",\"mismatch_iterations\":"<<mismatches<<",\"max_apply_delay_sim_ms\":"<<maxDelay*1000<<"}";
    gz::msgs::StringMsg message;message.set_data(json.str());health.Publish(message);
  }
};
GZ_ADD_PLUGIN(CommunitySignalSystem,gz::sim::System,gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(CommunitySignalSystem,"community::SignalSystem")

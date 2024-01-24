#ifndef HOVER_HOVER_BT_H
#define HOVER_HOVER_BT_H

#include <behaviortree_cpp/bt_factory.h>

namespace hover
{

class HoverSettingsLoader: public BT::SyncActionNode
{
public:
  HoverSettingsLoader(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class HoverUpdate: public BT::SyncActionNode
{
public:
  HoverUpdate(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

class HoverCommand: public BT::StatefulActionNode
{
public:
  HoverCommand(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};


} // namespace hover

#endif

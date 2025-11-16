# BehaviorTree.CPP Node Class Reference (ROS 2 Compatible)

This document provides an overview of available BehaviorTree.CPP node classes,
their characteristics, and templates for custom node implementation.

---

## 🟩 1. BT::ConditionNode
**Purpose:** Checks a condition and returns instantly (`SUCCESS` or `FAILURE`).

**Use case:** For checks like “Is goal reached?” or “Is object detected?”

```cpp
#include <behaviortree_cpp/action_node.h>

class IsGoalReached : public BT::ConditionNode
{
public:
  IsGoalReached(const std::string &name, const BT::NodeConfiguration &config)
      : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("distance") };
  }

  BT::NodeStatus tick() override
  {
    double dist;
    getInput("distance", dist);
    return (dist < 0.5) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
```

---

## 🟦 2. BT::SyncActionNode
**Purpose:** Performs a short, synchronous action in one tick.

**Use case:** For quick side effects — like publishing a message or toggling a flag.

```cpp
#include <behaviortree_cpp/action_node.h>

class ToggleLight : public BT::SyncActionNode
{
public:
  ToggleLight(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(rclcpp::get_logger("BT"), "Toggling light.");
    return BT::NodeStatus::SUCCESS;
  }
};
```

---

## 🟨 3. BT::StatefulActionNode
**Purpose:** Handles actions that run across multiple ticks or phases.

**Use case:** Long-running or asynchronous actions like moving, spinning, or waiting.

```cpp
#include <behaviortree_cpp/action_node.h>

class MoveBase : public BT::StatefulActionNode
{
public:
  MoveBase(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override
  {
    RCLCPP_INFO(rclcpp::get_logger("BT"), "Starting movement.");
    startTime_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if ((std::chrono::steady_clock::now() - startTime_) > 5s)
      return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_INFO(rclcpp::get_logger("BT"), "Movement halted.");
  }

private:
  std::chrono::steady_clock::time_point startTime_;
};
```

---

## 🟧 4. BT::AsyncActionNode
**Purpose:** Runs a task asynchronously in another thread.

**Use case:** For heavy computation or blocking I/O that must not block the tree tick.

```cpp
#include <behaviortree_cpp/action_node.h>
#include <future>

class ComputePathAsync : public BT::AsyncActionNode
{
public:
  ComputePathAsync(const std::string &name, const BT::NodeConfiguration &config)
      : BT::AsyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    if (!future_.valid())
    {
      future_ = std::async(std::launch::async, []() {
        std::this_thread::sleep_for(std::chrono::seconds(3));
        return true;
      });
      return BT::NodeStatus::RUNNING;
    }

    if (future_.wait_for(0s) == std::future_status::ready)
      return BT::NodeStatus::SUCCESS;

    return BT::NodeStatus::RUNNING;
  }

private:
  std::future<bool> future_;
};
```

---

## 🟥 5. BT::CoroutineActionNode
**Purpose:** Allows pausing/resuming action logic between ticks (`yield`).

**Use case:** Step-by-step actions that need to pause mid-process.

```cpp
#include <behaviortree_cpp/action_node.h>

class WaitCoroutine : public BT::CoroutineActionNode
{
public:
  WaitCoroutine(const std::string &name, const BT::NodeConfiguration &config)
      : BT::CoroutineActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    for (int i = 0; i < 5; ++i)
    {
      RCLCPP_INFO(rclcpp::get_logger("BT"), "Waiting... %d", i);
      setStatusRunningAndYield();  // yield for one tick
    }
    return BT::NodeStatus::SUCCESS;
  }
};
```

---

## 🟪 6. BT::DecoratorNode
**Purpose:** Wraps another node and modifies its return behavior.

**Use case:** To repeat, invert, or filter child node results.

```cpp
#include <behaviortree_cpp/decorator_node.h>

class Inverter : public BT::DecoratorNode
{
public:
  Inverter(const std::string &name, const BT::NodeConfiguration &config)
      : BT::DecoratorNode(name, config) {}

  BT::NodeStatus tick() override
  {
    const BT::NodeStatus child_status = child_node_->executeTick();
    if (child_status == BT::NodeStatus::SUCCESS)
      return BT::NodeStatus::FAILURE;
    if (child_status == BT::NodeStatus::FAILURE)
      return BT::NodeStatus::SUCCESS;
    return BT::NodeStatus::RUNNING;
  }
};
```

---

## 🟫 7. BT::ControlNode
**Purpose:** Base for logic-flow nodes (Sequence, Fallback, Parallel).

**Use case:** Rarely used directly — it’s the parent class for control structures.

```cpp
#include <behaviortree_cpp/control_node.h>

class CustomSequence : public BT::ControlNode
{
public:
  CustomSequence(const std::string &name)
      : BT::ControlNode(name, {}) {}

  BT::NodeStatus tick() override
  {
    for (auto &child : children_nodes_)
    {
      const BT::NodeStatus status = child->executeTick();
      if (status != BT::NodeStatus::SUCCESS)
        return status;
    }
    return BT::NodeStatus::SUCCESS;
  }
};
```

---

## 🟨 8. Predefined Control Nodes (no subclassing needed)
| Node | Behavior |
|------|-----------|
| **Sequence** | Runs children in order until one fails. |
| **Fallback** | Runs children until one succeeds. |
| **Parallel** | Runs all children; returns when success/failure threshold reached. |
| **ReactiveSequence / ReactiveFallback** | Like Sequence/Fallback but restarts evaluation each tick. |

---

## 🧭 Summary Table

| Node Type | Multi-Tick | Async | Implements | Use For |
|------------|-------------|--------|-------------|----------|
| ConditionNode | ❌ | ❌ | tick() | Checks and predicates |
| SyncActionNode | ❌ | ❌ | tick() | Quick side effects |
| StatefulActionNode | ✅ | ❌ | onStart(), onRunning(), onHalted() | Long or async tasks |
| AsyncActionNode | ✅ | ✅ | tick() | Threaded heavy tasks |
| CoroutineActionNode | ✅ | ❌ | tick() with yield | Step-by-step actions |
| DecoratorNode | depends | ❌ | tick() | Modify child behavior |
| ControlNode | ✅ | ❌ | tick() | Manage multiple children |

---

## 🧩 Quick Selection Guide
| Goal | Recommended Base Class |
|------|------------------------|
| Check condition | `ConditionNode` |
| One-shot command | `SyncActionNode` |
| Multi-step process | `StatefulActionNode` |
| Heavy background task | `AsyncActionNode` |
| Stepwise yield logic | `CoroutineActionNode` |
| Change child outcome | `DecoratorNode` |
| Custom control logic | `ControlNode` |

---

**Author:** BehaviorTree.CPP Quick Reference for ROS2 Developers  
**License:** MIT  
**Version:** 1.0

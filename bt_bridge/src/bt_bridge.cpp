// Thin pybind11 bridge exposing real BehaviorTree.CPP (+ its Groot2Publisher) to Python. See
// docs/behavior-tree-migration.md for the full design and the Phase 0 spike that validated this
// builds/links/ticks against Isaac Sim's own bundled Python 3.11 and that Groot2Publisher's ZMQ
// port is reachable.
//
// Design principle carried over from docs/omnigraph-migration-plan.md's cuRobo-hosting verdict:
// this bridge owns control flow only (SUCCESS/FAILURE/RUNNING sequencing), never the actual
// grasp/place math -- every leaf node's tick() is a callback into existing Python
// (scripts/mefron_lib/behavior_tree.py), which calls straight into grasp.py/teleop.py. And
// BTExecutor::tick_once() is meant to be called synchronously, once per frame, from Python's
// existing _step_arm() loop -- never from a separate C++ thread -- so there is no background
// thread ever touching CUDA/PhysX/cuRobo state, the same risk category that plan already rejected
// for Script-Node-hosted cuRobo.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace py = pybind11;

namespace
{
// Process-wide registry from an XML node's `callback_name` attribute to the Python callable it
// invokes on tick(). Registered once per name via register_callback() (see behavior_tree.py),
// looked up on every tick. Guarded by a mutex defensively -- in the intended single-threaded,
// one-tick-per-frame usage there is never real contention, since Groot2Publisher's own background
// ZMQ thread only reads tree/blackboard state, it never calls back into Python.
std::mutex g_registry_mutex;
std::unordered_map<std::string, py::function> g_callback_registry;

BT::NodeStatus StatusFromString(const std::string& callback_name, const std::string& value)
{
  if (value == "SUCCESS")
  {
    return BT::NodeStatus::SUCCESS;
  }
  if (value == "FAILURE")
  {
    return BT::NodeStatus::FAILURE;
  }
  if (value == "RUNNING")
  {
    return BT::NodeStatus::RUNNING;
  }
  throw BT::RuntimeError("Python callback '" + callback_name + "' returned unrecognized status '" + value + "'");
}
}  // namespace

// One generic leaf node type, registered under both "PyAction" and "PyCondition" XML tags (see
// assembly_placement.xml): reads its `callback_name` input port, looks up the matching Python
// callable, calls it with no arguments, and maps its returned status string to a BT::NodeStatus.
// Deliberately a single reusable type instead of one C++ class per behavior -- every real leaf
// (IsHoldingSomething, SnapToLiftWaypoint, WaitForPlanIdle, SnapToFinalPose) is authored in Python
// and wired in purely via the XML + register_callback(), not by adding new C++ node types.
// BT::ActionNodeBase, not BT::SyncActionNode: SyncActionNode's executeTick() throws if tick()
// ever returns RUNNING, but WaitForPlanIdle (see assembly_placement.xml) legitimately needs to
// return RUNNING across many frames while state["cmd_plan"] is still in flight -- confirmed live,
// SyncActionNode raised "MUST never return RUNNING" the moment a wait callback returned it.
// ActionNodeBase places no such restriction on tick()'s return value.
class PyActionNode : public BT::ActionNodeBase
{
public:
  PyActionNode(const std::string& name, const BT::NodeConfig& config) : BT::ActionNodeBase(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("callback_name")};
  }

  BT::NodeStatus tick() override
  {
    std::string callback_name;
    if (!getInput("callback_name", callback_name))
    {
      throw BT::RuntimeError("PyAction/PyCondition node '" + name() + "' is missing its required 'callback_name' input");
    }

    py::function callback;
    {
      std::lock_guard<std::mutex> lock(g_registry_mutex);
      auto it = g_callback_registry.find(callback_name);
      if (it == g_callback_registry.end())
      {
        throw BT::RuntimeError("No Python callback registered for '" + callback_name + "' -- call bt_bridge.register_callback() first");
      }
      callback = it->second;
    }

    // The GIL is already held here: tick_once() below is an ordinary pybind11-wrapped call, and
    // pybind11 keeps the GIL held for the duration of any C++ call from Python unless explicitly
    // released -- which BTExecutor::tick_once() deliberately never does, since this bridge never
    // ticks off the calling Python thread.
    std::string result;
    try
    {
      result = callback().cast<std::string>();
    }
    catch (const py::error_already_set& e)
    {
      throw BT::RuntimeError("Python callback '" + callback_name + "' raised: " + std::string(e.what()));
    }
    return StatusFromString(callback_name, result);
  }

  // No coroutine-style suspension to unwind: RUNNING here just means "call the same Python
  // callback again next tick," so there's no per-node async state to tear down on halt.
  void halt() override
  {
    resetStatus();
  }
};

class BTExecutor
{
public:
  BTExecutor(const std::string& xml_path, int groot2_port)
  {
    factory_.registerNodeType<PyActionNode>("PyAction");
    factory_.registerNodeType<PyActionNode>("PyCondition");
    tree_ = factory_.createTreeFromFile(xml_path);
    publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, static_cast<unsigned>(groot2_port));
  }

  std::string tick_once()
  {
    return BT::toStr(tree_.tickOnce());
  }

  void halt()
  {
    tree_.haltTree();
  }

private:
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::unique_ptr<BT::Groot2Publisher> publisher_;
};

static void register_callback(const std::string& name, py::function callback)
{
  std::lock_guard<std::mutex> lock(g_registry_mutex);
  g_callback_registry[name] = std::move(callback);
}

// Named mefron_bt_bridge, not bt_bridge -- this repo's top-level bt_bridge/ source directory has
// no __init__.py, so it's an implicit Python 3 namespace package. Confirmed live: with the
// compiled module also named "bt_bridge", `import bt_bridge` from a script run with the repo root
// on sys.path (Kit adds CWD) silently resolved to that empty namespace package instead of this
// extension -- `AttributeError: module 'bt_bridge' has no attribute 'BTExecutor'`, no import error
// at all. A distinct module name sidesteps the collision regardless of sys.path ordering.
PYBIND11_MODULE(mefron_bt_bridge, m)
{
  m.doc() = "pybind11 bridge onto real BehaviorTree.CPP + Groot2Publisher for the mefron assembly-placement tree";

  m.def(
      "register_callback",
      &register_callback,
      py::arg("name"),
      py::arg("callback"),
      "Registers a zero-argument Python callable (returning 'SUCCESS'/'FAILURE'/'RUNNING') under "
      "the given name, for lookup by any XML leaf node whose callback_name input matches it.");

  py::class_<BTExecutor>(m, "BTExecutor")
      .def(py::init<const std::string&, int>(), py::arg("xml_path"), py::arg("groot2_port") = 1667)
      .def("tick_once", &BTExecutor::tick_once, "Ticks the tree once; returns 'SUCCESS'/'FAILURE'/'RUNNING'.")
      .def("halt", &BTExecutor::halt, "Halts every running node in the tree (e.g. on a fresh Play/Stop reset).");
}

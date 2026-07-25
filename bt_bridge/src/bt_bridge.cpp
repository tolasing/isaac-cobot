// Thin pybind11 bridge exposing real BehaviorTree.CPP (+ its Groot2Publisher) to Python. See
// docs/behavior-tree-migration.md for the full design and the Phase 0 spike that validated this
// builds/links/ticks against Isaac Sim's own bundled Python 3.11 and that Groot2Publisher's ZMQ
// port is reachable.
//
// Design principle carried over from docs/omnigraph-migration-plan.md's cuRobo-hosting verdict:
// this bridge owns control flow AND per-object parameter plumbing only, never the actual
// grasp/place math -- every leaf node's tick() is a callback into existing Python
// (scripts/mefron_lib/behavior_tree.py), which calls straight into grasp.py/teleop.py. And
// BTExecutor::tick_once() is meant to be called synchronously, once per frame, from Python's
// existing _step_arm() loop -- never from a separate C++ thread -- so there is no background
// thread ever touching CUDA/PhysX/cuRobo state, the same risk category that plan already rejected
// for Script-Node-hosted cuRobo.
//
// Ports (added for the generalized grasp/place port): PyActionNode declares a fixed, generously-
// named set of OPTIONAL input ports covering every parameter either generated tree
// (bt_bridge/trees/generated/*.xml, see behavior_tree.py's generate_*_tree_xml()) needs on any of
// its leaves. Only the ports actually present on a given XML tag get resolved and forwarded to the
// Python callback, as a dict -- so one generic node type still covers every leaf, per-object data
// just arrives as real port/blackboard values now instead of being baked into a Python closure.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/xml_parsing.h>

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

// Every optional *string* port PyActionNode declares, beyond callback_name -- a superset across
// both generated trees (grasp and placement). See generate_grasp_tree_xml()/
// generate_placement_tree_xml() in behavior_tree.py for which of these each leaf actually sets.
const std::vector<std::string>& OptionalStringPortNames()
{
  static const std::vector<std::string> names = {
      "object_name", "requested_object", "relationship_name", "requested_relationship",
      "pose_source", "yaml_path",        "grasp_name",        "part_prim_path", "end_effector_kind",
  };
  return names;
}

// Numeric ports, declared as `double` (not `std::string`) -- confirmed live: BT.CPP's XML parser
// auto-infers a `double` port type for any <SubTree> remapping attribute whose literal value
// parses as a number (e.g. delay_seconds="2.0"), independent of whatever type the referenced
// leaf's own C++ port declares, and then fails tree construction on the mismatch
// ("port was initially created with type [double] and, later type [std::string] was used").
// Declaring these as double here matches that inference; tick() converts back to a Python str so
// callbacks still see a uniform dict[str, str] regardless of a port's underlying BT.CPP type.
const std::vector<std::string>& OptionalDoublePortNames()
{
  static const std::vector<std::string> names = {"position_tolerance", "orientation_tolerance", "delay_seconds"};
  return names;
}

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

// One generic leaf node type, registered under both "PyAction" and "PyCondition" XML tags: reads
// its `callback_name` input port plus whichever of OptionalPortNames() are actually set on this
// XML tag, looks up the matching Python callable, calls it with a dict of the resolved ports, and
// maps its returned status string to a BT::NodeStatus. Deliberately a single reusable type instead
// of one C++ class per behavior -- every real leaf (SnapToObjectPose, WaitForHandAtTarget,
// IsHoldingSomething, ...) is authored in Python and wired in purely via generated XML +
// register_callback(), not by adding new C++ node types.
// BT::ActionNodeBase, not BT::SyncActionNode: SyncActionNode's executeTick() throws if tick()
// ever returns RUNNING, but e.g. WaitForHandAtTarget legitimately needs to return RUNNING across
// many frames while the arm is still in flight -- confirmed live, SyncActionNode raised "MUST
// never return RUNNING" the moment a wait callback returned it. ActionNodeBase places no such
// restriction on tick()'s return value.
class PyActionNode : public BT::ActionNodeBase
{
public:
  PyActionNode(const std::string& name, const BT::NodeConfig& config) : BT::ActionNodeBase(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports = {BT::InputPort<std::string>("callback_name")};
    for (const auto& name : OptionalStringPortNames())
    {
      ports.insert(BT::InputPort<std::string>(name));
    }
    for (const auto& name : OptionalDoublePortNames())
    {
      ports.insert(BT::InputPort<double>(name));
    }
    return ports;
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

    py::dict ports;
    for (const auto& port_name : OptionalStringPortNames())
    {
      std::string value;
      if (getInput(port_name, value))
      {
        ports[py::str(port_name)] = value;
      }
    }
    for (const auto& port_name : OptionalDoublePortNames())
    {
      double value;
      if (getInput(port_name, value))
      {
        ports[py::str(port_name)] = std::to_string(value);
      }
    }

    // The GIL is already held here: tick_once() below is an ordinary pybind11-wrapped call, and
    // pybind11 keeps the GIL held for the duration of any C++ call from Python unless explicitly
    // released -- which BTExecutor::tick_once() deliberately never does, since this bridge never
    // ticks off the calling Python thread.
    std::string result;
    try
    {
      result = callback(ports).cast<std::string>();
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

  // Writes a string value onto the tree's root blackboard -- the one thing Python still pushes
  // into the tree per request (e.g. "requested_object" = "finger_print_scanner"), so the
  // generated MainTree's per-object Fallback branches can compare their own literal object_name
  // port against it via {requested_object} remapping. Everything else per-object flows in as
  // literal port values baked into the generated XML itself, not via the blackboard.
  void set_blackboard(const std::string& key, const std::string& value)
  {
    tree_.rootBlackboard()->set<std::string>(key, value);
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

// Returns a <TreeNodesModel> XML fragment describing PyAction/PyCondition's real registered
// ports (BT::writeTreeNodesModelXML() -- the same official mechanism BT.CPP itself uses to make
// custom node types self-describing). generate_grasp_tree_xml()/generate_placement_tree_xml()
// embed this into every generated file. Without it, a tool parsing the file standalone (Groot2's
// Editor mode, no live connection to this process) has no way to know PyAction/PyCondition's
// ports and rejects any it doesn't recognize -- confirmed live: "Node 'WaitForHandAtTarget'
// contains port 'orientation_tolerance' in the XML that doesn't match any of the ports in the
// model". A throwaway factory (not the long-lived per-BTExecutor one) is enough -- this only
// reads the static port declarations off the registered node type, no tree/blackboard involved.
static std::string node_models_xml()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<PyActionNode>("PyAction");
  factory.registerNodeType<PyActionNode>("PyCondition");
  return BT::writeTreeNodesModelXML(factory);
}

// Named mefron_bt_bridge, not bt_bridge -- this repo's top-level bt_bridge/ source directory has
// no __init__.py, so it's an implicit Python 3 namespace package. Confirmed live: with the
// compiled module also named "bt_bridge", `import bt_bridge` from a script run with the repo root
// on sys.path (Kit adds CWD) silently resolved to that empty namespace package instead of this
// extension -- `AttributeError: module 'bt_bridge' has no attribute 'BTExecutor'`, no import error
// at all. A distinct module name sidesteps the collision regardless of sys.path ordering.
PYBIND11_MODULE(mefron_bt_bridge, m)
{
  m.doc() = "pybind11 bridge onto real BehaviorTree.CPP + Groot2Publisher for the mefron grasp/place trees";

  m.def(
      "register_callback",
      &register_callback,
      py::arg("name"),
      py::arg("callback"),
      "Registers a Python callable (dict[str, str] -> 'SUCCESS'/'FAILURE'/'RUNNING') under the "
      "given name, for lookup by any XML leaf node whose callback_name input matches it. The dict "
      "contains whichever of PyActionNode's optional ports are actually set on that leaf's XML tag.");

  m.def(
      "node_models_xml",
      &node_models_xml,
      "Returns a <TreeNodesModel> XML fragment describing PyAction/PyCondition's real ports, for "
      "embedding into generated tree files so external tools (Groot2) can parse/validate them "
      "without a live connection to this process.");

  py::class_<BTExecutor>(m, "BTExecutor")
      .def(py::init<const std::string&, int>(), py::arg("xml_path"), py::arg("groot2_port") = 1667)
      .def("tick_once", &BTExecutor::tick_once, "Ticks the tree once; returns 'SUCCESS'/'FAILURE'/'RUNNING'.")
      .def("halt", &BTExecutor::halt, "Halts every running node in the tree (e.g. on a fresh Play/Stop reset).")
      .def(
          "set_blackboard",
          &BTExecutor::set_blackboard,
          py::arg("key"),
          py::arg("value"),
          "Writes a string value onto the tree's root blackboard.");
}

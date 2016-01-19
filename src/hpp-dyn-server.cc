#include "hpp/corbaserver/server.hh"
#include <hpp/util/pointer.hh>
#include <hpp/dyn/dynPlanner.hh>

// main function of the corba server
int main (int argc, const char* argv[])
{
  // create a ProblemSolver instance.
  // This class is a container that does the interface between hpp-core library
  // and component to be running in a middleware like CORBA or ROS.
  hpp::core::ProblemSolverPtr_t problemSolver =   hpp::core::ProblemSolver::create ();
  // Add the new planner type in order to be able to select it from python
  // client.

  problemSolver->addPathPlannerType ("dyn", hpp::core::DynPlanner::createWithRoadmap);
  // Create the CORBA server.
  hpp::corbaServer::Server server (problemSolver, argc, argv, true);
  // Start the CORBA server.
  server.startCorbaServer ();
  // Wait for CORBA requests.
  server.processRequest (true);
}

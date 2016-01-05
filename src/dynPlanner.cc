#include<hpp/dyn/dynPlanner.hh>


namespace hpp{
  namespace dyn {

    DynPlannerPtr_t DynPlanner::create (const core::Problem& problem,
                                const core::RoadmapPtr_t& roadmap)
    {
      DynPlanner* ptr = new DynPlanner (problem, roadmap);
      DynPlannerPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }


    DynPlanner::DynPlanner (const core::Problem& problem,
             const core::RoadmapPtr_t& roadmap) :
      core::PathPlanner (problem, roadmap),
      shooter_ (core::BasicConfigurationShooter::create (problem.robot ()))
    {
    }
    /// Store weak pointer to itself
    void DynPlanner::init (const DynPlannerWkPtr_t& weak)
    {
      core::PathPlanner::init (weak);
      weakPtr_ = weak;
    }


    /// One step of extension.
    ///
    /// This method implements one step of your algorithm. The method
    /// will be called iteratively until one goal configuration is accessible
    /// from the initial configuration.
    ///
    /// We will see how to implement a basic PRM algorithm.
    void DynPlanner::oneStep ()
    {
      // Retrieve the robot the problem has been defined for.
      model::DevicePtr_t robot (problem ().robot ());
      // Retrieve the path validation algorithm associated to the problem
      core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
      // Retrieve configuration validation methods associated to the problem
      core::ConfigValidationsPtr_t configValidations
        (problem ().configValidations ());
      // Retrieve the steering method
      core::SteeringMethodPtr_t sm (problem ().steeringMethod ());
      // Retrieve the constraints the robot is subject to
      core::ConstraintSetPtr_t constraints (problem ().constraints ());
      // Retrieve roadmap of the path planner
      core::RoadmapPtr_t r (roadmap ());
      // shoot a valid random configuration
      core::ConfigurationPtr_t qrand;
      do {
        qrand = shooter_->shoot ();
      } while (!configValidations->validate (*qrand));
      // Add qrand as a new node
      core::NodePtr_t newNode = r->addNode (qrand);
      // try to connect the random configuration to each connected component
      // of the roadmap.
      for (core::ConnectedComponents_t::const_iterator itcc =
             r->connectedComponents ().begin ();
           itcc != r->connectedComponents ().end (); ++itcc) {
        core::ConnectedComponentPtr_t cc = *itcc;
        // except its own connected component of course
        if (cc != newNode->connectedComponent ()) {
          double d;
          // Get nearest node to qrand in connected component
          core::NodePtr_t nearest = r->nearestNode (qrand, cc, d);
          core::ConfigurationPtr_t qnear = nearest->configuration ();
          // Create local path between qnear and qrand
          core::PathPtr_t localPath = (*sm) (*qnear, *qrand);
          // validate local path
          core::PathPtr_t validPart;
          if (pathValidation->validate (localPath, false, validPart)) {
            // Create node and edges with qrand and the local path
            r->addEdge (nearest, newNode, localPath);
            r->addEdge (newNode, nearest, localPath->reverse ());
          }
        }
      }
    }


  }//namespace dyn
}//namespace hpp

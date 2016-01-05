#include <hpp/util/pointer.hh>

#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>
#include <hpp/core/problem.hh>


namespace hpp {
  namespace dyn {
    // forward declaration of class Planner
    HPP_PREDEF_CLASS (DynPlanner);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <DynPlanner> DynPlannerPtr_t;

    /// Example of path planner
    class DynPlanner : public core::PathPlanner
    {
      public:
        /// Create an instance and return a shared pointer to the instance
        static DynPlannerPtr_t create (const core::Problem& problem,const core::RoadmapPtr_t& roadmap);
        virtual void oneStep ();

      protected:
        /// Protected constructor
        /// Users need to call Planner::create in order to create instances.
        DynPlanner (const core::Problem& problem,const core::RoadmapPtr_t& roadmap);
        /// Store weak pointer to itself
        void init (const DynPlannerWkPtr_t& weak);
      private:
        /// Configuration shooter to uniformly shoot random configurations
        core::BasicConfigurationShooterPtr_t shooter_;
        /// weak pointer to itself
        DynPlannerWkPtr_t weakPtr_;
    }; // class Planner
  } // namespace dyn
} // namespace hpp

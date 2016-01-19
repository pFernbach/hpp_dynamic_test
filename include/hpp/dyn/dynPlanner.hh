#ifndef DYNPLANNER_H
#define DYNPLANNER_H


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
  namespace core {
    // forward declaration of class Planner
    HPP_PREDEF_CLASS (DynPlanner);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <DynPlanner> DynPlannerPtr_t;

    /// Example of path planner
    class DynPlanner : public PathPlanner
    {
      public:
        /// Create an instance and return a shared pointer to the instance
        static DynPlannerPtr_t create (const Problem& problem);
        static DynPlannerPtr_t createWithRoadmap(const Problem& problem, const RoadmapPtr_t& roadmap);
        void configurationShooter (const ConfigurationShooterPtr_t& shooter);


        virtual void oneStep ();

      protected:
        /// Protected constructor
        /// Users need to call Planner::create in order to create instances.
        DynPlanner (const Problem& problem,const RoadmapPtr_t& roadmap);
        DynPlanner (const Problem& problem);

        /// Extend a node in the direction of a configuration
        /// \param near node in the roadmap,
        /// \param target target configuration
        virtual PathPtr_t extend (const NodePtr_t& near,
                                  const ConfigurationPtr_t& target);

        /// Store weak pointer to itself
        void init (const DynPlannerWkPtr_t& weak);
      private:
        /// Configuration shooter to uniformly shoot random configurations
        ConfigurationShooterPtr_t configurationShooter_;
        mutable Configuration_t qProj_;
        /// weak pointer to itself
        DynPlannerWkPtr_t weakPtr_;
        SteeringMethodPtr_t sm_;
    }; // class Planner
  } // namespace dyn
} // namespace hpp



#endif

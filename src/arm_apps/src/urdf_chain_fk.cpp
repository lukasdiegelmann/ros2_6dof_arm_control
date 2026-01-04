#include "arm_apps/urdf_chain_fk.hpp"

#include <stdexcept>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>

namespace arm_apps
{
    void UrdfChainFK::initFromUrdf(const std::string &urdf_xml,
                                   const std::string &base_link,
                                   const std::string &tip_link)
    {
        if (!urdf_model_.initString(urdf_xml))
        {
            throw std::runtime_error("Failed to parse URDF XML into urdf::Model");
        }

        if (!kdl_parser::treeFromString(urdf_xml, tree_))
        {
            throw std::runtime_error("Failed to parse URDF XML into KDL Tree");
        }

        if (!tree_.getChain(base_link, tip_link, chain_))
        {
            throw std::runtime_error("Failed to extract KDL Chain from Tree");
        }

        // Initialize Forward Kinematics Solver
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

        // Extract joint names in chain order
        joint_names_.clear();
        for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
        {
            const auto &segment = chain_.getSegment(i);
            if (segment.getJoint().getType() != KDL::Joint::None)
            {
                joint_names_.push_back(segment.getJoint().getName());
            }
        }
    }

    KDL::Frame UrdfChainFK::fk(const std::vector<double> &q_chain_order) const
    {
        if (!fk_solver_)
        {
            throw std::runtime_error("FK not initialized. Call initFromUrdf first.");
        }

        if (q_chain_order.size() != joint_names_.size())
        {
            throw std::runtime_error("Input joint vector size does not match chain DOF.");
        }

        KDL::JntArray q(chain_.getNrOfJoints());
        for (unsigned int i = 0; i < q.rows(); ++i)
        {

            q(i) = q_chain_order[i];
        }

        KDL::Frame out;
        int rc = fk_solver_->JntToCart(q, out);
        if (rc < 0)
        {
            throw std::runtime_error("KDL FK solver failed");
        }
        return out;
    }

    std::size_t UrdfChainFK::dof() const { return chain_.getNrOfJoints(); }
}
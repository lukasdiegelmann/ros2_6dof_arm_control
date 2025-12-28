#pragma once
#include <string>
#include <vector>
#include <memory>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace arm_apps
{

    class UrdfChainFK
    {
    public:
        UrdfChainFK() = default;

        // Build KDL chain from URDF XML string
        void initFromUrdf(const std::string &urdf_xml,
                          const std::string &base_link,
                          const std::string &tip_link);

        // FK: q must be in chain joint order
        KDL::Frame fk(const std::vector<double> &q_chain_order) const;

        const std::vector<std::string> &jointNames() const { return joint_names_; }
        std::size_t dof() const;

    private:
        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::vector<std::string> joint_names_;
    };

} // namespace arm_apps

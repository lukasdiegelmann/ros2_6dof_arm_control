#pragma once
#include <string>
#include <vector>
#include <memory>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <urdf/model.h>

namespace arm_apps
{

    /**
     * @brief Forward kinematics helper built from a URDF string.
     *
     * This class parses the URDF XML into:
     * - an internal @c urdf::Model (kept for later queries, e.g. joint limits)
     * - a KDL tree/chain for FK computations
     *
     * The joint order exposed by @ref jointNames() is the chain joint order and
     * must match the order of joint vectors passed to @ref fk().
     */
    class UrdfChainFK
    {
    public:
        UrdfChainFK() = default;

        /**
         * @brief Initialize the FK chain from a URDF XML string.
         *
         * Parses the URDF XML, builds a KDL tree and extracts a KDL chain from
         * @p base_link to @p tip_link. Also extracts joint names in chain order.
         *
         * @param urdf_xml URDF as XML string (typically from the ROS parameter robot_description).
         * @param base_link Root link name for the chain.
         * @param tip_link Tip/end-effector link name for the chain.
         *
         * @throws std::runtime_error if the URDF cannot be parsed or the chain cannot be built.
         */
        void initFromUrdf(const std::string &urdf_xml,
                          const std::string &base_link,
                          const std::string &tip_link);

        /**
         * @brief Compute forward kinematics for the given joint vector.
         *
         * @param q_chain_order Joint positions in the exact order returned by @ref jointNames().
         * @return End-effector pose as KDL::Frame.
         *
         * @throws std::runtime_error if called before @ref initFromUrdf() or if sizes mismatch.
         */
        KDL::Frame fk(const std::vector<double> &q_chain_order) const;

        /**
         * @brief Joint names in KDL chain order.
         *
         * This order is the canonical order for joint vectors used by FK/IK.
         */
        const std::vector<std::string> &jointNames() const { return joint_names_; }

        /**
         * @brief Access the parsed URDF model.
         *
         * Useful for querying joint metadata such as limits.
         */
        const urdf::Model &urdfModel() const { return urdf_model_; }

        /**
         * @brief Number of joints (DOF) of the extracted chain.
         */
        std::size_t dof() const;

    private:
        urdf::Model urdf_model_;
        KDL::Tree tree_;
        KDL::Chain chain_;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::vector<std::string> joint_names_;
    };

} // namespace arm_apps

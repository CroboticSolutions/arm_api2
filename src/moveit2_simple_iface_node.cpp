/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Crobotic Solutions d.o.o.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title       : moveit2_simple_iface_node.cpp
 *      Project     : arm_api2
 *      Created     : 05/10/2024
 *      Author      : Filip Zoric
 *
 *      Description : ROS 2 node wrapper for the moveit2_iface.cpp class
 */

#include "arm_api2/moveit2_simple_iface.hpp"

#include <cstdlib>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char * argv [])
{
    /* Cap BLAS/OpenMP threads: parallel collision / OMPL can race MoveIt on small stacks. */
    setenv("OMP_NUM_THREADS", "1", 0);
    setenv("OPENBLAS_NUM_THREADS", "1", 0);
    setenv("VECLIB_MAXIMUM_THREADS", "1", 0);
    setenv("MKL_NUM_THREADS", "1", 0);

    rclcpp::init(argc, argv); 
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_options.use_intra_process_comms(false); 

    auto iface = std::make_shared<m2SimpleIface>(node_options);

    /* Single-threaded: MoveGroupInterface + MoveIt callbacks must not run concurrently (SEGV).
       tryCompletePreviousExecution() is non-blocking so this executor is not starved by sleep. */
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(iface);
    executor.add_node(iface->moveit_ros_node());
    executor.spin();

    rclcpp::shutdown(); 
    return 0; 

}
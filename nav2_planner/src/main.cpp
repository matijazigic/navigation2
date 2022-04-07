// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include "nav2_planner/planner_server.hpp"
#include "rclcpp/rclcpp.hpp"


const std::uint16_t kDefaultMaxLogFileSize { 32 };

void writeFailureToFile(const char* data, int size){
  std::cerr << std::string(data, size) << std::endl;
  LOG(ERROR) << std::string(data, size);
}

int main(int argc, char ** argv)
{
  
  auto default_log_dir = boost::filesystem::path(getenv("HOME")).concat("/planner_server");
  boost::filesystem::create_directory(default_log_dir);

  fLI::FLAGS_max_log_size = kDefaultMaxLogFileSize;
  fLS::FLAGS_log_dir = default_log_dir.c_str();
  fLI::FLAGS_logbuflevel = -1;

  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::InstallFailureWriter(writeFailureToFile);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_planner::PlannerServer>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}

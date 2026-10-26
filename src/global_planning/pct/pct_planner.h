#pragma once
#include "../global_node.h"
#include <memory>
#include <string>
namespace global_planning {
std::unique_ptr<GlobalPlanner> makePctPlanner(const std::string & config_file);
}

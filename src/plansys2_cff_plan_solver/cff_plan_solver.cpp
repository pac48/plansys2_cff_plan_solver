// Copyright 2019 Intelligent Robotics Lab
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

#include <sys/stat.h>
#include <sys/types.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <queue>

#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_core/Utils.hpp"
#include "plansys2_cff_plan_solver/cff_plan_solver.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::TFDPlanSolver, plansys2::PlanSolverBase);

namespace plansys2 {

    unsigned int split_string(const std::string &text, std::vector<std::string> &tokens, char separator) {
        size_t pos = text.find(separator);
        unsigned int initialPos = 0;
        tokens.clear();

        while (pos != std::string::npos && pos < text.length()) {
            if (text.substr(initialPos, pos - initialPos + 1) != " ") {
                std::string s = text.substr(initialPos, pos - initialPos + 1);
                s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(),
                        s.end());
                tokens.push_back(s);
            }
            initialPos = pos + 1;
            pos = text.find(separator, initialPos);
        }

        tokens.push_back(text.substr(initialPos, text.size() - initialPos));
        return tokens.size();
    }

    void extract_elements_from_line(const std::vector<std::string> &tokens, std::string &action_id,
                                    std::string &operator_name,
                                    std::vector<std::string> &operator_params, std::string &true_action_id,
                                    std::string &false_action_id) {

        // Example of actions of Contigent-FF:
        //   17||0 --- FIND_OBJECT C1 ITEM_0 --- SON: 18||0
        //   18||0 --- PICKUP_OBJECT C1 ITEM_0 --- SON: 19||0

        action_id = tokens[0];
        operator_name = tokens[2];

        int idx = 3;
        while (idx < tokens.size() && tokens[idx] != "---") {
            operator_params.push_back(tokens[idx]);
            idx++;
        }
        if (tokens.size() - operator_params.size() == 9) {
            true_action_id = tokens[5 + operator_params.size()];
            false_action_id = tokens[8 + operator_params.size()];
        } else {
            true_action_id = tokens[5 + operator_params.size()];
            false_action_id = true_action_id;
        }
    }

    void createAction(const float &level, std::string operator_name, std::vector<std::string> operator_params,
                      std::string &action) {
        transform(operator_name.begin(), operator_name.end(), operator_name.begin(), ::tolower);
        for (auto &val: operator_params) {
            transform(val.begin(), val.end(), val.begin(), ::tolower);
        }

        std::string operator_parameters;
        for (auto it = operator_params.begin(); it != operator_params.end(); ++it) {
            if (std::next(it) != operator_params.end()) {
                operator_parameters = operator_parameters + (*it) + " ";
            } else {
                operator_parameters = operator_parameters + (*it);
            }
        }
        std::stringstream ss;
        transform(operator_name.begin(), operator_name.end(), operator_name.begin(), ::tolower);
//        ss << level << ": (" << operator_name << " " << operator_parameters << ")" << [" << 1 + level - 0.001 << "]\n";
        ss << "(" << operator_name << " " << operator_parameters << ")";
        action = ss.str();
    }


    TFDPlanSolver::TFDPlanSolver() {
    }

    void
    TFDPlanSolver::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr &node, const std::string &id) {

    }

    std::optional<plansys2_msgs::msg::Plan>
    TFDPlanSolver::getPlan(
            const std::string &domain, const std::string &problem,
            const std::string &node_namespace) {
        if (node_namespace != "") {
            //  This doesn't work as cxx flags must apper at end of link options, and I didn't
            //  find a way
            // std::experimental::filesystem::create_directories("/tmp/" + node_namespace);
            mkdir(("/tmp/" + node_namespace).c_str(), ACCESSPERMS);
        }

        plansys2_msgs::msg::Plan ret;
        std::ofstream domain_out("/tmp/" + node_namespace + "/domain.pddl");
        domain_out << domain;
        domain_out.close();

        std::ofstream problem_out("/tmp/" + node_namespace + "/problem.pddl");
        problem_out << problem;
        problem_out.close();

        system(
                ("ros2 run plansys2_cff_plan_solver FF -o /tmp/" + node_namespace + "/domain.pddl -f /tmp/" +
                 node_namespace + "/problem.pddl > " + "/tmp/" + node_namespace + "/pddlplan.1").c_str()
        );

        std::string line;
        std::ifstream plan_file("/tmp/" + node_namespace + "/pddlplan.1");
        bool solution = false;
        bool is_parse_completed = false;
        float time = 0;
        std::unordered_map<std::string, std::shared_ptr<PlanNode>> tree_map;

        if (plan_file.is_open()) {

            while (std::getline(plan_file, line)) {
                if (line.compare("ff: found plan as follows") == 0) {
                    break;
                }
            }
            if (!line.empty()) {
                solution = true;

                while (!plan_file.eof() && !is_parse_completed) {
                    std::getline(plan_file, line);

                    if (!line.empty()) {
                        if (line.compare("-------------------------------------------------") != 0) {

                            std::vector<std::string> tokens;
                            split_string(line, tokens, ' ');

                            std::string action_id;
                            std::string operator_name;
                            std::vector<std::string> operator_params;
                            std::string true_action_id;
                            std::string false_action_id;
                            extract_elements_from_line(tokens, action_id, operator_name, operator_params,
                                                       true_action_id, false_action_id);

                            plansys2_msgs::msg::PlanItem item;
                            auto level = std::stoi(action_id.substr(0, action_id.find("||")));
                            item.time = level;
                            item.duration = 1.0;
                            createAction(level, operator_name, operator_params, item.action);

                            if (tree_map.find(action_id) == tree_map.end()) {
                                tree_map[action_id] = std::make_shared<PlanNode>();
                            }
                            tree_map[action_id]->item = item;

                            auto tmp = true_action_id.substr(true_action_id.find("||") + 2,
                                                             true_action_id.length() - 1);
                            if (std::stoi(tmp) != -1) {
                                if (tree_map.find(true_action_id) == tree_map.end()) {
                                    tree_map[true_action_id] = std::make_shared<PlanNode>();
                                }
                                tree_map[action_id]->true_node = tree_map[true_action_id];
                            }

                            tmp = false_action_id.substr(false_action_id.find("||") + 2, false_action_id.length() - 1);
                            if (std::stoi(tmp) != -1) {
                                if (tree_map.find(false_action_id) == tree_map.end()) {
                                    tree_map[false_action_id] = std::make_shared<PlanNode>();
                                }
                                tree_map[action_id]->false_node = tree_map[false_action_id];
                            }

                        }
                    } else {
                        is_parse_completed = true;
                    }
                }

            }
            plan_file.close();
        }

        auto root_node = tree_map["0||0"];

        std::vector<plansys2_msgs::msg::PlanItem> data;
        std::vector<int> struc;
        encode_plan(root_node, ret.structure, ret.items);

//        auto root_node_decode = decode_plan(ret.structure, ret.items);

        if (!solution) {
            return {};
        } else {
            return ret;
        }
    }

}  // namespace plansys2

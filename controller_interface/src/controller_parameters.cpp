// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
//
/// \author: Denis Stogl

#include "controller_interface/controller_parameters.hpp"

#include <string>
#include <vector>

#include "rcutils/logging_macros.h"

namespace controller_interface
{
ControllerParameters::ControllerParameters(
  const std::string & params_prefix,
  int nr_bool_params, int nr_double_params, int nr_string_params, int nr_double_array_params,
  int nr_string_list_params) : declared_(false), up_to_date_(false), params_prefix_("")
{
  params_prefix_ = impl::normalize_params_prefix(params_prefix);

  bool_parameters_.reserve(nr_bool_params);
  double_parameters_.reserve(nr_double_params);
  string_parameters_.reserve(nr_string_params);
  double_array_parameters_.reserve(nr_double_array_params);
  string_list_parameters_.reserve(nr_string_list_params);
}

void ControllerParameters::declare_parameters(rclcpp::Node::SharedPtr node)
{
  declare_parameters(node->get_node_logging_interface(), node->get_node_parameters_interface());
}

void ControllerParameters::declare_parameters(
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logger,
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params)
{
  if (!declared_)
  {
    logger_name_ = std::string(node_logger->get_logger_name()) +
      (params_prefix_.empty() ? "" : ("::" + params_prefix_)) +
      "::parameters";
    logging_interface_ = node_logger;
    params_interface_ = node_params;

    declare_parameters_from_list(bool_parameters_);
    declare_parameters_from_list(double_parameters_);
    declare_parameters_from_list(string_parameters_);
    declare_parameters_from_list(double_array_parameters_);
    declare_parameters_from_list(string_list_parameters_);

    declared_ = true;
  }
  else
  {
    RCUTILS_LOG_WARN_NAMED(
      logger_name_.c_str(),
      "Parameters already declared. Declaration should be done only once. "
      "Nothing bad will happen, but please correct your code-logic.");
  }
}

bool ControllerParameters::get_parameters(
  rclcpp::Node::SharedPtr node, bool check_validity, bool update)
{
  if (!declared_)
  {
    declare_parameters(node);
  }

  return get_parameters(check_validity, update);
}

bool ControllerParameters::get_parameters(bool check_validity, bool update)
{
  if (!declared_)
  {
    RCUTILS_LOG_ERROR_NAMED(
      logger_name_.c_str(), "Can not get parameters. Please declare them first.");
    return false;
  }

  bool ret = false;

  // If parameters are updated using dynamic reconfigure callback then there is no need to read
  // them again. To ignore multiple manual reads
  ret = get_parameters_from_list(bool_parameters_) &&
        get_parameters_from_list(double_parameters_) &&
        get_parameters_from_list(string_parameters_) &&
        get_parameters_from_list(double_array_parameters_) &&
        get_parameters_from_list(string_list_parameters_);

  if (ret && check_validity)
  {
    ret = this->check_if_parameters_are_valid();
  }

  // If it is all good until now the parameters are not up to date anymore
  if (ret)
  {
    up_to_date_ = false;
  }

  if (ret && update)
  {
    ret = this->update(false);
  }

  return ret;
}

bool ControllerParameters::update(bool check_validity)
{
  bool ret = true;

  // Let's make this efficient and execute code only if parameters are updated
  if (!up_to_date_)
  {
    if (check_validity)
    {
      ret = this->check_if_parameters_are_valid();
    }

    if (ret)
    {
      this->update_storage();
    }
    else
    {
      RCUTILS_LOG_WARN_NAMED(
        logger_name_.c_str(), "Parameters are not valid and therefore will not be updated");
    }
    // reset variable to update parameters only when this is needed
    up_to_date_ = true;
  }

  return ret;
}



rcl_interfaces::msg::SetParametersResult ControllerParameters::set_parameter_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // TODO(destogl): this is probably executed in another thread --> mutex protection is needed.
  for (const auto & input_parameter : parameters)
  {
    bool found = false;

    try
    {
      found = find_and_assign_parameter_value(bool_parameters_, input_parameter);
      if (!found)
      {
        found = find_and_assign_parameter_value(double_parameters_, input_parameter);
      }
      if (!found)
      {
        found = find_and_assign_parameter_value(string_parameters_, input_parameter);
      }
      if (!found)
      {
        found = find_and_assign_parameter_value(double_array_parameters_, input_parameter);
      }
      if (!found)
      {
        found = find_and_assign_parameter_value(string_list_parameters_, input_parameter);
      }

      RCUTILS_LOG_INFO_EXPRESSION_NAMED(
        found, logger_name_.c_str(),
        "Dynamic parameters got changed! To update the parameters internally please "
        "restart the controller.");
    }
    catch (const rclcpp::exceptions::InvalidParameterTypeException & e)
    {
      result.successful = false;
      result.reason = e.what();
      RCUTILS_LOG_ERROR_NAMED(logger_name_.c_str(), "%s", result.reason.c_str());
      break;
    }
    catch (const std::runtime_error & e)
    {
      result.successful = false;
      result.reason = e.what();
      RCUTILS_LOG_ERROR_NAMED(logger_name_.c_str(), "%s", result.reason.c_str());
      break;
    }
  }

  if (result.successful)
  {
    up_to_date_ = false;
  }

  return result;
}

}  // namespace controller_interface

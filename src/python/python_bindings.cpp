/*
 * Copyright 2019 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <reach/utils.h>
#include <reach_ros/utils.h>
#include <rclcpp/rclcpp.hpp>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <boost/python.hpp>
#include <boost/python/converter/builtin_converters.hpp>
#include <cstdarg>

namespace bp = boost::python;

namespace reach_ros
{

void init_ros(const bp::list& argv)
{
  int argc = bp::len(argv);
  if (argc == 0)
  {
    // init() does not accept argv with length 0
    rclcpp::init(0, nullptr);
  }
  else
  {
    char* argv_c[argc];
    for (bp::ssize_t i = 0; i < argc; ++i)
      argv_c[i] = bp::extract<char*>{ argv[i] }();
    rclcpp::init(argc, argv_c);
  }
}

bp::object get_parameter(std::string name)
{
  rclcpp::Parameter parameter = reach_ros::utils::getNodeInstance()->get_parameter(name);
  rclcpp::ParameterType type = parameter.get_type();
  switch (type)
  {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return bp::object(parameter.as_bool());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return bp::object(parameter.as_int());
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return bp::object(parameter.as_double());
    case rclcpp::ParameterType::PARAMETER_STRING:
      return bp::object(parameter.as_string());
    case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
      return bp::object(parameter.as_byte_array());
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return bp::object(parameter.as_bool_array());
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return bp::object(parameter.as_integer_array());
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return bp::object(parameter.as_double_array());
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return bp::object(parameter.as_string_array());
    default:
      return bp::object();
  }
}

void set_parameter_bool(std::string name, bool value)
{
  reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, value));
}

void set_parameter_integer(std::string name, int value)
{
  reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, value));
}

void set_parameter_double(std::string name, double value)
{
  reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, value));
}

void set_parameter_string(std::string name, std::string value)
{
  reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, value));
}

void set_parameter_byte_array(std::string name, uint8_t* value)
{
  reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, value));
}

void set_parameter_bool_array(std::string name, bool* value)
{
  reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, value));
}

void set_parameter_integer_array(std::string name, int* value)
{
  reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, value));
}

void set_parameter_double_array(std::string name, double* value)
{
  reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, value));
}

void set_parameter_string_array(std::string name, std::string* value)
{
  reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, value));
}

void set_logger_level(std::string logger_name, int level_int)
{
  rclcpp::Logger::Level level;
  switch (level_int)
  {
    case 10:
      level = rclcpp::Logger::Level::Debug;
      break;
    case 20:
      level = rclcpp::Logger::Level::Info;
      break;
    case 30:
      level = rclcpp::Logger::Level::Warn;
      break;
    case 40:
      level = rclcpp::Logger::Level::Error;
      break;
    case 50:
      level = rclcpp::Logger::Level::Fatal;
      break;
    default:
      std::cerr << "Invalid log level: " << level_int << std::endl;
      level = rclcpp::Logger::Level::Unset;
  }
  rclcpp::get_logger(logger_name).set_level(level);
}

BOOST_PYTHON_MODULE(MODULE_NAME)
{
  Py_Initialize();
  {
    bp::def("init_ros", &init_ros);
    bp::def("get_parameter", &get_parameter);
    bp::def("set_parameter", &set_parameter_bool);
    // It is important to define the double option before the integer one.
    // Otherwise, all integers are interpreted as doubles.
    bp::def("set_parameter", &set_parameter_double);
    bp::def("set_parameter", &set_parameter_integer);
    bp::def("set_parameter", &set_parameter_string);
    bp::def("set_parameter", &set_parameter_byte_array);
    bp::def("set_parameter", &set_parameter_bool_array);
    bp::def("set_parameter", &set_parameter_integer_array);
    bp::def("set_parameter", &set_parameter_double_array);
    bp::def("set_parameter", &set_parameter_string_array);
    bp::def("set_logger_level", &set_logger_level);
  }
}

}  // namespace reach_ros

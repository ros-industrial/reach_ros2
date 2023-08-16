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
#include <reach_ros/utils.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <boost/python.hpp>
#include <boost/python/converter/builtin_converters.hpp>
#include <cstdarg>
#include <reach/utils.h>
#include <rclcpp/rclcpp.hpp>

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
      throw std::runtime_error("Unknown parameter type");
  }
}

void set_parameter(std::string name, const bp::object& obj)
{
  // We use the direct checks for object type since bp::extract().check() does not distinguish between int and bool
  if (PyBool_Check(obj.ptr()))
    reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, bp::extract<bool>(obj)));
  else if (PyLong_Check(obj.ptr()))
    reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, bp::extract<int>(obj)));
  else if (PyFloat_Check(obj.ptr()))
    reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, bp::extract<float>(obj)));
  else if (PyUnicode_Check(obj.ptr()))
    reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, bp::extract<std::string>(obj)));
  else if (PyList_Check(obj.ptr()))
  {
    bp::list list_obj = bp::extract<bp::list>(obj);
    if (PyBool_Check(bp::object(list_obj[0]).ptr()))
      reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, bp::extract<bool*>(list_obj)));
    else if (PyLong_Check(bp::object(list_obj[0]).ptr()))
      reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, bp::extract<int*>(list_obj)));
    else if (PyFloat_Check(bp::object(list_obj[0]).ptr()))
      reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, bp::extract<double*>(list_obj)));
    else if (PyUnicode_Check(bp::object(list_obj[0]).ptr()))
      reach_ros::utils::getNodeInstance()->set_parameter(rclcpp::Parameter(name, bp::extract<std::string*>(list_obj)));
  }
  else
    throw std::runtime_error("Unsupported Python value type '" +
                             bp::extract<std::string>{ obj.attr("__class__").attr("__name__") }() + "'");
}

void set_logger_level(std::string logger_name, int level_int)
{
#ifdef ROS2_AT_LEAST_HUMBLE
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
#else
  throw std::runtime_error("Logger level cannot be set in this version of ROS2");
#endif
}

BOOST_PYTHON_MODULE(MODULE_NAME)
{
  Py_Initialize();
  {
    bp::def("init_ros", &init_ros);
    bp::def("get_parameter", &get_parameter);
    bp::def("set_parameter", &set_parameter);
    bp::def("set_logger_level", &set_logger_level);
  }
}

}  // namespace reach_ros

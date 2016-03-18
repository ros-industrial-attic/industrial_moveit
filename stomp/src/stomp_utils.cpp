#include <stomp/stomp_utils.h>

namespace stomp
{

void getDifferentiationMatrix(int num_time_steps, CostComponents order, double dt, Eigen::MatrixXd& diff_matrix)
{
  diff_matrix = Eigen::MatrixXd::Zero(num_time_steps, num_time_steps);
  double multiplier = 1.0/pow(dt,(int)order);
  for (int i=0; i<num_time_steps; ++i)
  {
    for (int j=-DIFF_RULE_LENGTH/2; j<=DIFF_RULE_LENGTH/2; ++j)
    {
      int index = i+j;
      if (index < 0)
      {
        index = 0;
        continue;
      }

      if (index >= num_time_steps)
      {
        index = num_time_steps-1;
        continue;
      }
      diff_matrix(i,index) = multiplier * DIFF_RULES[order][j+DIFF_RULE_LENGTH/2];
    }
  }
}

void differentiate(const Eigen::VectorXd& parameters, CostComponents derivative_order,
                          double dt, Eigen::VectorXd& derivatives )
{
  Eigen::MatrixXd diff_matrix;
  getDifferentiationMatrix(parameters.size(),derivative_order,dt,diff_matrix);
  derivatives = diff_matrix*parameters;
}

bool readDoubleArray(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<double>& array, const bool verbose)
{
  XmlRpc::XmlRpcValue d_array_xml;
  if(!node_handle.getParam(parameter_name, d_array_xml))
  {
    ROS_ERROR_COND(verbose, "Could not retrieve parameter %s in namespace %s.", parameter_name.c_str(), node_handle.getNamespace().c_str());
    return false;
  }

  if (d_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_COND(verbose, "XmlRpcValue is not of type array.");
    return false;
  }

  array.clear();
  for (int i=0; i<d_array_xml.size(); ++i)
  {
    if (d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR_COND(verbose, "XmlRpcValue is neither a double nor a integer array.");
      return false;
    }
    double value = 0.0;
    if (d_array_xml[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      value = static_cast<double>(static_cast<int>(d_array_xml[i]));
    }
    else
    {
      value = static_cast<double>(d_array_xml[i]);
    }
    array.push_back(value);
  }

  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, double& value)
{
  if (!config.hasMember(key))
  {
    return false;
  }
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    return false;
  }
  value = param;
  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, std::vector<double>& double_array)
{
  if (!config.hasMember(key))
  {
    ROS_ERROR("XmlRpcValue does not contain key %s.", key.c_str());
    return false;
  }

  XmlRpc::XmlRpcValue d_array_xml = config[key];

  if (d_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("XmlRpcValue is not of type array.");
    return false;
  }

  double_array.clear();
  for (int i=0; i<d_array_xml.size(); ++i)
  {
    if (d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        d_array_xml[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
    {
      ROS_ERROR("XmlRpcValue is neither a double nor a integer array.");
      return false;
    }
    double value = 0.0;
    if (d_array_xml[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
    {
      value = static_cast<double>(static_cast<int>(d_array_xml[i]));
    }
    else
    {
      value = static_cast<double>(d_array_xml[i]);
    }
    double_array.push_back(value);
  }

  return true;
}

bool getParam(XmlRpc::XmlRpcValue& config, const std::string& key, bool& value)
{
  if (!config.hasMember(key))
    {
      return false;
    }
  XmlRpc::XmlRpcValue param = config[key];
  if (param.getType() != XmlRpc::XmlRpcValue::TypeBoolean)
  {
    return false;
  }
  value = param;
  return true;
}

bool readStringArray(ros::NodeHandle& node_handle, const std::string& parameter_name, std::vector<std::string>& str_array, const bool verbose)
{
  XmlRpc::XmlRpcValue list;
  if (!node_handle.getParam(parameter_name, list))
  {
    ROS_ERROR_COND(verbose, "Could not retrieve parameter %s in namespace %s.", parameter_name.c_str(), node_handle.getNamespace().c_str());
    return false;
  }

  XmlRpc::XmlRpcValue str_array_xml = list;
  if (str_array_xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR_COND(verbose, "XmlRpcValue is not of type array.");
    return false;
  }

  str_array.clear();
  for (int i = 0; i < str_array_xml.size(); ++i)
  {
    str_array.push_back(std::string(str_array_xml[i]));
  }
  return true;
}

} // namespace stomp

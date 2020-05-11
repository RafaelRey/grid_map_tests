#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <string>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <grid_map_costmap_2d/Costmap2DConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <filters/filter_chain.h>

class GridMapBridge
{
public:
  GridMapBridge(bool &success) : filterChain_("grid_map::GridMap")
  {
    grid_map_subscriber_ = nh.subscribe("grid_map", 1, &GridMapBridge::gridMapCallback, this);
    grid_map_publisher_ = nh.advertise<grid_map_msgs::GridMap>("grid_map_out", 1, true);

    tf_list_.reset(new tf2_ros::TransformListener(buffer_));
    costmap.reset(new costmap_2d::Costmap2DROS("costmap", buffer_));

    if (!readParameters())
    {
      success = false;
      return;
    }
    // Setup filter chain.
    if (!filterChain_.configure(filterChainParametersName_, nh))
    {
      ROS_ERROR("Could not configure the filter chain!");
      success = false;
      return;
    }
  }

private:
  void filterGridMap(grid_map::GridMap &input_map)
  {
    if (!filterChain_.update(input_map, input_map))
    {
      ROS_ERROR("Could not update the grid map filter chain!");
      return;
    }
  }
  void gridMapCallback(const grid_map_msgs::GridMap &gridmap_msg)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getCostmap()->getMutex()));

    grid_map::GridMap temp_gridmap;
    grid_map::GridMapRosConverter::fromMessage(gridmap_msg, temp_gridmap);

    filterGridMap(temp_gridmap);

    std::vector<std::string> layers_names;
    layers_names.push_back("elevation");

    ROS_WARN("Adding layers, resolution: %f", temp_gridmap.getResolution());

    if (first_map)
      mainGridMap.setGeometry(temp_gridmap.getLength(), temp_gridmap.getResolution());

    if (mainGridMap.addDataFrom(temp_gridmap, true, true, false, layers_names))
    {
      ROS_INFO("Grid map resolution: %f", mainGridMap.getResolution());

      if (first_map)
      {
        first_map = false;
        costmap_converter.initializeFromGridMap(mainGridMap, *costmap->getCostmap());
      }
      else
      {
        costmap_converter.setCostmap2DFromGridMap(mainGridMap, std::string("obstacle_layer"), *costmap->getCostmap());
      }
    }
    lock.unlock();
    grid_map_msgs::GridMap outputMessage;
    grid_map::GridMapRosConverter::toMessage(temp_gridmap, outputMessage);
    grid_map_publisher_.publish(outputMessage);
  }
  bool readParameters()
  {
    if (!nh.getParam("input_topic", inputTopic_))
    {
      ROS_ERROR("Could not read parameter `input_topic`.");
      return false;
    }
    nh.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
    return true;
  }
  ros::NodeHandle nh{ "~" };
  ros::Subscriber grid_map_subscriber_;
  ros::Publisher grid_map_publisher_;

  tf2_ros::Buffer buffer_{ ros::Duration(5) };

  filters::FilterChain<grid_map::GridMap> filterChain_;

  std::string filterChainParametersName_;
  std::string inputTopic_;

  // Costmsp
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap;
  std::unique_ptr<tf2_ros::TransformListener> tf_list_;
  // Converter
  grid_map::Costmap2DConverter<grid_map::GridMap> costmap_converter;
  // Grid map
  grid_map::GridMap mainGridMap;

  bool first_map = true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_map_bridge_node");

  bool success;
  GridMapBridge bridge(success);

  if (success){
    ros::spin();
  }else{
    ROS_ERROR("Error initializing grid map bridge");
    return 1;
  }

  return 0;
}
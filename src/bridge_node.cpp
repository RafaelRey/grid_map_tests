#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <ros/ros.h>
#include <string>

#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <grid_map_costmap_2d/Costmap2DConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <filters/filter_chain.h>

class GridMapBridge
{
public:
  GridMapBridge(bool &success) : filterChain_("grid_map::GridMap")
  {
    if (!readParameters())
    {
      ROS_ERROR("Failed to load parameters!");
      success = false;
      return;
    }

    if (!filterChain_.configure(filterChainParametersName_, nh))
    {
      ROS_ERROR("Could not configure the filter chain!");
      success = false;
      return;
    }

    grid_map_subscriber_ = nh.subscribe(inputTopic_, 1, &GridMapBridge::gridMapCallback, this);
    grid_map_publisher_ = nh.advertise<grid_map_msgs::GridMap>("grid_map_out", 1, true);
    nh.param("world_frame", world_frame_);
    nh.param("base_frame", base_frame_);

    tf_list_.reset(new tf2_ros::TransformListener(buffer_));
    costmap.reset(new costmap_2d::Costmap2DROS("costmap", buffer_));
  }

private:
  geometry_msgs::TransformStamped getRobotPose(const std::string &target_frame, const std::string &robot_frame)
  {

    geometry_msgs::TransformStamped transformStamped;
    try
    {
      transformStamped = buffer_.lookupTransform(target_frame, robot_frame,
                                                 ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    return transformStamped;
  }
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
    ROS_INFO("Map Position, [%f, %f]", temp_gridmap.getPosition().x(), temp_gridmap.getPosition().y());

    double tpx, tpy;
    int idx;
    double res = temp_gridmap.getResolution();
    
    int i_max = costmap->getCostmap()->getSizeInCellsX() * costmap->getCostmap()->getSizeInCellsY();
    ROS_INFO("I Max: %d", i_max);
    double wmx,wmy;
    /*for(int i = 0; i < i_max; ++i ){
        costmap->getCostmap()->indexToCells(i, mx, my);
        ROS_INFO("Index %d to cells: [%d, %d]", i, mx,my);
        costmap->getCostmap()->mapToWorld(mx,my,wmx,wmy);
        ROS_INFO("Cells to world: [%f, %f]", wmx,wmy);
        usleep(1e5);
    }*/
    unsigned int mx,my;
    // grid_map::Matrix& data = temp_gridmap["traversability"];
    grid_map::Position position;
    for (grid_map::GridMapIterator it(temp_gridmap); !it.isPastEnd(); ++it)
    {
      temp_gridmap.getPosition(*it, position);
      tpx = position.x();//- gridmap_msg.info.pose.position.x;
      tpy = position.y();//- gridmap_msg.info.pose.position.y;
      ROS_INFO("Temp pos: [%f, %f]", tpx, tpy);
      if(costmap->getCostmap()->worldToMap(tpx,tpy,mx,my) && temp_gridmap.at("traversability", *it) > 0.5 ){
        costmap->getCostmap()->setCost(mx,my, costmap_2d::LETHAL_OBSTACLE);
        ROS_WARN("Setting lethal obstacle");
      }
      ROS_INFO("Mx, My: [%d, %d]", mx, my);
    }
    
    /*if (first_map_)
    {
      grid_map::Length map_len = temp_gridmap.getLength();
      mainGridMap.setGeometry(temp_gridmap.getLength(), temp_gridmap.getResolution());
      ROS_INFO("Temp grid map length: [%f,%f], resolution: %f", map_len.x(), map_len.y(), temp_gridmap.getResolution());
    }*/

    /*if (mainGridMap.addDataFrom(temp_gridmap, true, true, false, layers_names))
    {
      ROS_INFO("Grid map resolution: %f", mainGridMap.getResolution());

      if (first_map_)
      {
        first_map_ = false;
        costmap_converter.initializeFromGridMap(mainGridMap, *costmap->getCostmap());
      }
      else
      {
        costmap_converter.setCostmap2DFromGridMap(mainGridMap, std::string("traversability"), *costmap->getCostmap());
      }
    }*/
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
  ros::NodeHandle nh{"~"};
  ros::Subscriber grid_map_subscriber_;
  ros::Publisher grid_map_publisher_;
  std::string base_frame_ = {"base_link"};
  std::string world_frame_ = {"map"};

  filters::FilterChain<grid_map::GridMap> filterChain_;

  std::string filterChainParametersName_;
  std::string inputTopic_;

  // Costmsp
  std::unique_ptr<costmap_2d::Costmap2DROS> costmap;
  tf2_ros::Buffer buffer_{ros::Duration(5)};
  std::unique_ptr<tf2_ros::TransformListener> tf_list_;
  // Converter
  grid_map::Costmap2DConverter<grid_map::GridMap, grid_map::Costmap2DCenturyTranslationTable> costmap_converter;
  // Grid map
  grid_map::GridMap mainGridMap;

  //Let's palay with a custom translation table
  // grid_map::Costmap2DTranslationTable<costmap_2d::NO_INFORMATION, costmap_2d::> translation_table_
  bool first_map_ = true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_map_bridge_node");

  bool success = true;
  GridMapBridge bridge(success);

  if (success)
  {
    ros::spin();
  }
  else
  {
    ROS_ERROR("Error initializing grid map bridge");
    return 1;
  }

  return 0;
}
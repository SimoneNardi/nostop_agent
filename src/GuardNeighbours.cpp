#include "GuardNeighbours.h"
#include "WorldMap.h"

#include "ros/ros.h"

#include <nav_msgs/OccupancyGrid.h>

#include "GlobalPreProcessor.h"

using namespace std;
using namespace Robotics;
using namespace Robotics::GameTheory;

/////////////////////////////////////////////
GuardNeighbours::GuardNeighbours() 
: m_data(nullptr)
, m_isUpdated(false)
, m_node("~")
{
  m_data = std::make_shared<WorldMap>();
  
#ifdef _DEBUG_PRINT
  std::cout << "Subscribe GuardNeighbours /simulator/neighbours"<<std::endl;
#endif
  m_sub = m_node.subscribe<nav_msgs::OccupancyGrid>("/simulator/neighbours", 1, &GuardNeighbours::UpdateNeighboursCallBack, this);
}

/////////////////////////////////////////////
GuardNeighbours::~GuardNeighbours()
{}

/////////////////////////////////////////////
void plot(std::string filename_, const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
  FILE *fp = fopen(filename_.c_str(), "w");
  if (fp == NULL) return ;
 
  // The image header
  char header[ 18 ] = { 0 }; // char = byte
  header[ 2 ] = 2; // truecolor
  header[ 12 ] = msg->info.width & 0xFF;
  header[ 13 ] = (msg->info.width >> 8) & 0xFF;
  header[ 14 ] = msg->info.height & 0xFF;
  header[ 15 ] = (msg->info.height >> 8) & 0xFF;
  header[ 16 ] = 24; // bits per pixel
  
  fwrite((const char*)&header, 1, sizeof(header), fp);
  
  // The image data is stored bottom-to-top, left-to-right
  for (int y = msg->info.height -1; y >= 0; y--)
    for (int x = 0; x < msg->info.width; x++)
    {
      int index = x * msg->info.height + y;
      char b = (msg->data[index] & 0x0000FF);
      char g = (msg->data[index] & 0x00FF00) >> 8;
      char r = (msg->data[index] & 0xFF0000) >> 16;
      putc((int)(b & 0xFF),fp);
      putc((int)(g & 0xFF),fp);
      putc((int)(r & 0xFF),fp);
    }
 
  // The file footer
  static const char footer[ 26 ] =
  "\0\0\0\0" // no extension area
  "\0\0\0\0" // no developer directory
  "TRUEVISION-XFILE" // yep, this is a TGA file
  ".";
  fwrite((const char*)&footer, 1, sizeof(footer), fp);
 
  fclose(fp);
  return;
}

/////////////////////////////////////////////
void GuardNeighbours::UpdateNeighboursCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	Lock1 lock(m_mutex);
	m_time = msg->info.map_load_time;
	m_data->update(msg->data);
	
	//plot("/home/simone/debug/test_neighbours.tga", msg);
	
	this->setNew();
	// notify new data
}

/////////////////////////////////////////////
std::shared_ptr<WorldMap> GuardNeighbours::getData() const
{
  Lock1 lock(m_mutex);
  return m_data;
}

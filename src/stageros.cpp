/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>


// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf/LinearMath/Transform.h"
#include <std_srvs/Empty.h>

#include <stage_ros/CmdPosesRecScans.h>

#include "tf/transform_broadcaster.h"

#define USAGE "stageros <worldfile>"
#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define BASE_SCAN "base_scan"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"
#define POSE "cmd_pose"
#define POSESTAMPED "cmd_pose_stamped"

// Our node
class StageNode
{
private:

    // roscpp-related bookkeeping
    ros::NodeHandle n_;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;
    std::vector<Stg::Color *> colors;

    //a structure representing a robot inthe simulator
    struct StageRobot
    {
        //stage related models
        Stg::ModelPosition* positionmodel; //one position
        std::vector<Stg::ModelRanger *> lasermodels; //multiple rangers per position

        Stg::Color* color;

        //ros publishers

        std::vector<ros::Publisher> laser_pubs; //multiple lasers

        ros::Subscriber cmdvel_sub; //one cmd_vel subscriber
        ros::Subscriber pose_sub;
    };

    std::vector<StageRobot const *> robotmodels_;

    // Used to remember initial poses for soft reset
    std::vector<Stg::Pose> initial_poses;
    ros::ServiceServer reset_srv_;

    ros::ServiceServer cmd_poses_rec_scans_srv_;
    // stage_ros::CmdPosesRecScans cmd_poses_rec_scans_srv_;

    ros::Publisher clock_pub_;

    bool isDepthCanonical;
    bool use_model_names;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model* mod, StageNode* node);

    static bool s_update(Stg::World* world, StageNode* node)
    {
        // node->WorldCallback();
        // We return false to indicate that we want to be called again (an
        // odd convention, but that's the way that Stage works).
        return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID, Stg::Model* mod) const;
    const char *mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const;

    tf::TransformBroadcaster tf;

    // Last time that we received a velocity command
    ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;

    // Last time we saved global position (for velocity calculation).
    ros::Time base_last_globalpos_time;
    // Last published global pose of each robot
    std::vector<Stg::Pose> base_last_globalpos;

public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names);
    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();

    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a cmd_vel message, which set velocities.
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg);

    // Message callback for a cmd_pose message, which sets positions.
    void poseReceived(int idx, const boost::shared_ptr<geometry_msgs::Pose const>& msg);

    // Service callback for soft reset
    bool cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    // Service callback for commanding an array of poses and returning an array of laserscans
    bool cb_cmd_poses_rec_scans_srv(stage_ros::CmdPosesRecScans::Request& request, stage_ros::CmdPosesRecScans::Response& response);

    // The main simulator object
    Stg::World* world;
};

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s", robotID, name);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s", ((Stg::Ancestor *) mod)->Token(), name);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
        }

        return buf;
    }
    else
        return name;
}

const char *
StageNode::mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s:%lu", robotID, name, deviceID);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s_%u", ((Stg::Ancestor *) mod)->Token(), name, (unsigned int)deviceID);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s_%u", (unsigned int)robotID, name, (unsigned int)deviceID);
        }

        return buf;
    }
    else
    {
        static char buf[100];
        snprintf(buf, sizeof(buf), "/%s_%u", name, (unsigned int)deviceID);
        return buf;
    }
}

void
StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
    // mod->ClearBlocks();
    // mod->AddBlockRect(
    
    // Stg::Color color = mod->GetColor();
    // ROS_INFO_STREAM(color.r);
    // color.r = 0.0;
    // mod->SetColor(color);
    if (dynamic_cast<Stg::ModelRanger *>(mod))
        node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
    if (dynamic_cast<Stg::ModelPosition *>(mod)) {
      Stg::ModelPosition * p = dynamic_cast<Stg::ModelPosition *>(mod);
      // remember initial poses
      node->positionmodels.push_back(p);
      node->initial_poses.push_back(p->GetGlobalPose());
    }
}

bool
StageNode::cb_cmd_poses_rec_scans_srv(stage_ros::CmdPosesRecScans::Request& request, stage_ros::CmdPosesRecScans::Response& response)
{
    // necessary??
    boost::mutex::scoped_lock lock(msg_lock);
    int num_robots = request.poses.size();

    // Update pose of all robots
    for (int robot_idx=0; robot_idx<num_robots; robot_idx++){
        geometry_msgs::Pose ros_pose = request.poses[robot_idx];
        Stg::Pose pose;
        pose.x = ros_pose.position.x;
        pose.y = ros_pose.position.y;
        pose.z = 0;
        // major ROS hack to just send yaw within a quaternion
        pose.a = ros_pose.orientation.w;
        this->positionmodels[robot_idx]->SetPose(pose);
    }
    this->world->UpdateAll();
    this->world->Stop();


    // Then update laser sensor for all robots
    for (int robot_idx=0; robot_idx<num_robots; robot_idx++){
        StageRobot* robotmodel = const_cast<StageRobot*>(this->robotmodels_[robot_idx]);
        Stg::ModelRanger* lasermodel = robotmodel->lasermodels[1];
        std::vector<Stg::ModelRanger::Sensor>& sensors = const_cast<std::vector<Stg::ModelRanger::Sensor>&>(lasermodel->GetSensors());
        Stg::ModelRanger::Sensor& sensor = sensors[0];
        sensor.Update(lasermodel);

        sensor_msgs::LaserScan msg;
        msg.angle_min = -sensor.fov/2.0;
        msg.angle_max = +sensor.fov/2.0;
        msg.angle_increment = sensor.fov/(double)(sensor.sample_count-1);
        msg.range_min = sensor.range.min;
        msg.range_max = sensor.range.max;
        msg.ranges.resize(sensor.ranges.size());

        for(unsigned int i = 0; i < sensor.ranges.size(); i++)
        {
            msg.ranges[i] = sensor.ranges[i];
        }

        // if (robotmodel->lasermodels.size() > 1)
            // msg.header.frame_id = mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
        // else
            // msg.header.frame_id = mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel));

        msg.header.stamp = sim_time;
        response.laserscans.push_back(msg);
    }
    return true;
}


bool
StageNode::cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Resetting stage!");
  for (size_t r = 0; r < this->positionmodels.size(); r++) {
    this->positionmodels[r]->SetPose(this->initial_poses[r]);
    this->positionmodels[r]->SetStall(false);
  }
  return true;
}


void
StageNode::cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);
    this->positionmodels[idx]->SetSpeed(msg->linear.x,
                                        msg->linear.y,
                                        msg->angular.z);
    this->base_last_cmd = this->sim_time;
}

void
StageNode::poseReceived(int idx, const boost::shared_ptr<geometry_msgs::Pose const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);
    Stg::Pose pose;

    double roll, pitch, yaw;
    tf::Matrix3x3 m(tf::Quaternion(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w));
    m.getRPY(roll, pitch, yaw);
    pose.x = msg->position.x;
    pose.y = msg->position.y;
    pose.z = 0;
    pose.a = yaw;
    this->positionmodels[idx]->SetPose(pose);

    }

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names)
{
    this->use_model_names = use_model_names;
    this->sim_time.fromSec(0.0);
    this->base_last_cmd.fromSec(0.0);
    double t;
    ros::NodeHandle localn("~");
    if(!localn.getParam("base_watchdog_timeout", t))
        t = 0.2;
    this->base_watchdog_timeout.fromSec(t);

    if(!localn.getParam("is_depth_canonical", isDepthCanonical))
        isDepthCanonical = true;


    // We'll check the existence of the world file, because libstage doesn't
    // expose its failure to open it.  Could go further with checks (e.g., is
    // it readable by this user).
    struct stat s;
    if(stat(fname, &s) != 0)
    {
        ROS_FATAL("The world file %s does not exist.", fname);
        ROS_BREAK();
    }

    // initialize libstage
    Stg::Init( &argc, &argv );

    if(gui)
        this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
    else
        this->world = new Stg::World();

    // Apparently an Update is needed before the Load to avoid crashes on
    // startup on some systems.
    // As of Stage 4.1.1, this update call causes a hang on start.
    //this->UpdateWorld();
    this->world->Load(fname);

    // We add our callback here, after the Update, so avoid our callback
    // being invoked before we're ready.
    this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);

    this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);
}


// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageNode::SubscribeModels()
{
    n_.setParam("/use_sim_time", true);

    // for (size_t r = 0; r < this->positionmodels.size(); r++)
    // {
    //     StageRobot* new_robot = new StageRobot;
    //     new_robot->positionmodel = this->positionmodels[r];
    //     new_robot->positionmodel->Subscribe();


    //     for (size_t s = 0; s < this->lasermodels.size(); s++)
    //     {
    //         if (this->lasermodels[s] and this->lasermodels[s]->Parent() == new_robot->positionmodel)
    //         {
    //             new_robot->lasermodels.push_back(this->lasermodels[s]);
    //             this->lasermodels[s]->Subscribe();
    //         }
    //     }

    //     new_robot->cmdvel_sub = n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10, boost::bind(&StageNode::cmdvelReceived, this, r, _1));
    //     new_robot->pose_sub = n_.subscribe<geometry_msgs::Pose>(mapName(POSE, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10, boost::bind(&StageNode::poseReceived, this, r, _1));

    //     for (size_t s = 0;  s < new_robot->lasermodels.size(); ++s)
    //     {
    //         if (new_robot->lasermodels.size() == 1)
    //             new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
    //         else
    //             new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));

    //     }

    //     this->robotmodels_.push_back(new_robot);
    // }
    clock_pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 10);

    // advertising reset service
    reset_srv_ = n_.advertiseService("reset_positions", &StageNode::cb_reset_srv, this);
    
    cmd_poses_rec_scans_srv_ = n_.advertiseService("command_poses_receive_scans", &StageNode::cb_cmd_poses_rec_scans_srv, this);

    return(0);
}

StageNode::~StageNode()
{    
    for (std::vector<StageRobot const*>::iterator r = this->robotmodels_.begin(); r != this->robotmodels_.end(); ++r)
        delete *r;
}

bool
StageNode::UpdateWorld()
{
	return false;
    // return this->world->UpdateAll();
}

void
StageNode::WorldCallback()
{
    boost::mutex::scoped_lock lock(msg_lock);

    this->sim_time.fromSec(world->SimTimeNow() / 1e6);
    // We're not allowed to publish clock==0, because it used as a special
    // value in parts of ROS, #4027.
    if(this->sim_time.sec == 0 && this->sim_time.nsec == 0)
    {
        ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
        return;
    }

    // TODO make this only affect one robot if necessary
    if((this->base_watchdog_timeout.toSec() > 0.0) &&
            ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
    {
        for (size_t r = 0; r < this->positionmodels.size(); r++)
            this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
    }

    //loop on the robot models
    for (size_t r = 0; r < this->robotmodels_.size(); ++r)
    {
        StageRobot const * robotmodel = this->robotmodels_[r];

        //loop on the laser devices for the current robot
        for (size_t s = 0; s < robotmodel->lasermodels.size(); ++s)
        {
            Stg::ModelRanger const* lasermodel = robotmodel->lasermodels[s];
            const std::vector<Stg::ModelRanger::Sensor>& sensors = lasermodel->GetSensors();

            // ADD THIS BACK IN!!!
            // if( sensors.size() > 1 )

                // ROS_WARN( "ROS Stage currently supports rangers with 1 sensor only." );

            // for now we access only the zeroth sensor of the ranger - good
            // enough for most laser models that have a single beam origin
            const Stg::ModelRanger::Sensor& sensor = sensors[0];

            if( sensor.ranges.size() )
            {
                // Translate into ROS message format and publish
                sensor_msgs::LaserScan msg;
                msg.angle_min = -sensor.fov/2.0;
                msg.angle_max = +sensor.fov/2.0;
                msg.angle_increment = sensor.fov/(double)(sensor.sample_count-1);
                msg.range_min = sensor.range.min;
                msg.range_max = sensor.range.max;
                msg.ranges.resize(sensor.ranges.size());
                msg.intensities.resize(sensor.intensities.size());

                for(unsigned int i = 0; i < sensor.ranges.size(); i++)
                {
                    msg.ranges[i] = sensor.ranges[i];
                    msg.intensities[i] = (uint8_t)sensor.intensities[i];
                }

                if (robotmodel->lasermodels.size() > 1)
                    msg.header.frame_id = mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    msg.header.frame_id = mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel));

                msg.header.stamp = sim_time;
                robotmodel->laser_pubs[s].publish(msg);
            }

            // // Also publish the base->base_laser_link Tx.  This could eventually move
            // // into being retrieved from the param server as a static Tx.
            // Stg::Pose lp = lasermodel->GetPose();
            // tf::Quaternion laserQ;
            // laserQ.setRPY(0.0, 0.0, lp.a);
            // tf::Transform txLaser =  tf::Transform(laserQ, tf::Point(lp.x, lp.y, robotmodel->positionmodel->GetGeom().size.z + lp.z));

            // if (robotmodel->lasermodels.size() > 1)
            //     tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
            //                                           mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
            //                                           mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
            // else
            //     tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
            //                                           mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
            //                                           mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));
        }

        // //the position of the robot
        // tf.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),
        //                                       sim_time,
        //                                       mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
        //                                       mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

    }

    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = sim_time;
    this->clock_pub_.publish(clock_msg);
}

int 
main(int argc, char** argv)
{ 
    if( argc < 2 )
    {
        puts(USAGE);
        exit(-1);
    }

    ros::init(argc, argv, "stageros");

    bool gui = true;
    bool use_model_names = false;
    for(int i=0;i<(argc-1);i++)
    {
        if(!strcmp(argv[i], "-g"))
            gui = false;
        if(!strcmp(argv[i], "-u"))
            use_model_names = true;
    }

    StageNode sn(argc-1,argv,gui,argv[argc-1], use_model_names);

    if(sn.SubscribeModels() != 0)
        exit(-1);

    boost::thread t = boost::thread(boost::bind(&ros::spin));

    // New in Stage 4.1.1: must Start() the world.
    // sn.world->Start();

    // TODO: get rid of this fixed-duration sleep, using some Stage builtin
    // PauseUntilNextUpdate() functionality.
    ros::WallRate r(10.0);
    while(ros::ok() && !sn.world->TestQuit())
    {
        if(gui){
            Fl::wait(r.expectedCycleTime().toSec());
        }
        else
        {
        	std::cout << "else!!" << std::endl;
            sn.UpdateWorld();
            r.sleep();
        }
    }
    t.join();

    exit(0);
}

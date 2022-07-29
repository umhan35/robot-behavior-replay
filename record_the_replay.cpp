// This is the code we used to generate videos shown in Figure 2-4.

#include <ros/ros.h>
#include "main/run_node.h"
#include "../common/NavOnlyArena.h"
#include <arena_nav/ArenaNavTestStub.h>
#include <nav/NavigationMoveBase.h>
#include <arena_nav/TestCourseNav.h>
#include <mongodb/MongodbLogger.h>
#include <pcl/common/centroid.h>
#include <PointTurretHead.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>
#include <arbotix_msgs/Relax.h>
#include <sound_play/sound_play.h>
#include <boost/format.hpp>
/**

 RECORD

 // SIM
roslaunch fetchit_challenge main_arena_montreal2019_simple_gearbox_and_caddy.launch

 // start moveit

 // ON REAL ROBOT
  // nav_to_caddy_station
rosrun arena_nav test_test_course_nav _use_move_base:=true _fn:=nav_to_caddy_station
  // prepare arm
rosrun common_manipulation test_prepare_arm

 // widget service
rosrun widget_detect_and_grab widget_detect_and_grab

 // SIM: move robot to
 //  gearbox bottoms
rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: fetch, pose: { position: { x: -0.075, y: 0.6, z: 0 }, orientation: {x: 0, y: 0, z: 1.0, w: 1 } }}'
 //  caddy table
rostopic pub -1 /gazebo/set_model_state gazebo_msgs/ModelState '{model_name: fetch, pose: { position: { x: -0.1349, y: -0.62, z: 0 }, orientation: {x: 0, y: 0, z: -0.707, w: 0.707 } }}'


 // LOCAL
  // start mongodb_store
roslaunch mongodb_store mongodb_store.launch machine:=localhost use_daemon:=true port:=27017
  // logger server
rosrun mongodb mongodb_logger_action_server.py

 // run
rosrun main record_the_replay



 REPLAY (either pick or place gearbox bottom)
 // ON REAL ROBOT
  // nav_to_safest_place_for_arm_movement
rosrun arena_nav test_test_course_nav _use_move_base:=true _fn:=nav_to_safest_place_for_arm_movement
  // prepare arm
rosrun common_manipulation test_prepare_arm

 // SIM
   // start sim with clock remapped to clock_temp
roslaunch fetchit_challenge main_arena_montreal2019_simple_gearbox_and_caddy.launch
 // start moveit
 // start mongodb mongodb_store
roslaunch mongodb_store mongodb_store.launch host:=localhost use_daemon:=true port:=27017
 // playback with
rosrun mongodb_store mongodb_play.py --mongodb-name NAME
 */
class ArenaForReplay: public Arena {
private:
    MongodbLogger mongodb_logger;
    sound_play::SoundClient sc;

public:

    explicit ArenaForReplay(ArenaNav* nav) : Arena(nav) {
        white_pub = nh.advertise<sensor_msgs::PointCloud2>("/projector/white_points", 1);
        green_pub = nh.advertise<sensor_msgs::PointCloud2>("/projector/green_points", 1);

        vis_pub = nh.advertise<visualization_msgs::Marker>("/gearbox_compartment_marker", 1);
    }

    void create_bt() override {
        mongodb_logger.set_topics({"/gripper_controller/gripper_action/goal", //gripper
                                   "/arm_with_torso_controller/follow_joint_trajectory/goal", // arm & torso
                                   "/torso_controller/follow_joint_trajectory/goal", // torso
                                   // head
                                   "/head_controller/point_head/goal",
                                   "/head_controller/follow_joint_trajectory/goal",
                                   "/cmd_vel", // base
                                   // Projector turret
                                   "/projector/follow_joint_trajectory/goal",
                                   // Point cloud projection
                                   "/projector/white_points",
                                   "/projector/green_points",
                                   // Caddy compartment marker
                                   "/gearbox_compartment_marker",
                                   // Navigation projection
                                   "/projector/nav/path", // path
                                   "/base_scan_obstacles_only", // ground obstacle
                                   // Speech
                                   "/robotsound"
        });

        register_bt_nodes();
        bt_tree = factory.createTreeFromFile(ros::package::getPath("main") + "/BTs/replay experiment/replay.xml");
        tree = new ExplainableBT(bt_tree);
    }

    void run() override {
        mongodb_logger.start();
        Arena::run();
        mongodb_logger.end();
    }

    // methods that must be overridden
    void pick_large_gear() override {}
    void insert_large_gear() override {}
    void close_schunk_machine_door() override {}
    void pick_small_gear() override {}
    void place_small_gear() override {}
    void pick_up_screw() override {}
    void place_screw() override {}
    void pick_up_gearbox_top() override {}
    void pick_up_gearbox_bottom() override {}
    void place_gearbox_top() override {}
    void place_gearbox_bottom() override {}
    void open_schunk_machine_door() override {}
    void remove_large_gear() override {}
    void place_machined_large_gear() override {}
    void pick_up_caddy() override {}
    void drop_off_caddy() override {}

protected:
    void register_bt_nodes() override {
        Arena::register_bt_nodes();

        factory.registerSimpleAction("project point cloud", std::bind(&ArenaForReplay::project_point_cloud, this, std::placeholders::_1), { BT::InputPort<sensor_msgs::PointCloud2>("white"), BT::InputPort<sensor_msgs::PointCloud2>("green") });
        factory.registerSimpleAction("clear point cloud projection", std::bind(&ArenaForReplay::clear_point_cloud_projection, this));
        factory.registerSimpleAction("reset projector", std::bind(&ArenaForReplay::reset_projector, this));
        factory.registerSimpleAction("tilt projector for navigation", std::bind(&ArenaForReplay::tilt_projector_for_navigation, this));
        factory.registerSimpleAction("visualize pose", std::bind(&ArenaForReplay::visualize_pose, this, std::placeholders::_1), { BT::InputPort<geometry_msgs::PoseStamped>("pose") });
        factory.registerSimpleAction("say", std::bind(&ArenaForReplay::play_audio, this, std::placeholders::_1), { BT::InputPort<std::string>("what"), BT::InputPort<double>("speed") });
        factory.registerSimpleAction("wait", std::bind(&ArenaForReplay::wait_seconds, this, std::placeholders::_1), { BT::InputPort<int>("seconds") });
    }

    ros::Publisher white_pub;
    ros::Publisher green_pub;
    tf::TransformBroadcaster broadcaster;

    PointTurretHead pointTurretHead;

    bool project_point_cloud(BT::TreeNode& self) { // TODO maybe a class? a little bit complex here

        // publish white points
        sensor_msgs::PointCloud2 white_points;
        self.getInput("white", white_points);
        white_pub.publish(white_points);

        // publish green points
        sensor_msgs::PointCloud2 green_points;
        self.getInput("green", green_points);
        green_pub.publish(green_points);

        // calculate centroid of white points
        pcl::PointCloud<pcl::PointXYZ> white_point_cloud;
        pcl::fromROSMsg(white_points, white_point_cloud);
        pcl::PointXYZ centroid;
        pcl::computeCentroid(white_point_cloud, centroid);

        // point turret to the centroid
        geometry_msgs::PointStamped centroid_pointStamped;
        centroid_pointStamped.header.frame_id = "base_link";
        centroid_pointStamped.point.x = centroid.x;
        centroid_pointStamped.point.y = centroid.y;
        centroid_pointStamped.point.z = centroid.z;

        // send the centroid as a TF frame
        tf::Transform tf;
        tf.setRotation(tf::Quaternion(0,0,0,1));
        tf.setOrigin(tf::Vector3(centroid.x, centroid.y, centroid.z));
        broadcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "base_link", "projection_looking_at"));

        pointTurretHead.look_at(centroid_pointStamped);

        return true;
    }

    bool clear_point_cloud_projection() {
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        sensor_msgs::PointCloud2 empty;
        pcl::toROSMsg(pcl_cloud, empty);
        empty.header.frame_id = "base_link";
        white_pub.publish(empty);
        green_pub.publish(empty);

        return true;
    }

    bool reset_projector() {
        pointTurretHead.pan_tilt_turret(0, 0);

        return true;
    }

    bool tilt_projector_for_navigation() {
        pointTurretHead.pan_tilt_turret(0, -1.5);

        // relax tilt servo (http://wiki.ros.org/arbotix_python#Services)
        //  turn off torque until next command
        ros::NodeHandle n;
        ros::ServiceClient serviceClient = n.serviceClient<arbotix_msgs::Relax>("/projector/projector_servo_tilt_joint/relax");
        arbotix_msgs::Relax _;
        serviceClient.call(_);

        return true;
    }

    ros::Publisher vis_pub;

    bool visualize_pose(BT::TreeNode& self) {

        geometry_msgs::PoseStamped pose;
        self.getInput("pose", pose);
        ROS_INFO_STREAM(pose);

        visualization_msgs::Marker marker;
        marker.header.frame_id = pose.header.frame_id;
        marker.lifetime = ros::Duration(60);
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose.pose;
        const auto error = 0.02;
        marker.scale.x = 0.1 + error;
        marker.scale.y = 0.1 + error;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 255;
        marker.color.g = 255;
        marker.color.b = 255;
        ROS_INFO_STREAM("Publishing marker: " << marker);
        vis_pub.publish(marker);

        // point turret to the centroid
        geometry_msgs::PointStamped pose_position;
        pose_position.header.frame_id = pose.header.frame_id;
        pose_position.point = pose.pose.position;

        pointTurretHead.look_at(pose_position);

        return true;
    }

    bool play_audio(BT::TreeNode& self) {
        auto what = self.getInput<std::string>("what");
        if ( ! what) {
            ROS_ERROR("[what] is required but not provided");
            return false;
        }

        double speed = 1;
        self.getInput("speed", speed);

        std::string filepath = (boost::format("speech/%s/output---speed-%.2f---pitch-0.ogg") % what.value() % speed).str();
        ROS_INFO_STREAM("Playing " << filepath << "...");

        sc.playWaveFromPkg("main", filepath);

        ROS_INFO("Done.");
    }

    bool wait_seconds(BT::TreeNode& self) {

        auto seconds = self.getInput<double>("seconds");
        if ( ! seconds) {
            ROS_ERROR("[seconds] is required but not provided");
            return false;
        }

        ros::Duration(seconds.value()).sleep();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "replay_experiment");
    ros::NodeHandle n; // without this, ROS_INFO won't print

    if (ros::param::has("use_sim_time")) {
        run_node(new ArenaForReplay(new ArenaNavTestStub()), true, false);
    }
    else {
        run_node(new ArenaForReplay(new TestCourseNav(new NavigationMoveBase())), true, false);
    }

    return 0;
}
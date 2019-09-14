// this node will publish the topic "blocks_poses"
// including all current blocks, pose is 3-D position

// ros communication:
    // subscribe to topic "/current_blocks"
    // subscribe to topic "/gazebo/model_states"
    // publish the topic "/blocks_poses"

#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Int8MultiArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <ur_rl_msgs/blocks_poses.h>

// global variables
int g_quantity;
int tracking_id;
geometry_msgs::Point target_point_msg;
std::vector<int8_t> g_current_blocks;
std::vector<double> g_x;
std::vector<double> g_y;
std::vector<double> g_z;
bool g_current_callback_started = false;
bool g_poses_updated = false;  // act as frequency control of publish loop

std::string intToString(int a) 
{
    std::stringstream ss;
    ss << a;
    return ss.str();
}

void currentBlockCallback(const std_msgs::Int8MultiArray& current_blocks) 
{
    // this topic contains information of what  blocks have been spawned
    if (!g_current_callback_started) 
    {
        // set first time started flag to true
        g_current_callback_started = true;
        ROS_INFO("current callback has been invoked first time");
    }
    g_quantity = current_blocks.data.size(); // Block spawner publish info about spawnd blocks
    
    // Set g_current_blocks with current_blocks from blocks_spawner  
    g_current_blocks.resize(g_quantity); // resize: size of 'g_current_block' is changed to 'n', exteded variables is initialized to the default value.
    g_current_blocks = current_blocks.data;
}

void modelStatesCallback(const gazebo_msgs::ModelStates& current_model_states) 
{
    // modelStatesCallback() updates global values of block positions
    if (g_current_callback_started) 
    {
        // only update when currentBlockCallback has been invoked the first time
        // get blocks positions according to settings in g_current_blocks
        std::vector<double> box_x;
        std::vector<double> box_y;
        std::vector<double> box_z;
        box_x.resize(g_quantity);
        box_y.resize(g_quantity);
        box_z.resize(g_quantity);

        // Find positiond of all current block in topic message
        bool poses_completed = true;
        for (int i=0; i<g_quantity; i++) 
        {
            // Get index of ith block
            std::string indexed_model_name;
                
            indexed_model_name = "red_blocks_" + intToString(i);
            /*
            if (g_current_blocks[i] == 0) 
            {
                indexed_model_name = "red_blocks_" + intToString(i);
                tracking_id = i;
            }
            else 
            {
                indexed_model_name = "red_blocks_" + intToString(tracking_id);
            }
            */

            // For matching the model we want, for loop is executed
            int index = -1;
            for (int j=0; j < current_model_states.name.size(); j++)             // current_model_states.name.size(): The number of models measured (the number of total model in GAZEBO)
            {
                if (current_model_states.name[j] == indexed_model_name) 
                {
                    index = j; 
                    break;
                }
            }
            
            if (index != -1) 
            {
                // this model name has been successfully indexed
                box_x[i] = current_model_states.pose[index].position.x;
                box_y[i] = current_model_states.pose[index].position.y;
                box_z[i] = current_model_states.pose[index].position.z;
            }
            else 
            {
                ROS_ERROR("fail to find model name in the model_states topic" );
                // in the test run, there is chance that the last block is not in the topic message
                // and g_quantity (fron spawner node) is larger than the block quantity here
                // because /gazebo/model_states are sampled at a high rate of about 1000Hz
                // so the position data should be aborted if fail to find the last block
                poses_completed = false;
            }

            if(i == g_quantity -1)
            {
                target_point_msg.x = current_model_states.pose[index].position.x;
                target_point_msg.y = current_model_states.pose[index].position.y;
                target_point_msg.z = current_model_states.pose[index].position.z;
            }
        }
        if (poses_completed) 
        {
            // only pass data to globals when they are completed
            g_x.resize(g_quantity);
            g_y.resize(g_quantity);
            g_z.resize(g_quantity);
            g_x = box_x;
            g_y = box_y;
            g_z = box_z;
            if (!g_poses_updated) 
            {
                // reset flag to true, after updating global value of g_x, g_y, g_z
                g_poses_updated = true;
            }
        }
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "blocks_poses_publisher");
    ros::NodeHandle nh;

    // initialize subscribers for "/current_blocks" and "/gazebo/model_states"
    ros::Subscriber current_subscriber = nh.subscribe("/current_blocks", 1, currentBlockCallback);
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    // initialize publisher for "/blocks_poses"
    ros::Publisher poses_publisher = nh.advertise<ur_rl_msgs::blocks_poses>("blocks_poses", 1);
    ros::Publisher target_pose_publisher = nh.advertise<geometry_msgs::Point>("target_blocks_pose", 1);
    ur_rl_msgs::blocks_poses current_poses_msg;

    // publishing loop
    while (ros::ok()) 
    {
        if (g_poses_updated) 
        {
            // only publish when blocks positions are updated
            // no need to publish repeated data
            g_poses_updated = false;  // set flag to false
            // there is tiny possibility that g_x is not in the length of g_blocks_quantity
            int local_quantity = g_x.size();  // so get length of g_x
            
            current_poses_msg.x.resize(local_quantity);
            current_poses_msg.y.resize(local_quantity);
            current_poses_msg.z.resize(local_quantity);
            current_poses_msg.x = g_x;
            current_poses_msg.y = g_y;
            current_poses_msg.z = g_z;
            poses_publisher.publish(current_poses_msg);      
            target_pose_publisher.publish(target_point_msg);      
            
        }
        ros::spinOnce();
    }
    return 0;
}

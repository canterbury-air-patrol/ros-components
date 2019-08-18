#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "smm_interface/Command.h"
#include "smm_interface/Search.h"
#include "smm_interface/SearchBegin.h"
#include "smm_interface/SearchCompleted.h"

extern "C" {
#include "smm-asset.h"
}

smm_connection object = NULL;
smm_asset asset = NULL;
smm_asset_command current_command = SMM_COMMAND_NONE;
smm_search current_search = NULL;

ros::Publisher command_pub;
ros::Publisher search_pub;
ros::ServiceServer search_begin_srv;
ros::ServiceServer search_completed_srv;

void updateCommand()
{
	smm_asset_command new_command = smm_asset_last_command(asset);

	if (new_command != current_command)
	{
		smm_interface::Command msg;
		switch(new_command) {
			case SMM_COMMAND_NONE:
			case SMM_COMMAND_CONTINUE:
			case SMM_COMMAND_UNKNOWN:
				msg.command = smm_interface::Command::COMMAND_NONE;
				break;
			case SMM_COMMAND_CIRCLE:
				msg.command = smm_interface::Command::COMMAND_CIRCLE;
				break;
			case SMM_COMMAND_RTL:
				msg.command = smm_interface::Command::COMMAND_RTL;
				break;
			case SMM_COMMAND_GOTO:
				msg.command = smm_interface::Command::COMMAND_GOTO;
				smm_asset_last_goto_pos (asset, &msg.latitude, &msg.longitude);
				ROS_INFO("GOTO: %lf %lf", msg.latitude, msg.longitude);
				break;
			default:
				ROS_INFO("Unknown command: %i", new_command);
		}
		ROS_INFO("Published new command: %u", msg.command);
		command_pub.publish(msg);
		current_command = new_command;
	}
}

double current_lat = 0.0;
double current_lon = 0.0;

void positionUpdated(const sensor_msgs::NavSatFix& msg)
{
	if (msg.status.status >= sensor_msgs::NavSatStatus::STATUS_FIX)
	{
		current_lat = msg.latitude;
		current_lon = msg.longitude;
		smm_asset_report_position(asset, msg.latitude, msg.longitude, msg.altitude, 0, 3);
		/* The current command might have been updated in response to position reporting */
		updateCommand();
	}
}

bool search_begin_cb(smm_interface::SearchBegin::Request& req, smm_interface::SearchBegin::Response& resp)
{
	resp.success = true;
	ROS_INFO("Search begin");
	while (current_search == NULL)
	{
		current_search = smm_asset_get_search (asset, current_lat, current_lon);
		if (current_search == NULL)
		{
			resp.success = false;
			return true;
		}

		/* Accept the search */
		if(smm_search_accept (current_search))
		{
			break;
		}
		/* Failed to accept search */
		smm_search_destroy (current_search);
		current_search = NULL;
	}

	/* Publish the search */
	smm_waypoints wps = NULL;
	size_t wps_count = 0;
	smm_interface::Search search;
	search.distance = smm_search_distance (current_search);
	search.search_length = smm_search_length (current_search);
	search.sweep_width = 200;
	smm_search_get_waypoints (current_search, &wps, &wps_count);
	for (size_t i = 0; i < wps_count; i++)
	{
		geometry_msgs::Point wp;
		wp.x = wps[i]->lat;
		wp.y = wps[i]->lon;
		search.waypoints.push_back(wp);
	}
	smm_waypoints_free (wps, wps_count);
	search_pub.publish(search);

	return true;
}

bool search_completed_cb(smm_interface::SearchCompleted::Request& req, smm_interface::SearchCompleted::Response& resp)
{
	ROS_INFO("Search completed");
	resp.accepted = true;
	if (current_search == NULL)
	{
		resp.accepted = false;
	}

	smm_search_complete (current_search);
	smm_search_destroy (current_search);
	current_search = NULL;

	return true;
}

int main(int argc, char *argv[])
{
	std::string host;
	std::string user;
	std::string pass;
	std::string asset_name;
	smm_assets assets = NULL;
	size_t assets_count = 0;
    smm_connection_status conn_status;

	ros::init(argc, argv, "smm_interface");

	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	if (!nh.getParam("smm_host", host))
	{
		ROS_ERROR("Parameter: smm_host not set");
	}
	if (!nh.getParam("smm_user", user))
	{
		ROS_ERROR("Parameter: smm_user not set");
	}
	if (!nh.getParam("smm_pass", pass))
	{
		ROS_ERROR("Parameter: smm_pass not set");
	}
	if (!nh.getParam("smm_asset_name", asset_name))
	{
		ROS_ERROR("Parameter: smm_asset_name not set");
	}

	/* Connect to the smm */
	object = smm_asset_connect(host.c_str(), user.c_str(), pass.c_str());
	if (object == NULL)
	{
		ROS_ERROR("Failed to create smm object");
	}
    conn_status = smm_asset_connection_get_state (object);
    if (conn_status != SMM_CONNECTION_CONNECTED)
    {
        ROS_ERROR("Failed to connect to server");
    }

	/* Get all assets for this account */
	{
		if (smm_asset_get_assets(object, &assets, &assets_count))
		{
			for(size_t i = 0; i < assets_count; i++)
			{
				if (strcmp (smm_asset_name (assets[i]), asset_name.c_str()) == 0)
				{
					asset = assets[i];
					break;
				}
				ROS_INFO("Ignoring '%s'", smm_asset_name(assets[i]));
			}
			if (asset == NULL)
			{
				ROS_ERROR("Asset '%s' is not currently assigned to this user", asset_name.c_str());
			}
		}
		else
		{
			ROS_ERROR("No assets found");
		}
	}

	command_pub = nh.advertise<smm_interface::Command>("command", 10);
	search_pub = nh.advertise<smm_interface::Search>("current_search", 1);

	search_begin_srv = nh.advertiseService("search/begin", search_begin_cb);
	search_completed_srv = nh.advertiseService("search/completed", search_completed_cb);

	ros::Subscriber position_sub = n.subscribe("mavros/global_position/global", 10, positionUpdated);

	ros::spin();

	smm_asset_free_assets (assets, assets_count);
}

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/WaypointSetCurrent.h"
#include "mavros_msgs/WaypointReached.h"
#include "smm_interface/Command.h"
#include "smm_interface/Search.h"
#include "smm_interface/SearchBegin.h"
#include "smm_interface/SearchCompleted.h"

enum state {
	state_unknown,
	state_awaiting_launch,
	state_airborne,
	state_searching,
	state_cmd_circle,
	state_cmd_goto,
	state_cmd_rtl,
	state_low_battery,
	state_comms_failure,
	state_landing,
	state_shutdown,
} current_state = state_unknown;

/* All the current fault/commands codes */
#define FAULT_BATTERY_LOW	0x0010
#define FAULT_COMMS_FAILURE	0x0020
#define	FAULT_COMMAND_RTL	0x1000
#define FAULT_COMMAND_GOTO	0x2000
#define FAULT_COMMAND_CIRCLE	0x4000
uint32_t current_faults_and_cmds = 0;

struct lat_lon {
	double lat;
	double lon;
} goto_position;

/* List of waypoints for the current search */
struct lat_lon *current_search = NULL;
/* How many waypoints are there in the current search */
uint32_t current_search_waypoints = 0;
/* Which way point are we up to ? */
uint32_t current_search_waypoint = 0;

/* Current altitude target in metres */
uint16_t current_alt = 50;

/* Client to talk to the autopilot */
ros::ServiceClient clientAPMode;

/**
 * Set the autopilot into AUTO mode
 */
void setModeAuto()
{
	mavros_msgs::SetMode mode;
	mode.request.base_mode = 0;
	mode.request.custom_mode = "AUTO";
	if (clientAPMode.call(mode) && mode.response.mode_sent)
	{
		ROS_INFO("Mode Changed to Auto");
	}
	else
	{
		ROS_ERROR("Failed to change mode to Auto");
	}
}

/**
 * Set the autopilot into CIRCLE mode
 */
void setModeCircle()
{
	mavros_msgs::SetMode mode;
	mode.request.base_mode = 0;
	mode.request.custom_mode = "CIRCLE";
	if (clientAPMode.call(mode) && mode.response.mode_sent)
	{
		ROS_INFO("Mode Changed to Circle");
	}
	else
	{
		ROS_ERROR("Failed to change mode to Circle");
	}
}

/**
 * Client to send WayPoints to the autopilot
 */
ros::ServiceClient clientWPPush;
/**
 * Set the autopilot to go to a specific point
 * @param pos the position to go to
 */
void setModeGoto(struct lat_lon pos)
{
	mavros_msgs::WaypointPush wppush;
	wppush.request.start_index = 0;
	mavros_msgs::Waypoint wp;
	wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
	wp.param1 = 0;
	wp.param2 = 10;
	wp.param3 = 0;
	wp.x_lat = pos.lat;
	wp.y_long = pos.lon;
	wp.z_alt = current_alt;
	wppush.request.waypoints.push_back(wp);
	mavros_msgs::Waypoint wp2;
	wp2.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	wp2.command = mavros_msgs::CommandCode::NAV_LOITER_UNLIM;
	wp2.param3 = 20;
	wp2.x_lat = pos.lat;
	wp2.y_long = pos.lon;
	wp2.z_alt = current_alt;
	wppush.request.waypoints.push_back(wp2);
	if (clientWPPush.call(wppush) && wppush.response.success)
	{
		ROS_INFO("Sent Goto");
		setModeAuto();
	}
	else
	{
		ROS_ERROR("Failed to send Goto");
	}
}

/**
 * Set the autopilot into RTL mode
 */
void setModeRTL()
{
	mavros_msgs::SetMode mode;
	mode.request.base_mode = 0;
	mode.request.custom_mode = "RTL";
	if (clientAPMode.call(mode) && mode.response.mode_sent)
	{
		ROS_INFO("Mode Changed to RTL");
	}
	else
	{
		ROS_ERROR("Failed to change mode to RTL");
	}
}

/**
 * Set the autopilot into LAND mode
 */
void setModeLand()
{
	mavros_msgs::SetMode mode;
	mode.request.base_mode = 0;
	mode.request.custom_mode = "LAND";
	if (clientAPMode.call(mode) && mode.response.mode_sent)
	{
		ROS_INFO("Mode Changed to Land");
	}
	else
	{
		ROS_ERROR("Failed to change mode to Land");
	}
}

/**
 * Client to clear the waypoints from the autopilot
 */
ros::ServiceClient clientWPClear;

/**
 * Clear all way points from the autopilot
 * Also sets the autopilot into RTL mode
 */
void clearWaypoints()
{
	/* It's not possible to clear the waypoints while in auto mode
	 * Set RTL mode, in case something goes wrong
	 */
	setModeRTL();
	mavros_msgs::WaypointClear wpclear;
	if (clientWPClear.call(wpclear) && wpclear.response.success)
	{
		ROS_INFO("Waypoints Cleared");
	}
	else
	{
		ROS_ERROR("Failed to clear waypoints");
	}
}

/**
 * Remove the GOTO waypoint
 */
void clearGoto()
{
	clearWaypoints();
}

/**
 * Client to tell the search map we have begun the search
 */
ros::ServiceClient clientSearchBegin;
/**
 * Find the next search to perform
 * @return true if a search was started, false if none found
 */
bool findNextSearch()
{
	smm_interface::SearchBegin sb;
	sb.request.max_distance = 0;
	if(clientSearchBegin.call(sb) && sb.response.success)
	{
		ROS_INFO("Started new search");
        return true;
	}
	else
	{
		/* TODO: This should actually be a delayed retry */
		ROS_ERROR("Failed to start new search");
        return false;
	}
}

/**
 * Load the search into the autopilot
 */
void SendSearchToAP()
{
	mavros_msgs::WaypointPush wppush;
	wppush.request.start_index = 0;
	/* Add a dummy first waypoint */
	{
		mavros_msgs::Waypoint wp;
		wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
		wp.param1 = 0;
		wp.param2 = 10;
		wp.param3 = 0;
		wp.x_lat = current_search[0].lat;
		wp.y_long = current_search[1].lon;
		wp.z_alt = current_alt;
		wppush.request.waypoints.push_back(wp);
	}
	for (uint32_t i = 0; i < current_search_waypoints; i++)
	{
		mavros_msgs::Waypoint wp;
		wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
		wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
		wp.param1 = 0;
		wp.param2 = 10;
		wp.param3 = 0;
		wp.x_lat = current_search[i].lat;
		wp.y_long = current_search[i].lon;
		wp.z_alt = current_alt;
		wppush.request.waypoints.push_back(wp);
	}
	if (clientWPPush.call(wppush) && wppush.response.success)
	{
		ROS_INFO("Sent Search");
		setModeAuto();
	}
	else
	{
		ROS_ERROR("Failed to send Search");
	}
}


/**
 * Implement the state machine
 * @param next_state which state to transition to
 */
void changeState(enum state next_state)
{
	if (next_state == current_state)
	{
		return;
	}
	ROS_INFO("Changing mode from %i to %i", current_state, next_state);
	switch (current_state)
	{
		case state_landing:
			/* Only allow landing->shutdown transitions */
			if (next_state == state_shutdown)
			{
				break;
			}
			return;
		case state_shutdown:
			/* Only allow shutdown->awaiting launch transitions */
			if (next_state == state_awaiting_launch)
			{
				break;
			}
			return;
		case state_low_battery:
			/* Don't allow leaving low battery, except to land */
			if (next_state == state_landing)
			{
				break;
			}
			return;
		case state_comms_failure:
			/* While the comms is failed, only go to low battery or landing */
			if ((current_faults_and_cmds & FAULT_COMMS_FAILURE) == FAULT_COMMS_FAILURE)
			{
				if (next_state == state_low_battery || next_state == state_landing)
				{
					break;
				}
				return;
			}
			break;
		case state_cmd_circle:
			/* While the circle command is set, only allow transitions to low battery */
			if ((current_faults_and_cmds & FAULT_COMMAND_CIRCLE) == FAULT_COMMAND_CIRCLE)
			{
				if (next_state == state_low_battery)
				{
					break;
				}
				return;
			}
			break;
		case state_cmd_goto:
			/* While the goto command is set, only allow transitions to low battery or comms failure */
			if ((current_faults_and_cmds & FAULT_COMMAND_GOTO) == FAULT_COMMAND_GOTO)
			{
				if (next_state == state_low_battery || next_state == state_comms_failure)
				{
					break;
				}
				return;
			}
			break;
		case state_cmd_rtl:
			/* While the return to launch command is set, only allow transitions to landing */
			if ((current_faults_and_cmds & FAULT_COMMAND_RTL) == FAULT_COMMAND_RTL)
			{
				if (next_state == state_landing)
				{
					break;
				}
				return;
			}
			break;
		default:
			/* If not prohibited, accept the transition */
			break;
	}

	current_state = next_state;

	switch (current_state)
	{
		case state_unknown:
		case state_awaiting_launch:
			clearWaypoints();
			clearGoto();
			break;
		case state_airborne:
            clearWaypoints();
            clearGoto();
			findNextSearch();
			break;
		case state_searching:
			SendSearchToAP();
			setModeAuto();
			break;
		case state_cmd_circle:
			clearWaypoints();
			clearGoto();
			setModeCircle();
			break;
		case state_cmd_goto:
			clearWaypoints();
			setModeGoto(goto_position);
			break;
		case state_cmd_rtl:
		case state_low_battery:
		case state_comms_failure:
			clearWaypoints();
			clearGoto();
			setModeRTL();
			break;
		case state_landing:
			clearWaypoints();
			clearGoto();
			setModeLand();
			break;
		case state_shutdown:
			clearWaypoints();
			clearGoto();
			break;
	}
}

/**
 * Based on the current state of things, decide what state we should be in
 */
void decideNewState()
{
	if (current_faults_and_cmds != 0)
	{
		/* There is a fault or a command to deal with */
		if (current_faults_and_cmds & FAULT_BATTERY_LOW)
		{
			changeState(state_low_battery);
		}
		else if(current_faults_and_cmds & FAULT_COMMS_FAILURE)
		{
			changeState(state_comms_failure);
		}
		else if(current_faults_and_cmds & FAULT_COMMAND_CIRCLE)
		{
			changeState(state_cmd_circle);
		}
		else if(current_faults_and_cmds & FAULT_COMMAND_GOTO)
		{
			changeState(state_cmd_goto);
		}
		else if(current_faults_and_cmds & FAULT_COMMAND_RTL)
		{
			changeState(state_cmd_rtl);
		}
		return;
	}

	if (current_search != NULL)
	{
		ROS_INFO("Search data present");
		ROS_INFO("Search completed: %i of %i", current_search_waypoint, current_search_waypoints);
		if (current_search_waypoint < current_search_waypoints)
		{
			changeState(state_searching);
			return;
		}
	}
	else
	{
		ROS_INFO("No search in progress");
	}
	changeState(state_airborne);
}

/* Client for telling the search map the search has been completed */
ros::ServiceClient clientSearchCompleted;

/**
 * Called each time we reach a new waypoint
 * @param msg the waypoint we have reached
 */
void waypointReach(const mavros_msgs::WaypointReached& msg)
{
	if (current_state == state_searching)
	{
		ROS_INFO("Reached waypoint %u of %u", msg.wp_seq, current_search_waypoints);
		if (msg.wp_seq == current_search_waypoints)
		{
			/* Search is finished */
			smm_interface::SearchCompleted sc;
			sc.request.successful = true;
			clientSearchCompleted.call(sc);
			free (current_search);
			current_search = NULL;
			current_search_waypoints = 0;
			current_search_waypoint = 0;
			decideNewState();
		}
		else
		{
			current_search_waypoint = msg.wp_seq;
		}
	}
	/* TODO: Goto, RTL, etc */
}

/**
 * Called when a new command is received from the search map
 * @param msg the new command
 */
void newCommand(const smm_interface::Command& msg)
{
	ROS_INFO("New command: %u", msg.command);
	/* Clear the command bits */
	current_faults_and_cmds &= ~(FAULT_COMMAND_CIRCLE|FAULT_COMMAND_GOTO|FAULT_COMMAND_RTL);
	switch (msg.command)
	{
		case smm_interface::Command::COMMAND_NONE:
			break;
		case smm_interface::Command::COMMAND_CIRCLE:
			current_faults_and_cmds |= FAULT_COMMAND_CIRCLE;
			break;
		case smm_interface::Command::COMMAND_RTL:
			current_faults_and_cmds |= FAULT_COMMAND_RTL;
			break;
		case smm_interface::Command::COMMAND_GOTO:
			current_faults_and_cmds |= FAULT_COMMAND_GOTO;
			goto_position.lat = msg.latitude;
			goto_position.lon = msg.longitude;
			ROS_INFO("Goto %lf %lf", goto_position.lat, goto_position.lon);
			break;
		default:
			ROS_INFO("Unknown command");
	}
	decideNewState();
}

/**
 * Called when a new search is recieved from the search map
 * @msg The new search
 */
void newSearch(const smm_interface::Search& msg)
{
	ROS_INFO("New Search: %lu, %lu, %u, %lu", msg.distance, msg.search_length, msg.sweep_width, msg.waypoints.size());
	if (current_search != NULL)
	{
		free(current_search);
	}

	current_search = NULL;
	current_search_waypoints = 0;
	current_search_waypoint = 0;

	current_search_waypoints = msg.waypoints.size();
	current_search = (struct lat_lon *) calloc (current_search_waypoints, sizeof (struct lat_lon));
	uint32_t i = 0;
	for (geometry_msgs::Point p : msg.waypoints)
	{
		if (i >= current_search_waypoints)
		{
			ROS_ERROR("size() was wrong :(");
			break;
		}
		current_search[i].lat = p.x;
		current_search[i].lon = p.y;
		i++;
	}

	decideNewState();
}

/**
 * Call this periodically to find a new search if we haven't currently got one
 * @param event The timer event that triggered this call
 */
void searchTimer(const ros::TimerEvent &event)
{
    if (current_state == state_airborne)
    {
        /* See if we can find a (new) search */
        findNextSearch();
    }
}

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "cap_controller");

	ros::NodeHandle n;
	ros::NodeHandle nh("~");

    /* Setup the clients for the autopilot */
	clientAPMode = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	clientWPClear = n.serviceClient<mavros_msgs::WaypointClear>("mavros/mission/clear");
	clientWPPush = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");
    /* Setup the clients for the search map */
	clientSearchBegin = n.serviceClient<smm_interface::SearchBegin>("smm_interface/search/begin");
	clientSearchCompleted = n.serviceClient<smm_interface::SearchCompleted>("smm_interface/search/completed");
    /* Watch for new commands from the search map */
	ros::Subscriber command_sub = n.subscribe("smm_interface/command", 1, newCommand);
    /* Watch for search being set */
	ros::Subscriber search_sub = n.subscribe("smm_interface/current_search", 1, newSearch);
    /* Watch for notification that we have reached a new waypoint */
	ros::Subscriber waypoint_sub = n.subscribe("mavros/mission/reached", 1, waypointReach);
    
    /* Set the state to awaiting launch, so we don't accidently try to fly on the ground */
	changeState(state_awaiting_launch);

    /* Setup the timer to look for a search if we don't have one */
    ros::Timer searchtimer = nh.createTimer(ros::Duration(30), searchTimer);

	ros::spin();
}

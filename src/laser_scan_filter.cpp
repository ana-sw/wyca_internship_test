#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

// *** Global variables ***

int size;
int nb_input_points;
int nb_filtered_points;
std::string type;
double inc;
double* input_ranges;
double* input_angle;
double* list_filtered_ranges;
double* list_filtered_angles;



//////////////////////////////////////////////

// *** Functions declaration ***

// Callback function:
void callback_reading_laserscan(const sensor_msgs::LaserScan::ConstPtr& msg){

    // Angles:
    double min, max, angle;
    min = msg->angle_min;
    max = msg->angle_max;
    inc = msg->angle_increment;
    angle = min;

    // Number of points:
    nb_input_points = msg->ranges.size();
    nb_filtered_points = (int) nb_input_points / size;

    // Input arrays:
    input_ranges = new double [nb_input_points];
    input_angle = new double [nb_input_points];
    for(int i = 0; i < nb_input_points; i++){
        input_ranges[i] = msg->ranges[i];
        input_angle[i] = angle;
        angle = angle + inc;
    }
}


// Average filter:
void average_filter(){

    double filtered_range;
    double filtered_angle;
    double ranges_window[size];
    double angles_window[size];
    double ranges_sum = 0;
    double angles_sum = 0;
    int i = 0;
    int j = 0;

    list_filtered_ranges = new double[nb_filtered_points];
    list_filtered_angles = new double[nb_filtered_points];

    while(j < nb_filtered_points){
        for(int k = 0; k < size; k++){
            // Creating rolling windows:
            ranges_window[k] = input_ranges[i+k];
            angles_window[k] = input_angle[i+k];

            // Summing all the elements in the windows:
            ranges_sum = ranges_sum + ranges_window[k];
            angles_sum = angles_sum + angles_window[k];
        }

        // Average:
        filtered_range = (double) ranges_sum / size;
        filtered_angle = (double) angles_sum / size;
        list_filtered_ranges[j] = filtered_range;
        list_filtered_angles[j] = filtered_angle;

        ranges_sum = 0;
        angles_sum = 0;
        i = i + size;
        j++;
    }

}


// Median filter:
void median_filter(){
    double filtered_range;
    double filtered_angle;
    int i = 0;
    int j = 0;
    int n , m;
    double ranges_window[size];
    double angles_window[size];
    double tmp;

    list_filtered_ranges = new double[nb_filtered_points];
    list_filtered_angles = new double[nb_filtered_points];

    while(j < nb_filtered_points){
        // Creating rolling windows:
        for(int k = 0; k < size; k++){
            ranges_window[k] = input_ranges[i+k];
            angles_window[k] = input_angle[i+k];
        }

        // Sorting the ranges' window:
        for(int n = 0; n < size; n++){
            for(int m = n; m < size; m++){
                if(ranges_window[m]<ranges_window[n]){
                    tmp = ranges_window[n];
                    ranges_window[n] = ranges_window[m];
                    ranges_window[m] = tmp;
                }
            }
        }

        // Selection of the two middle elements in the windows, and  averaging:
        filtered_range = (double) ((ranges_window[(int) (size/2.0)] + ranges_window[(int) (1 + size/2.0)]) / 2);
        filtered_angle = (double) ((angles_window[(int) (size/2.0)] + angles_window[(int) (1 + size/2.0)]) / 2);
        list_filtered_ranges[j] = filtered_range;
        list_filtered_angles[j] = filtered_angle;

        i = i + size;
        j++;
    }
}


// Function to create a LaserScan message:
sensor_msgs::LaserScan create_laserscan_msg(){

    sensor_msgs::LaserScan scan;
    ros::Time scan_time;
    double a_min, a_max, r_min , r_max;

    // Extraction of the min and max, for the ranges and the angles:
    a_min = list_filtered_angles[0];
    a_max = list_filtered_angles[0];
    r_min = list_filtered_ranges[0];
    r_max = list_filtered_ranges[0];
    for(int i = 1; i < nb_filtered_points; i++){
        a_min = std::min(a_min,list_filtered_angles[i]);
        a_max = std::max(a_max,list_filtered_angles[i]);
        r_min = std::min(r_min,list_filtered_ranges[i]);
        r_max = std::max(r_max,list_filtered_ranges[i]);
    }
    scan.angle_min = a_min;
    scan.angle_max = a_max;
    scan.range_min = r_min;
    scan.range_max = r_max;

    scan_time = ros::Time::now();
    scan.header.stamp = scan_time;

    scan.header.frame_id = "laser";

    scan.angle_increment = inc * size;

    scan.time_increment = 0;

    scan.ranges.resize(nb_filtered_points);
    scan.intensities.resize(nb_filtered_points);

    for(int k = 0; k < nb_filtered_points; k++){
        scan.ranges[k] = list_filtered_ranges[k];
        scan.intensities[k] = 0;
    }

    return(scan);
}


/////////////////////////////////////////////

int main(int argc, char ** argv){

    // Initialisation and local variables:
    ros::init(argc,argv,"laser_scan_filter");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);    // 10Hz = /scan topic's frequency (according to rqt topic manager)
    sensor_msgs::LaserScan filtered_scan;

    // Getting the paramerters' values set by the user:
    nh.getParam("filter_type", type);
    nh.getParam("window_size", size);

    // Shutting down in case of unacceptable parameters' values:
    if (type!="average" & type!="median"){
        ROS_ERROR("Error entering filter type. Filter type may be 'average' or 'median'.");
        ros::shutdown();
    }
    if (size!=4 & size!=8){
        ROS_ERROR("Error entering rolling window size. Rolling window size may be '4' or '8'.");
        ros::shutdown();
    }

    // Publisher and subscriber declarations:
    ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("FilteredScan", 1000);
    ros::Subscriber sub = nh.subscribe("scan", 1000, callback_reading_laserscan);

    // Filtering loop:
    while(ros::ok()){

        // Filter type choice:
        if (type=="average"){
            average_filter();
        }
        else if (type=="median"){
            median_filter();
        }

        // Creating a LaserScan message and publishing it:
        filtered_scan = create_laserscan_msg();
        pub.publish(filtered_scan);

        // Next message:
        ros::spinOnce();    // Calls callback function
        loop_rate.sleep();   // at a 10 Hz frequency
    }

    // Pointers destruction:
    delete input_ranges;
    delete input_angle;
    delete list_filtered_ranges;
    delete list_filtered_angles;

    return 0;
}

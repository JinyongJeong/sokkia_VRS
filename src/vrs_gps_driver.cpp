#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <vrs_gps_driver/vrs_gps.h>

#include <GeoCoordConv.h>
#include <boost/algorithm/string.hpp>
#include <basic_serial.h>

using namespace std;

int 
main (int argc, char *argv[])
{

    ros::init(argc, argv, "vrs_gps_driver_node");
    ros::NodeHandle nh;
    ros::Publisher vrs_gps_pub = nh.advertise<vrs_gps_driver::vrs_gps>("vrs_gps_data", 10);

    char ttydev[] = "/dev/ttyUSB-vrs";
    int brate = 115200;
    char channel[] = "vrs_gps_data";

    int fd = serial_open (ttydev, brate, 0);

    serial_set_canonical (fd);

    char buf [256];
    int len;
    memset (&buf, '\0', 256);
    
    struct pollfd poll_events;

    int poll_state;

    poll_events.fd = fd;
    poll_events.events = POLLIN|POLLERR;
    poll_events.revents = 0;
    
    //vrs data
    CGeoCoordConv CoordConv;
    CoordConv.SetSrcType(kWgs84, kGeographic);
    CoordConv.SetDstType(kBessel1984, kTmMidNew);

    bool GPGGA_flag;
    bool GNGSA_flag;
    bool GNZDA_flag;
    bool GNVTG_flag;
    char vrs_data_label[6];

    vrs_gps_driver::vrs_gps gps_data_publish;

    while(ros::ok()) {
        ros::spinOnce();
        poll_state = poll((struct pollfd*)&poll_events, 1, 10);

        if(0 < poll_state) {
            if (poll_events.revents & POLLIN) {

                ROS_INFO_STREAM("Reading from serial port");
                memset (&buf, '\0', 256);
                memset (&vrs_data_label, '\0', 5);

                len = read (fd, buf, sizeof buf);
                //get data in buf
                for(int i = 1 ; i < 6 ; i ++) vrs_data_label[i-1] = buf[i];
                vrs_data_label[5] = '\0';
                string vrs_data_label_str(vrs_data_label);

                if(vrs_data_label_str.compare("GPGGA") == 0 ){

                    gps_data_publish.header.stamp  = ros::Time::now();
                    gps_data_publish.header.frame_id = "vrs_gps";

                    string GPS_GPGGA = buf;

                    gps_data_publish.GPGGA = GPS_GPGGA;
                    GPGGA_flag = true;

                    double RTK_lat;
                    double RTK_lon;
                    int fix_state;
                    int num_of_sat;
                    double RTK_x;   //final result
                    double RTK_y;   //final result

                    //$GPGGA,045329.00,3622.5776675,N,12723.7853191,E,4,08,1.28,45.7583,M,21.3310,M,1.0,0506*42
                    std::vector<std::string> x;
                    boost::split(x,GPS_GPGGA,boost::is_any_of(","));

                    if((x[2].empty() || x[4].empty()) != 1){

                        //data conversion
                        int degree = (int)((double)std::stod(x[2]) / 100);
                        double sec = ((double)std::stod(x[2]) - degree * 100) / 60.;
                        double latitude_conv = degree + sec;

                        degree = (int)((double)std::stod(x[4]) / 100);
                        sec = ((double)std::stod(x[4]) - degree * 100) / 60.;
                        double longitude_conv = degree + sec;

                        RTK_lat = latitude_conv;
                        RTK_lon = longitude_conv;

                        cout << RTK_lat << endl;
                        cout << RTK_lon << endl;

                        fix_state = (int)std::stod(x[6]);
                        num_of_sat = (int)std::stod(x[7]);

                        CoordConv.Conv(RTK_lon, RTK_lat, RTK_x, RTK_y);
                        gps_data_publish.longitude = RTK_lon;

                        gps_data_publish.latitude = RTK_lat;

                        gps_data_publish.x_coordinate = RTK_x;
                        gps_data_publish.y_coordinate = RTK_y;

                        cout << RTK_x << endl;
                        cout << RTK_y << endl;

                        gps_data_publish.number_of_sat = num_of_sat;

                        gps_data_publish.fix_state = fix_state;
                        if(fix_state == 1){
                            gps_data_publish.fix_state_str = "normal";
                        }
                        if(fix_state == 4){
                            gps_data_publish.fix_state_str = "fix";
                        }
                        if(fix_state == 5){
                            gps_data_publish.fix_state_str = "float";
                        }

                    }
                }

                if(vrs_data_label_str.compare("GNGSA") == 0 ){
                    string GPS_GNGSA = buf;
                    gps_data_publish.GNGSA = GPS_GNGSA;
                    GNGSA_flag = true;
                }

                if(vrs_data_label_str.compare("GNZDA") == 0 ){
                    string GPS_GNZDA = buf;
                    gps_data_publish.GNZDA = GPS_GNZDA;
                    GNZDA_flag = true;
                }

                if(vrs_data_label_str.compare("GNVTG") == 0 ){
                    string GPS_GNVTG = buf;
                    gps_data_publish.GNVTG = GPS_GNVTG;
                    GNVTG_flag = true;
                }

                if(GPGGA_flag == true && GNZDA_flag == true && GNVTG_flag == true){
                    vrs_gps_pub.publish(gps_data_publish);
                    cout << "publsih" << endl;
                    GPGGA_flag = false;
                    GNGSA_flag = false;
                    GNZDA_flag = false;
                    GNVTG_flag = false;
                }
            }

            if (poll_events.revents & POLLERR) {
                printf( "receiver error!" );
                break;
            }
        }
    }

    serial_close(fd);

    return 0;
}



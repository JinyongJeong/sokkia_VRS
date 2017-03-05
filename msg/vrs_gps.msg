Header header  # standard ROS message header

string GPGGA
string GNZDA
string GNVTG
string GNGSA


float64 longitude
float64 latitude

int8 fix_state

float64 h_accuracy
float64 v_accuracy

int8 number_of_sat

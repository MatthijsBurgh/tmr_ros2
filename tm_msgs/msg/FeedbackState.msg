std_msgs/Header header

# Connection Status
bool is_svr_connected       # Ethernet connection
bool is_sct_connected       # Listen Node connection

# Response ROS Node Status
int32 tmsrv_cperr           # TM Ethernet Slave Communication Last CPERR Error Code
int32 tmscript_cperr        # External Script Communication Last CPERR Error Code
int32 tmsrv_dataerr         # TMSVR Communication Last Data Error Code
int32 tmscript_dataerr      # External Script Communication TMSCT (or TMSTA) Last Data Error Code

# Disconnection Message
int32 max_not_connect_in_s
int32 disconnection_times

# Movement
float64[6] joint_pos
float64[6] joint_vel
float64[6] joint_tor
float64[6] tool0_pose       # flange_pose
float64[6] tool_pose
float64[6] tcp_speed
float64[3] tcp_force

# Torque Data(for TM-Flow 1.84)
float64[6] joint_tor_average
float64[6] joint_tor_min
float64[6] joint_tor_max

# Status
bool robot_link              # Link Robot Controller
bool is_data_table_correct   # Data Table Setting is Correct
bool robot_error             # Error or Not
bool project_run             # Project Runing or not
bool project_pause           # Project Pause or not
bool safetyguard_a           # Safety IO(Safeguard Port A trigger)
bool e_stop                  # Emergency STOP
uint8 camera_light           # Light
int32 error_code             # Last Error Code
int32 project_speed          # Project Running Speed
int32 ma_mode                # M/A Mode
int32 robot_light            # Robot light

# IO
bool[16] cb_digital_output
bool[16] cb_digital_input
float32[2] cb_analog_output
float32[2] cb_analog_input
bool[4] ee_digital_output
bool[4] ee_digital_input
float32[2] ee_analog_output
float32[2] ee_analog_input

# String
string error_content

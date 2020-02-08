
# TODO change this to a yaml file
topics = {
        "cmd_vel_topic" : "kermit/cmd_vel", 
        "localizer_topic" : "kermit/state_localizer", 
        "path_state_topic" : "kermit/path_state", 
        "imu_topic" : "imu_message", 
        "camera_topic" : "camera",
        "tof_ping_topic" : "tof"}

nodes = {
        "Localizer_Node" : "Localizer",
        "Camera_Node" : "Camera",
        "Kernel_Node" : "Kernel",
        "Controller_Node" : "Controller"}


function hmi_ros_ip() {
    // return '192.168.2.75'; // robot ip
    //return '147.250.35.38'; // ensta ip
    return window.location.hostname; // localhost 
}

function hmi_ros_url() {
    return 'ws://' + hmi_ros_ip() + ':9090';
}

function hmi_mjpeg_host() {
    return hmi_ros_ip();
}

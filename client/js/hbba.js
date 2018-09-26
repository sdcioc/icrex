var srv_add_desires;
var srv_rem_desires;

var topic_intentions;
var hbba_intentions = undefined;

function init_hbba() {
    srv_add_desires = new ROSLIB.Service({
        ros:            ros,
        name:           '/hbba/add_desires',
        serviceType:    'hbba_msgs/AddDesires'
    });

    srv_rem_desires = new ROSLIB.Service({
        ros:            ros,
        name:           '/hbba/remove_desires',
        serviceType:    'hbba_msgs/RemoveDesires'
    });

}

function add_desire(des_id, des_type, des_params = "", des_utility = 1, des_intensity = 1) {
    console.log(ros);
    des = {
        id: des_id,
        type: des_type,
        utility: des_utility,
        intensity: des_intensity,
        params: des_params // As to be a stringified json struct, ex. "{sentence: 'hello'}".
    };

    var req = new ROSLIB.ServiceRequest({desires: [des]});
    srv_add_desires.callService(req, function(res) {});
}

function rem_desire(des_id) {
    console.log(ros);
    var req = new ROSLIB.ServiceRequest({ids: [des_id]});
    srv_rem_desires.callService(req, function(res) {});
}

// Requests Face/Look/Approach person all at once.
function start_tracking_person(source="hmi") {
    add_desire(source + "_track_person", "TrackPerson", "", 1, 10);
    add_desire(source + "_track_people", "TrackPeople", "", 1, 10);
}

function stop_tracking_person(source="hmi") {
    rem_desire(source + "_track_person");
    rem_desire(source + "_track_people");
}

// Request Face/Look person only.
function start_face_person(source="hmi") {
    add_desire(source + "_face_person",       "FacePerson",     "", 1, 10);
    add_desire(source + "_look_at_person",    "LookAtPerson",   "", 1, 10);
    add_desire(source + "_track_people",      "TrackPeople",    "", 1, 10);
}

function stop_face_person(source="hmi") {
    rem_desire(source + "_face_person");
    rem_desire(source + "_look_at_person");
    rem_desire(source + "_track_people");
}

// Create the 'back up' desire. Deletion is auto-handled by iw_observer.
function start_back_up(source="hmi") {
    add_desire(source + "_back_up", "BackUp", "", 1, 10);
}

function stop_back_up(source="hmi") {
    rem_desire(source + "_back_up");
}

function start_docking(source="hmi") {
    // add_desire(source + '_goto_dock', "", 1, 10);
    add_desire(source + "_goto_dock", "GoToDock", "", 1, 10);
    stop_tracking_person();
}

function stop_docking(source="hmi") {
    rem_desire(source + '_goto_dock');
}

function start_undock(source="hmi") {
    stop_docking();
    add_desire(source + "_undock", "Undock", "", 1, 10);
    start_tracking_person();
}

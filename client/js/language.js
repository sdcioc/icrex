
var ros = new ROSLIB.Ros({
  url : hmi_ros_url()
});

var speech_topic = new ROSLIB.Topic({
  ros: ros,
  name: '/robotSay',
  messageType: 'std_msgs/String'
});
// ///////////////////// ROS PARAMETERS /////////////////
function get_lang(prefix, fun_cb) {
  lang_param = new ROSLIB.Param({
      ros: ros,
      name: '/user_lang'
  });

  lang_param.get(function(value){
    lang_file = 'js/json/' + prefix + '_' + value + '.json';


    fun_cb(lang_file);
  });
};

function get_param(param_name, fun_cb){
  new_name = '/' + param_name
  new_param = new ROSLIB.Param({
    ros: ros,
    name: new_name
  });

  new_param.get(function(value) {
    fun_cb(param_name, value);
  });
};

// function set_param(parametru){
//   get_param(parametru, update_param);
// };

// function update_param(value){
//   return value
// };

// ///////////////////////// SPEECH ////////////////////

// function robotSpeakLanguage(message, fun_cb){
//   lang_param = new ROSLIB.Param({
//     ros: ros,
//     name: '/user_lang'
//   });

//   lang_param.get(function(value){
//     fun_cb(value, message);
//   });
// };

// function robotSay(msg){
//   robotSpeakLanguage(msg, robotSpeak);
// };

// function robotSpeak(language, msg){

//     f = function(session) {
//         session.call('komspeech.tts.speak', [{lang: language, text: msg}] )
//     };

//     komcom(f);
// };

function robotSay(msg){
  speech_topic.publish({
    data: msg
  })
}


function robotListenLanguage(messages, fun_cb){
  lang_param = new ROSLIB.Param({
    ros: ros,
    name: '/user_lang'
  });

  lang_param.get(function(value){
    fun_cb(value, messages)
  });
};

function robotHear(msgs){
  robotListenLanguage(msgs, robotListen);
};

function robotListen(language, msgs){
  f = function(session) {
    session.call('komspeech.sr.listen', [{ lang: language, grammar: msgs }]);
  };

  komcom(f);
};

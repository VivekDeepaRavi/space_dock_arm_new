// ROS connection establish
ros = new ROSLIB.Ros({
   url : 'ws://localhost:9090'
 });

//  Checking connectino status
 ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});

// Defile all Topics

  // Topic for alert status 
  var alert_listener = new ROSLIB.Topic({
    ros : ros,
    name : '/alert_listener',
    messageType : 'std_msgs/String'
  });


 // Topic for button fedback
 var button_data = new ROSLIB.Topic({
 ros: ros,
 name: "/button",
 messageType: "std_msgs/String",
});

// the function passes the button id
function buttonClick(button_id) {
 var button = new ROSLIB.Message({
   data:`${button_id}`,
 });
 button_data.publish(button); // Publishing data for getting which button pressed
}




#include <IRremote.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>

// Define IR receiver pin
const int RECV_PIN = 7;
IRrecv irrecv(RECV_PIN);
decode_results results;

// Define IR transmitter pin
IRsend irsend1(3);

// ROS setup
ros::NodeHandle nh;
std_msgs::UInt32 ir_msg;
ros::Publisher ir_pub("ir_data", &ir_msg);

// Callback function for ROS subscriber
void messageCb(const std_msgs::Int32& toggle_msg) {
  // Log received message
  nh.loginfo("Received");

  // Extract toggle value from the received message
  int toggle_value = toggle_msg.data;

  // Toggle pin state based on received value
  if (toggle_value == 1) {
    nh.loginfo("Attached");
    digitalWrite(6, HIGH);
  } else if (toggle_value == 0) {
    nh.loginfo("Unattached");
    digitalWrite(6, LOW);
  } else {
    nh.loginfo("Invalid command");
  }
}

// Create ROS subscriber
ros::Subscriber<std_msgs::Int32> sub("magnet_control", &messageCb);

// Setup function
void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize pin for magnet control
  pinMode(6, OUTPUT);

  // Initialize ROS node
  nh.initNode();

  // Subscribe to the magnet control topic
  nh.subscribe(sub);

  // Enable IR receiver
  irrecv.enableIRIn();
  irrecv.blink13(true);

  // Advertise IR data publisher
  nh.advertise(ir_pub);
}

// Loop function
void loop() {
  // Check if IR signal is received
  if (irrecv.decode(&results)) {
    // Publish the received IR data to the ROS topic
    ir_msg.data = results.value;
    ir_pub.publish(&ir_msg);

    // Resume IR receiver
    irrecv.resume();
  }

  // Example: Transmitting NEC IR command
  unsigned long necCode = 0xAABBCC;
  irsend1.sendNEC(necCode, 32);

  // Add any other actions or logic you want to perform in the loop

  // Process ROS callbacks
  nh.spinOnce();
}

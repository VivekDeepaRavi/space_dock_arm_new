
#include <IRremote.h>
#include <ros.h>
#include <std_msgs/Int32.h> // Add the necessary ROS message type for the IR data
#include <std_msgs/UInt32.h>
const int RECV_PIN = 7;
IRrecv irrecv(RECV_PIN);
decode_results results;

IRsend irsend1(3);

ros::NodeHandle nh;
std_msgs::UInt32 ir_msg;
ros::Publisher ir_pub("ir_data", &ir_msg); // Create a ROS publisher for IR data


void messageCb(const std_msgs::Int32& toggle_msg) {
  // Use ROS_INFO for logging instead of nh.loginfo
  nh.loginfo("Received");

  int toggle_value = toggle_msg.data;

  if (toggle_value == 1) {
    // Log and set the pin state in one place
    nh.loginfo("Attached");
    digitalWrite(6, HIGH);
  } else if (toggle_value == 0) {
    nh.loginfo("Unattached");
    digitalWrite(6, LOW);
  } else {
    nh.loginfo("Invalid command");
  }
}

ros::Subscriber<std_msgs::Int32> sub("magnet_control", &messageCb );

void setup() {
  Serial.begin(9600);
  //////////////////////////////

  pinMode(6, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

 //////////////////////////////
  irrecv.enableIRIn();
  irrecv.blink13(true);

  nh.initNode(); // Initialize the ROS node
  nh.advertise(ir_pub); // Advertise the publisher

  // Add any other setup code you need here
}

void loop() {
  if (irrecv.decode(&results)) {
    //Serial.println("Received NEC IR command:");
    //Serial.println(results.value, HEX);

    // Publish the received IR data to the ROS topic
    ir_msg.data = results.value;
    ir_pub.publish(&ir_msg);

    irrecv.resume();
  }

  unsigned long necCode = 0xAABBCC;
  //Serial.println("Transmitting NEC IR command...");
  irsend1.sendNEC(necCode, 32);

  // Add any other actions or logic you want to perform in the loop

  delay(1000);
  nh.spinOnce(); // Process ROS callbacks
}

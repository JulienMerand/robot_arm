#include <ros2arduino.h>


#define XRCEDDS_PORT  Serial 

void callback(std_msgs::String* msg, void* arg)
{
  (void)(arg);

  String get = msg->data;
  Serial.println(get);
  
}

class Sub : public ros2::Node{
  public:Sub():Node("ros2arduino_sub_node"){
      this->createSubscriber<std_msgs::String>("topic_arduino", (ros2::CallbackFunc)callback, nullptr);
    }
};

void setup() 
{
  XRCEDDS_PORT.begin(115200);
  while (!XRCEDDS_PORT); 

  ros2::init(&XRCEDDS_PORT);
  // pinMode(LED_BUILTIN, OUTPUT);
}

void loop() 
{
  static Sub MyNode;

  // ros2::spin(&MyNode);
}
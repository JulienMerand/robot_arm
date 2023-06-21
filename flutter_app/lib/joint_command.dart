import 'package:flutter/material.dart';
import 'package:roslibdart/roslibdart.dart';

class JointControl extends StatefulWidget {
  const JointControl({super.key});

  @override
  State<JointControl> createState() => _JointControlState();
}

class _JointControlState extends State<JointControl> {
  double J1_val = 0;
  double J2_val = 0;
  double J3_val = 0;
  double J4_val = 0;
  double J5_val = 0;
  double J6_val = 0;

  late Ros ros;
  late Topic chatter;

  // @override
  // void initState() {
  //   ros = Ros(url: 'ws://127.0.0.1:9090');
  //   chatter = Topic(
  //       ros: ros,
  //       name: '/flutter_topic_chatter',
  //       type: "std_msgs/String",
  //       reconnectOnClose: true,
  //       queueLength: 10,
  //       queueSize: 10);
  //   ros.connect();
  //   chatter.advertise();
  //   super.initState();
  // }

  // void publishChatter(double data) async {
  //   Map<String, dynamic> json = {"data": data.toString()};
  //   await chatter.publish(json);
  // }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: const Text("Joints Control"),
          backgroundColor: Colors.teal,
          leading: IconButton(
            icon: const Icon(Icons.keyboard_arrow_left),
            onPressed: () {
              Navigator.pop(context);
              // ros.close();
            },
          ),
        ),
        body: Container(
          padding: const EdgeInsets.all(20),
          child: Center(
              child: Column(
            children: [
              Text(J1_val.toString()),
              Text(J2_val.toString()),
              Text(J3_val.toString()),
              Text(J4_val.toString()),
              Text(J5_val.toString()),
              Text(J6_val.toString()),
            ],
          )),
        ),
      ),
    );
  }
}

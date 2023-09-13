import 'package:flutter/material.dart';
import 'package:roslibdart/roslibdart.dart';
import 'dart:math';

double pi = 3.141592653;

double roundDouble(double value, int places) {
  num mod = pow(10.0, places);
  return ((value * mod).round().toDouble() / mod);
}

List<double> jointlimitsLower = [
  roundDouble(-1.658 * 180 / pi, 2),
  roundDouble(-0.873 * 180 / pi, 2),
  roundDouble(-2.10 * 180 / pi, 2),
  roundDouble(-2.36 * 180 / pi, 2),
  roundDouble(-1.7 * 180 / pi, 2),
  roundDouble(-2.36 * 180 / pi, 2)
];
List<double> jointlimitsUpper = [
  roundDouble(1.658 * 180 / pi, 2),
  roundDouble(1.92 * 180 / pi, 2),
  roundDouble(2.10 * 180 / pi, 2),
  roundDouble(2.36 * 180 / pi, 2),
  roundDouble(1.7 * 180 / pi, 2),
  roundDouble(2.36 * 180 / pi, 2)
];

List<double> jointStates = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
List<double> eulerPos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

class JointData {
  final int id;
  final String name;

  late double value;
  late double lower;
  late double upper;

  JointData(this.id, this.name) {
    value = jointStates[id];
    lower = jointlimitsLower[id];
    upper = jointlimitsUpper[id];
  }

  void setValue(double val) {
    value = val;
    jointStates[id] = value;
  }
}

List joint = [
  JointData(0, "Base"),
  JointData(1, "Shoulder"),
  JointData(2, "Elbow"),
  JointData(3, "Wrist 1"),
  JointData(4, "Wrist 2"),
  JointData(5, "Wrist 3"),
];

class Control extends StatefulWidget {
  const Control({super.key});

  @override
  State<Control> createState() => _ControlState();
}

class _ControlState extends State<Control> {
  double increment = 1;

  late Ros ros;
  late Topic jointStatesPub;
  late Topic eulerPosPub;

  @override
  void initState() {
    ros = Ros(url: 'ws://127.0.0.1:9090');
    jointStatesPub = Topic(
        ros: ros,
        name: '/controller/joint_states',
        type: "std_msgs/Float32MultiArray",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);
    eulerPosPub = Topic(
        ros: ros,
        name: '/controller/euler_pos',
        type: "std_msgs/Float32MultiArray",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);
    ros.connect();
    jointStatesPub.advertise();
    eulerPosPub.advertise();
    super.initState();
  }

  void publishJoints() async {
    Map<String, dynamic> json = {"data": jointStates};
    await jointStatesPub.publish(json);
  }

  void publishPose() async {
    Map<String, dynamic> json = {"data": eulerPos};
    await eulerPosPub.publish(json);
  }

  @override
  Widget build(BuildContext context) {
    final ButtonStyle buttonStyle = ElevatedButton.styleFrom(
      backgroundColor: const Color.fromARGB(141, 33, 149, 243),
    );

    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(
          title: const Text("Joints Control"),
          backgroundColor: Colors.teal,
          leading: IconButton(
            icon: const Icon(Icons.keyboard_arrow_left),
            onPressed: () {
              Navigator.pop(context);
              ros.close();
            },
          ),
        ),
        body: Container(
          padding: const EdgeInsets.all(20),
          child: Center(
              child: Column(
            children: [
              const SizedBox(height: 20),
              Center(
                child: Row(
                  children: [
                    ElevatedButton(
                        onPressed: () {
                          setState(() {
                            increment = 0.01;
                          });
                        },
                        style: buttonStyle,
                        child: const Text("0.01")),
                    const SizedBox(width: 10),
                    ElevatedButton(
                        onPressed: () {
                          setState(() {
                            increment = 0.1;
                          });
                        },
                        style: buttonStyle,
                        child: const Text("0.1")),
                    const SizedBox(width: 10),
                    ElevatedButton(
                        onPressed: () {
                          setState(() {
                            increment = 1;
                          });
                        },
                        style: buttonStyle,
                        child: const Text("1")),
                    const SizedBox(width: 10),
                    ElevatedButton(
                        onPressed: () {
                          setState(() {
                            increment = 10;
                          });
                        },
                        style: buttonStyle,
                        child: const Text("10")),
                  ],
                ),
              ),
              const SizedBox(height: 20),
              Column(
                children: List.generate(joint.length, (index) {
                  return Row(
                    children: [
                      Text(joint[index].name),
                      IconButton(
                          onPressed: () {
                            setState(() {
                              if (joint[index].value > joint[index].lower) {
                                joint[index]
                                    .setValue(joint[index].value - increment);
                              } else {
                                joint[index].setValue(joint[index].lower);
                              }
                            });
                            publishJoints();
                          },
                          iconSize: 50,
                          icon: const Icon(Icons.arrow_left)),
                      Expanded(
                        child: SizedBox(
                          height: 30,
                          child: LinearProgressIndicator(
                            value: ((joint[index].value - joint[index].lower) /
                                (joint[index].upper - joint[index].lower)),
                            // value: 0.5,
                            backgroundColor: Colors.cyan[100],
                            valueColor: const AlwaysStoppedAnimation<Color>(
                                Colors.blue),
                          ),
                        ),
                      ),
                      IconButton(
                          onPressed: () {
                            setState(() {
                              if (joint[index].value < joint[index].upper) {
                                joint[index]
                                    .setValue(joint[index].value + increment);
                              } else {
                                joint[index].setValue(joint[index].upper);
                              }
                            });
                            publishJoints();
                          },
                          iconSize: 50,
                          icon: const Icon(Icons.arrow_right)),
                      Text(roundDouble(joint[index].value, 2).toString()),
                    ],
                  );
                }),
              ),
              const SizedBox(height: 20),
// Joysticks
              Expanded(
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Container(
                      padding: const EdgeInsets.all(0.0),
                      child: Column(
                        children: [
                          Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              SizedBox(
                                width: 30,
                                child: ElevatedButton(
                                    onPressed: () {
                                      setState(() {
                                        eulerPos[3] -= increment;
                                      });
                                      publishPose();
                                    },
                                    child: const Text("-")),
                              ),
                              IconButton(
                                onPressed: () {
                                  setState(() {
                                    eulerPos[5] += increment;
                                  });
                                  publishPose();
                                },
                                icon: const Image(
                                    image:
                                        AssetImage('assets/fleche_haut.png')),
                                iconSize: 30,
                              ),
                              SizedBox(
                                width: 30,
                                child: ElevatedButton(
                                    onPressed: () {
                                      setState(() {
                                        eulerPos[3] += increment;
                                      });
                                      publishPose();
                                    },
                                    child: const Text("+")),
                              ),
                            ],
                          ),
                          Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              IconButton(
                                onPressed: () {
                                  setState(() {
                                    eulerPos[4] -= increment;
                                  });
                                  publishPose();
                                },
                                icon: const Image(
                                    image:
                                        AssetImage('assets/fleche_gauche.png')),
                                iconSize: 30,
                              ),
                              const SizedBox(width: 50),
                              IconButton(
                                onPressed: () {
                                  setState(() {
                                    eulerPos[4] += increment;
                                  });
                                  publishPose();
                                },
                                icon: const Image(
                                    image:
                                        AssetImage('assets/fleche_droite.png')),
                                iconSize: 30,
                              ),
                            ],
                          ),
                          Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              IconButton(
                                onPressed: () {
                                  setState(() {
                                    eulerPos[5] -= increment;
                                  });
                                  publishPose();
                                },
                                icon: const Image(
                                    image: AssetImage('assets/fleche_bas.png')),
                                iconSize: 30,
                              ),
                            ],
                          ),
                        ],
                      ),
                    ),
                    const SizedBox(width: 20),
                    Container(
                      padding: const EdgeInsets.all(0.0),
                      child: Column(
                        children: [
                          Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              SizedBox(
                                width: 30,
                                child: ElevatedButton(
                                    onPressed: () {
                                      setState(() {
                                        eulerPos[2] -= increment;
                                      });
                                      publishPose();
                                    },
                                    child: const Text("-")),
                              ),
                              IconButton(
                                onPressed: () {
                                  setState(() {
                                    eulerPos[0] += increment;
                                  });
                                  publishPose();
                                },
                                icon: const Image(
                                    image:
                                        AssetImage('assets/fleche_haut.png')),
                                iconSize: 30,
                              ),
                              SizedBox(
                                width: 30,
                                child: ElevatedButton(
                                    onPressed: () {
                                      setState(() {
                                        eulerPos[2] += increment;
                                      });
                                      publishPose();
                                    },
                                    child: const Text("+")),
                              ),
                            ],
                          ),
                          Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              IconButton(
                                onPressed: () {
                                  setState(() {
                                    eulerPos[1] -= increment;
                                  });
                                  publishPose();
                                },
                                icon: const Image(
                                    image:
                                        AssetImage('assets/fleche_gauche.png')),
                                iconSize: 30,
                              ),
                              const SizedBox(width: 50),
                              IconButton(
                                onPressed: () {
                                  setState(() {
                                    eulerPos[1] += increment;
                                  });
                                  publishPose();
                                },
                                icon: const Image(
                                    image:
                                        AssetImage('assets/fleche_droite.png')),
                                iconSize: 30,
                              ),
                            ],
                          ),
                          Row(
                            mainAxisAlignment: MainAxisAlignment.center,
                            children: [
                              IconButton(
                                onPressed: () {
                                  setState(() {
                                    eulerPos[0] -= increment;
                                  });
                                  publishPose();
                                },
                                icon: const Image(
                                    image: AssetImage('assets/fleche_bas.png')),
                                iconSize: 30,
                              ),
                            ],
                          ),
                        ],
                      ),
                    ),
                  ],
                ),
              ),
            ],
          )),
        ),
      ),
    );
  }
}

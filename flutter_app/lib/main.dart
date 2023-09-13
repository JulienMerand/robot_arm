import 'package:flutter/material.dart';
import 'joint_command.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Robot Controller',
      theme: ThemeData(
        primarySwatch: Colors.teal,
      ),
      home: const MyHomePage(title: 'Robot Controller'),
    );
  }
}

class MyHomePage extends StatelessWidget {
  const MyHomePage({super.key, required this.title});

  final String title;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(title),
      ),
      body: ListView(
        primary: false,
        padding: const EdgeInsets.all(10),
        children: List.generate(exos.length, (index) {
          return Card(
            margin: const EdgeInsets.all(5.0),
            child: ListTile(
              // contentPadding:
              //     const EdgeInsets.symmetric(horizontal: 20.0, vertical: 10.0),
              title: Text(exos[index].titre),
              subtitle: Text(exos[index].soustitre),
              trailing: const Icon(Icons.keyboard_arrow_right),
              onTap: () {
                Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: exos[index].buildfnct,
                  ),
                );
              },
            ),
          );
        }),
      ),
    );
  }
}

class Exos {
  final String titre;
  final String soustitre;
  final WidgetBuilder buildfnct;

  const Exos(this.titre, this.soustitre, this.buildfnct);
}

List exos = [
  Exos("Control", "Control the robot and effector position",
      (context) => const Control()),
];

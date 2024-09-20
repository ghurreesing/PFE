import 'package:flutter/material.dart';
import 'ros_websocket.dart'; 

class OptionsScreen extends StatefulWidget {
  @override
  _OptionsScreenState createState() => _OptionsScreenState();
}

class _OptionsScreenState extends State<OptionsScreen> {
  late RosWebSocket rosWebSocket;

  // List of options representing launch files
  final List<Map<String, dynamic>> options = [
    {
      'icon': Icons.timeline_outlined,
      'name': 'Follow Line',
      'description': 'Robot follows a line using an infrared sensor.',
      'launch_file': 'line_follower_robot.launch',
      'active': false,
    },
    {
      'icon': Icons.remove_circle_outline,
      'name': 'Avoid Obstacle',
      'description': 'Robot detects and avoids obstacles using an ultrasonic sensor.',
      'launch_file': 'avoid_obstacle.launch',
      'active': false,
    },
    {
      'icon': Icons.track_changes,
      'name': 'Avoid with Line',
      'description': 'Robot avoids obstacles while following a track using both infrared and ultrasonic sensors.',
      'launch_file': 'line_with_ultrasound.launch',
      'active': false,
    },
    {
      'icon': Icons.navigation_outlined,
      'name': 'Station Navigation',
      'description': 'Robot navigates to predefined stations using a camera to scan QR codes and an infrared sensor to follow a line.',
      'launch_file': 'line_with_qr.launch',
      'active': false,
    },
  ];

  @override
  void initState() {
    super.initState();
    // Initialize WebSocket connection
     rosWebSocket = RosWebSocket('ws://1.1.17.1:9091'); 
  }

  @override
  void dispose() {
    rosWebSocket.closeConnection();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        automaticallyImplyLeading: false,
        title: Row(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(Icons.android, color: Colors.white),
            SizedBox(width: 10),
            Text('RoboControl'),
          ],
        ),
      ),
      body: ListView.builder(
        padding: EdgeInsets.all(16.0),
        itemCount: options.length,
        itemBuilder: (context, index) {
          return Container(
            margin: EdgeInsets.symmetric(vertical: 15.0),
            decoration: BoxDecoration(
              color: Colors.white,
              borderRadius: BorderRadius.circular(10),
              boxShadow: [
                BoxShadow(
                  color: Colors.grey.withOpacity(0.5),
                  spreadRadius: 3,
                  blurRadius: 5,
                  offset: Offset(0, 3),
                ),
              ],
            ),
            child: ListTile(
              leading: Icon(
                options[index]['icon'],
                size: 40,
                color: Colors.blue[900],
              ),
              title: Text(
                options[index]['name']!,
                style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold),
              ),
              subtitle: Text(options[index]['description']!),
              trailing: Switch(
                value: options[index]['active'],
                activeColor: Colors.blue[900], 
                inactiveThumbColor: Colors.grey[400], 
                inactiveTrackColor: Colors.grey[300], 
                onChanged: (bool value) async {
                  setState(() {
                    options[index]['active'] = value;
                  });

                  // Perform WebSocket interaction based on the switch value
                  if (value) {
                    // Launch selected file
                    await rosWebSocket.launchFile(options[index]['launch_file']);
                  } else {
                    // Stop selected file
                    await rosWebSocket.stopFile(options[index]['launch_file']);
                  }
                },
              ),
            ),
          );
        },
      ),
    );
  }
}

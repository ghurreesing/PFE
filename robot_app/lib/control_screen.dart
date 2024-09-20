import 'package:flutter/material.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:flutter_inappwebview/flutter_inappwebview.dart'; 
import 'ros_websocket.dart'; 
import 'dart:convert'; 
import 'package:http/http.dart' as http; 


class ControlScreen extends StatefulWidget {
  @override
  _ControlScreenState createState() => _ControlScreenState();
}

class _ControlScreenState extends State<ControlScreen> {
  bool isRobotControlActive = false;  
  late RosWebSocket rosWebSocket;     
  double linearVelocity = 0.0;       
  double angularVelocity = 0.0;       
  bool isCameraFeedActive = false;    
  String cameraStreamUrl = 'http://11.1.1.1:8080/stream?topic=/image_raw'; // MJPEG stream URL

  @override
  void initState() {
    super.initState();
    // Initialize the WebSocket connection to the ROS server
     rosWebSocket = RosWebSocket('ws://1.11.1.1:9091'); 
  }

  @override
  void dispose() {
    rosWebSocket.closeConnection();
    super.dispose();
  }


    // check if the camera stream is accessible
  Future<bool> checkCameraStream(String url) async {
    final uri = Uri.parse(url);
    const int timeoutDuration = 10; 
    const int maxRetries = 3; 

    for (int attempt = 1; attempt <= maxRetries; attempt++) {
      try {
        final response = await http.head(uri).timeout(Duration(seconds: timeoutDuration));
        if (response.statusCode == 200) {
          return true;
        }
      } catch (e) {
        print(e);
      }
      
      // If request failed, wait before retrying
      await Future.delayed(Duration(seconds: attempt * 2));
    }
    
    return false;
  }

  void _showStartDialog() {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Text('Activate Robot Control'),
          content: Text('Would you like to start controlling the robot now?'),
          actions: [
            TextButton(
              onPressed: () {
                Navigator.of(context).pop(); // close dialog
              },
              child: Text('Cancel'),
            ),
            TextButton(
              onPressed: () async {
                // close dialog 
                Navigator.of(context).pop();
                
                setState(() {
                  isRobotControlActive = true;  // start robot control
                });

                // Launch teleop_twist.launch using ROS WebSocket service
                await rosWebSocket.launchFile('teleop_twist.launch');

                // add short delay
                await Future.delayed(Duration(seconds: 1));

                // check if the camera stream is running
                bool isStreamReady = await checkCameraStream(cameraStreamUrl);
                setState(() {
                  isCameraFeedActive = isStreamReady;
                });
              },
              child: Text('OK'),
            ),
          ],
        );
      },
    );
  }

  
  void _showStopDialog() {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Text('Stop Robot Control'),
          content: Text('Are you sure you want to stop controlling the robot?'),
          actions: [
            TextButton(
              onPressed: () {
                Navigator.of(context).pop();  // Close dialog
              },
              child: Text('Cancel'),
            ),
            TextButton(
              onPressed: () async {
                // Close dialog 
                Navigator.of(context).pop();

                // Update state to stop robot control and camera feed
                setState(() {
                  isRobotControlActive = false;  // Stop robot control
                  isCameraFeedActive = false;    // Stop camera feed
                });

                // Stop teleop_twist.launch file using ROS WebSocket service
                await rosWebSocket.stopFile('teleop_twist.launch');
              },
              child: Text('Stop'),
            ),
          ],
        );
      },
    );
  }


  // Function to send the Twist message to control the robot
  void _sendTwistMessage(double linear, double angular) {
    if (!isRobotControlActive) return; 

    // Create Twist message in ROSBridge format
    final twistMessage = {
      "op": "publish",
      "topic": "/cmd_vel",  
      "msg": {
        "linear": {
          "x": -linear, 
          "y": 0.0,
          "z": 0.0
        },
        "angular": {
          "x": 0.0,
          "y": 0.0,
          "z": -angular 
        }
      }
    };

    // Send message over the WebSocket
    rosWebSocket.channel.sink.add(json.encode(twistMessage));
  }

  // Function to handle tap on "Start Camera Feed"
  void _onCameraFeedTapped() {
    if (isRobotControlActive) {
      _showStopDialog();  
    } else {
      _showStartDialog();  
    }
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
      body: Column(
        children: [
          Expanded(
            flex: 2,
            child: GestureDetector(
              onTap: _onCameraFeedTapped,  
              child: Container(
                color: Colors.black,
                child: Stack(
                  children: [
                    Center(
                      child: isCameraFeedActive
                          ? InAppWebView(
                              initialUrlRequest: URLRequest(
                                url: WebUri(Uri.parse(cameraStreamUrl).toString()), // Load  MJPEG stream
                              ),
                            )
                          : Text(
                              isRobotControlActive
                                  ? 'Loading Camera Feed...'
                                  : 'Start Camera Feed',
                              style: TextStyle(color: Colors.white, fontSize: 18),
                            ),
                    ),
                    if (isCameraFeedActive) Positioned.fill(child: GestureDetector(onTap: _onCameraFeedTapped)),
                  ],
                ),
              ),
            ),
          ),
          Expanded(
            flex: 1,
            child: Center(
              child: Container(
                width: 150,
                height: 150,
                child: Joystick(
                  mode: JoystickMode.all, // Full control mode 
                  listener: (details) {
                    setState(() {
                      // Map joystick movement to velocities
                      linearVelocity = details.y * 2;   
                      angularVelocity = details.x * 2;  
                    });

                    // Send joystick data as Twist message to ROS
                    _sendTwistMessage(linearVelocity, angularVelocity);
                  },
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}

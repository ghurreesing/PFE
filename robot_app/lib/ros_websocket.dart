import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as status;

class RosWebSocket {
  late WebSocketChannel channel;

  // Initialize and connect to WebSocket
  RosWebSocket(String url) {
    channel = WebSocketChannel.connect(
      Uri.parse(url),
    );
  }

  // Function to launch ROS file
  Future<void> launchFile(String launchFile) async {
    final launchService = getLaunchService(launchFile);
    if (launchService.isEmpty) {
      print('Invalid launch file: $launchFile');
      return;
    }

    final launchMessage = {
      'op': 'call_service',
      'service': launchService, 
    };
    channel.sink.add(json.encode(launchMessage));

    // Listen to the WebSocket stream 
    _listenToStream(launchFile);
  }


   void _listenToStream(String launchFile) {
    // Convert WebSocket stream to a broadcast stream to allow multiple listeners
    final broadcastStream = channel.stream.asBroadcastStream();

    broadcastStream.listen((response) {
      var decodedResponse = json.decode(response);

      // Ensure result is a Map before trying to access 'success'
      if (decodedResponse['result'] is Map && decodedResponse['result']['success']) {
        print('$launchFile launched successfully.');
      } else if (decodedResponse['result'] is bool && decodedResponse['result']) {
        // case where result is boolean
        print('$launchFile launched successfully.');
      } else {
        print('Failed to launch $launchFile: ${decodedResponse['result']['message'] ?? 'Unknown error'}');
      }
    }, onError: (error) {
      print('Error in WebSocket stream: $error');
    });
  }

  // Function to stop ROS file
  Future<void> stopFile(String launchFile) async {
    final stopService = getStopService(launchFile);
    if (stopService.isEmpty) {
      print('Invalid stop file: $launchFile');
      return;
    }

    final stopMessage = {
      'op': 'call_service',
      'service': stopService, 
    };
    channel.sink.add(json.encode(stopMessage));

    _listenToStream(launchFile);
  }

  // Close the WebSocket connection
  void closeConnection() {
    channel.sink.close(status.goingAway); 
  }

  // Map each launch file to correct ROS service name for launching
  String getLaunchService(String launchFile) {
    switch (launchFile) {
      case 'line_follower_robot.launch':
        return '/launch_line_follower';
      case 'avoid_obstacle.launch':
        return '/launch_obstacle_avoidance';
      case 'line_with_ultrasound.launch':
        return '/launch_line_with_avoidance';
      case 'line_with_qr.launch':
        return '/launch_camera_qr';
      case 'teleop_twist.launch':
        return '/launch_teleop_twist';
      default:
        return '';
    }
  }

  // Map each stop file to correct ROS service name for stopping
  String getStopService(String launchFile) {
    switch (launchFile) {
      case 'line_follower_robot.launch':
        return '/stop_line_follower';
      case 'avoid_obstacle.launch':
        return '/stop_obstacle_avoidance';
      case 'line_with_ultrasound.launch':
        return '/stop_line_with_avoidance';
      case 'line_with_qr.launch':
        return '/stop_camera_qr';
      case 'teleop_twist.launch':
        return '/stop_teleop_twist';
      default:
        return '';
    }
  }
}

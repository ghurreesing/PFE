import 'package:flutter/material.dart';
import 'control_screen.dart';

class HomeScreen extends StatelessWidget {
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
      body: Padding(
        padding: EdgeInsets.all(15.0),
        child: ListView(
          children: [
            Text(
              'Welcome to RoboControl',
              style: TextStyle(fontSize: 26, fontWeight: FontWeight.bold, color: Colors.blue[900]),
            ),
            SizedBox(height: 10),
            Text(
              'Take control of your PiCar-S with effortless ease! Enjoy smooth navigation with the joystick, a real-time camera view, and the freedom to switch between various operation modes',
              style: TextStyle(fontSize: 15),
              textAlign: TextAlign.left,
            ),
            SizedBox(height: 20),
            Image.network(
              'https://static-data2.manualslib.com/product-images/bdf/1484176/sunfounder-picar-s-motorized-toy-car.jpg', 
              height: 200,
            ),
            SizedBox(height: 10),
            Text(
              'About the PiCar-S:',
              style: TextStyle(fontSize: 20, fontWeight: FontWeight.bold, color: Colors.blue[900]),
            ),
            SizedBox(height: 10),
            Text(
              'The SunFounder PiCar-S is a multifunctional, Raspberry Pi-powered robot car aimed for educational and recreational use. This compact car provides a hands-on platform for learning robotics, programming, and electronics. The PiCar-S is equipped with a range of sensors, allowing users to experiment with autonomous navigation, obstacle avoidance, line following, and other features.',
              style: TextStyle(fontSize: 15),
              textAlign: TextAlign.left,
            ),
            SizedBox(height: 20),
            ElevatedButton(
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.blue[900],
                padding: EdgeInsets.symmetric(vertical: 15.0),
              ),
              child: Text('Get Started', style: TextStyle(fontSize: 18, color: Colors.white)),
              onPressed: () {
                Navigator.push(context, MaterialPageRoute(builder: (context) => ControlScreen()));
              },
            ),
          ],
        ),
      ),
    );
  }
}
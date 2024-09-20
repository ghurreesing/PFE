import 'package:flutter/material.dart';
import 'bottom_nav_bar.dart';

void main() {
  runApp(RobotControlApp());
}

class RobotControlApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'RoboControl',
      theme: ThemeData(
        primaryColor: Colors.blue[900],
        scaffoldBackgroundColor: Colors.white,
        textTheme: TextTheme(
          bodyMedium: TextStyle(fontSize: 16.0, fontFamily: 'Roboto'),
        ),
        appBarTheme: AppBarTheme(
          backgroundColor: Colors.blue[900], 
          centerTitle: true,
          elevation: 2.0,
        ), colorScheme: ColorScheme.fromSwatch().copyWith(secondary: Colors.blue[800]),
      ),
      home: MainScreen(),
    );
  }
}
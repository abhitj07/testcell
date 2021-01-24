// File: MyController.java
// Date: 26th Dec 2020
// Description: Default Controller for COMP329 Programming Assignment (2020)
// Author: Terry Payne
// Modifications:

// ==========================================================================
// The navigation component of this controller is baaed on a simple collision
// avoidance approach used within other Webots simulations.

// ==============================================================
// COMP329 2020 Programming Assignment
// ==============================================================
// The navigation of the robot is managed using the navigateRobot() method
// which modifies the speed of the motors based on detecting obstables to the
// left or right of the robot.  The prototype for the method is:
//
//   public static void navigateRobot(SensorModel sensorModel,
//                                    Motor leftMotor, Motor rightMotor)
//
// You should modify this method according to Task A within the assignment.
//
// You should not need to update the other code, but may want to modify
// it as part of testing your solution.  If you choose to change the
// World that the robot moves around, then ensure that you create a
// new map based on the defaultConfig.txt.  Note that if you create
// a new file with a differet name, then change the string in the call
// to the constructor for MapModel, with the size of the map (width
// and height)
//   - MapModel myMap = new MapModel("defaultConfig.txt", 12, 10);
//
// ==============================================================


// ==============================================================


// You may need to add other webots classes such as
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Display;
import java.util.ArrayList;


// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class MyController {

  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  
  private final static double ROBOT_DIAMETER = 71;  // Robot size as defined by the data sheet
  private final static double WHEEL_RADIUS = 20;    // 0.02m - note that parameters here should be in mm.
  private final static double AXLE_LENGTH = 52;     // 0.052m - note that parameters here should be in mm.
  
  private final static double MAX_SPEED = 6.28;     // This is 2\pi radians per sec


// ==========================================================================
  // The following provides data about each of the sensors of the ePuck, as taken
  // from the Webots ePuck Data Sheet.  Note that the sensors are not evenly positioned
  // around the robot.  See: https://cyberbotics.com/doc/guide/epuck?tab-language=java
  // Note that coordinates are given here in mm
  //
  // UPDATE: The sensor positions assume that the robot has a heading of pi/2, and assumes
  // a negative y axis (actually this is z as y refers to the height in the data sheet).
  // The following positions reflect a rotation so that the sensors are positioned relative
  // to a robot with a heading of 0.
  private static String[] psNames = {
        "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"
    };    
  private static double[] psRelX = {
        30.0, 22.0, 0.0, -30.0, -30.0, 0.0, 22.0, 32.0
    };
  private static double[] psRelY = {
        -10.0, -25.0, -31.0, -15.0, 15.0, 31.0, 25.0, 10.0
    };
  private static double[] psRelTheta = {
        -0.301, -0.801, -1.571, 3.639, 2.639, 1.571, 0.799, 0.299
    };


// ==========================================================================

  public static void navigateRobot(SensorModel sensorModel, Motor leftMotor, Motor rightMotor){
    // The following code is based on the avoid obsticle code supplied
    // by the Webots platform for the ePuck and allows the robot to wander
    // randomly around the arena
    double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 8 ; i++) {
      psValues[i] = sensorModel.getSensorValue(i);
    }

    double proxDist = 8.0;    // Distance when robot avoids obstacle
    boolean right_obstacle =
          psValues[0] < proxDist ||
          psValues[1] < proxDist ||
          psValues[2] < proxDist;
    boolean left_obstacle =
          psValues[5] < proxDist ||
          psValues[6] < proxDist ||
          psValues[7] < proxDist;
          
    // ==============================================================
    // The boolean values above are then used to determine if the robot
    // should change direction based on a nearby obstacle
      
    // initialize motor speeds at 40% of MAX_SPEED.
    double mySpeed = 0.4;
    double leftSpeed  = mySpeed * MAX_SPEED;
    double rightSpeed = mySpeed * MAX_SPEED;
    // modify speeds according to obstacles
    if (left_obstacle) {
      // turn right
      leftSpeed  = mySpeed * MAX_SPEED;
      rightSpeed = -mySpeed * MAX_SPEED;
    }
    else if (right_obstacle) {
      // turn left
      leftSpeed  = -mySpeed * MAX_SPEED;
      rightSpeed = mySpeed * MAX_SPEED;
    }
    // write actuators inputs
    leftMotor.setVelocity(leftSpeed);
    rightMotor.setVelocity(rightSpeed);
  }

// ==========================================================================
     
  public static void main(String[] args) {
  
    // ======================
    // Section A (see Readme)
    // ======================

    // create the Robot instance.
    Robot robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    // set up other robot-based parameters
    double MAX_SPEED = 6.28;        // This is 2\pi radians per sec
    int step_count = 0;             // We use this to track time in our simulation
    
    // ==============================================================
    // Load map model, motion model and particle filter
    // ==============================================================
    MapModel myMap = new MapModel("defaultConfig.txt", 12, 10);
    System.out.println(myMap.toString());
    
    Pose myPose = myMap.getCenterPose();  // Assume robot is in the center of the map
    MotionModel motionModel = new MotionModel(WHEEL_RADIUS, AXLE_LENGTH);
    
    ParticleFilter myFilter = new ParticleFilter(myMap);
    
    // ===============================================================
   
    // ==============================================================
    // Initialise Devices
    // ==============================================================
    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor motor = robot.getMotor("motorname");
    //  DistanceSensor ds = robot.getDistanceSensor("dsname");
    //  ds.enable(timeStep);

    // ==============================================================
    // Initialise mapDisplay, odometryDisplay and sensorDisplay
    Display odometryDisplay = robot.getDisplay("odometryDisplay");
    motionModel.initialiseDisplay(odometryDisplay);
    
    Display mapDisplay = robot.getDisplay("mapDisplay");
    MapView mapView = new MapView(mapDisplay, myMap, myPose);
    
    Display sensorDisplay = robot.getDisplay("sensorDisplay");
        
    // ==============================================================
    // Initialise proximity sensors
    SensorModel sensorModel = new SensorModel(8);
    for (int i = 0; i < 8; i++) {
      sensorModel.initialiseSensor(i,
                                   robot.getDistanceSensor(psNames[i]),
                                   psNames[i],
                                   new Pose(psRelX[i], psRelY[i], psRelTheta[i]),
                                   timeStep);
    }
    
    sensorModel.initialiseDisplay(sensorDisplay);

    
    // ==============================================================
    // Initialise motors and wheel sensors for odometry
    
    // get a handler to the motors and set target position to infinity (speed control)
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    
    // get a handler to the position sensors and enable them
    PositionSensor leftPositionSensor = robot.getPositionSensor("left wheel sensor");
    PositionSensor rightPositionSensor = robot.getPositionSensor("right wheel sensor");
    leftPositionSensor.enable(timeStep);    // sampling period
    rightPositionSensor.enable(timeStep);   // sampling period
    
    // Initialise the odometry of the motion model
    // motionModel.initialiseOdometry(myPose, leftPositionSensor, rightPositionSensor);
    motionModel.initialiseOdometry(myPose, leftPositionSensor.getValue(),
                                           rightPositionSensor.getValue());


    // ==============================================================
    // Main loop:
    // ==============================================================
    // - perform simulation steps until Webots is stopping the controller

    while (robot.step(timeStep) != -1) {
      step_count++;      // increment step count
      

      // ======================
      // Section B (see Readme)
      // ======================
      navigateRobot(sensorModel, leftMotor, rightMotor);

      // ======================
      // Section C (see Readme)
      // ======================
      // compute odometry and speed values
      myPose = motionModel.updateOdometry(leftPositionSensor.getValue(),
                                            rightPositionSensor.getValue());
      mapView.setPose(myPose);
      
      // ==============================================================
      // Update the particle filter
        
      // 1) Draw new samples from distribution given weights
      myFilter.resampleParticles();
       
      myFilter.resetCachedParticleset();   
           
      double eta=0.0;    // Normaliser
          
      // 2) Iterate through each of the newly sampled particles
      for (Particle p:myFilter.getParticleSet()) {
        // 2.1) Apply the motion model to the particle and add to the cache
        Particle pp = motionModel.sample_motion_model(p); 
          
        // We can test to see if the particle moves into an obstacle
        // if this is the case, then we replace with new random particles
        if(myMap.get_nearest_map_feature_dist(pp.getX(), pp.getY()) <= ROBOT_DIAMETER/2.0) {
            
          // Replace with a random particle
          pp = myFilter.generateRandomParticle(ROBOT_DIAMETER/2.0,
                           myMap.getCellWidth() * myMap.getMapWidthInCells(),
                           myMap.getCellHeight()* myMap.getMapHeightInCells());
                           
         }
          	   
        // 3) Use the sensor model to update the particle weight
        double w = sensorModel.likelihood_field_range_finder_model(pp, myMap); 	   
        pp.setWeight(w);   // update the particle with the weight based on the sensor model
        myFilter.addToCachedParticleSet(pp);
    	   
        eta+=w;          // add the weight to the normaliser
      }
      // 5) Finally replace the particles in the filter with those in the cache
      myFilter.update_particleset_with_cached(eta);        

      // ======================
      // Section D (see Readme)
      // ======================

      // Update the different displays
      mapView.paintView();
      mapView.paintParticles(myFilter.getParticleSet());       
      sensorModel.paintSensorDisplay();
      
    };

    // Enter here exit cleanup code.
  }
}

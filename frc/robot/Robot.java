// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
//Imports for Network Tables Functions
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

//Imports for photonVision Functions
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
/* 
  private PhotonCamera turretCamera;


BooleanPublisher redGoalTagDetected;  
BooleanPublisher tag2Detected;     
BooleanPublisher tag10Detected;    
BooleanPublisher tag5Detected;  
BooleanPublisher blueGoalTagDetected;
BooleanPublisher tag18Detected;
BooleanPublisher tag26Detected;
BooleanPublisher tag21Detected;

DoublePublisher angleToTag10;
DoublePublisher distanceToTag10;
DoublePublisher zToTarget10;
DoublePublisher rotToTarget10;
DoublePublisher tag10Ambiguity;
*/

precisionVision thisRobotVisionHandler;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    thisRobotVisionHandler = new precisionVision();
    thisRobotVisionHandler.setWeAreBlueAliance(false);
/* 
    //Create a Net table for our targeting
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("GoalAprilTags");
    //IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("myTags").publish();
    redGoalTagDetected  = tagsTable.getBooleanTopic("Red GOAL DETECTED").publish();
    tag2Detected        = tagsTable.getBooleanTopic("Tag2Seen").publish();
    tag10Detected       = tagsTable.getBooleanTopic("Tag10Seen").publish();
    tag5Detected        = tagsTable.getBooleanTopic("Tag5Seen").publish();

    blueGoalTagDetected = tagsTable.getBooleanTopic("Blue GOAL DETECTED").publish();
    tag18Detected       = tagsTable.getBooleanTopic("Tag18Seen").publish();
    tag26Detected       = tagsTable.getBooleanTopic("Tag26Seen").publish();
    tag21Detected       = tagsTable.getBooleanTopic("Tag21Seen").publish();

    angleToTag10        = tagsTable.getDoubleTopic("Tag10Angle").publish();
    distanceToTag10     = tagsTable.getDoubleTopic("Tag10Distance").publish();
    zToTarget10         = tagsTable.getDoubleTopic("Tag10DZ").publish();
    rotToTarget10       = tagsTable.getDoubleTopic("Tag10rot").publish();

    tag10Ambiguity      = tagsTable.getDoubleTopic("Tag10ambig").publish();

    turretCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    */

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    thisRobotVisionHandler.periodicUpdate();
/*
    boolean isRedGoalTagDetected  = false;
    boolean isTag2Detected        = false;
    boolean isTag10Detected       = false;
    boolean isTag5Detected        = false;
    boolean isBlueGoalTagDetected = false;
    boolean isTag18Detected       = false;
    boolean isTag26Detected       = false;
    boolean isTag21Detected       = false;


    var turretCAMResults = turretCamera.getAllUnreadResults();
    if (!turretCAMResults.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var aTurretCAMResult= turretCAMResults.get(turretCAMResults.size() - 1);
      if (aTurretCAMResult.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : aTurretCAMResult.getTargets()) {
          switch(target.getFiducialId()){
              case 2:
                //Tag 2 was detected
                isTag2Detected = true;

                
                break;
              case 10:

                if(target.poseAmbiguity<0.2){
                  isTag10Detected = true;
                  angleToTag10.set(target.getYaw());                          //Display the angle to target 1`0
                  distanceToTag10.set(target.getBestCameraToTarget().getX());
                  zToTarget10.set(target.getBestCameraToTarget().getZ() * 180/3.14159 );
                  rotToTarget10.set(target.getBestCameraToTarget().getRotation().getAngle());

                }
                
                tag10Ambiguity.set(target.poseAmbiguity );

                break;

              case 5:
                isTag5Detected = true;


                break;
              case 18:
                isTag18Detected = true;
                break;
              case 26:
                isTag26Detected = true;

                break;
              case 21:
                isTag21Detected = true;

                break;
              default:

          }//End of switch
        }//End of iterating through targets
      }//End of check for some targets
    }//end of is empty

  
  isRedGoalTagDetected = isTag2Detected || isTag10Detected || isTag5Detected ;
  isBlueGoalTagDetected = isTag18Detected || isTag26Detected || isTag21Detected ;

  redGoalTagDetected.set(  isRedGoalTagDetected   );
  tag2Detected.set(        isTag2Detected         );        
  tag10Detected.set(       isTag10Detected        );      
  tag5Detected.set(        isTag5Detected         );
  blueGoalTagDetected.set( isBlueGoalTagDetected  ); 
  tag18Detected.set(       isTag18Detected        );       
  tag26Detected.set(       isTag26Detected        );           
  tag21Detected.set(       isTag21Detected        );       
*/

  }


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

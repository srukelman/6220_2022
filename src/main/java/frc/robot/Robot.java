// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;
import com.revrobotics.ColorSensorV3;
import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.cscore.*;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_chooser_2 = new SendableChooser<>();
  private VictorSPX LDriveV1 = new VictorSPX(8); //1st Left Drive Motor
  private VictorSPX LDriveV2 = new VictorSPX(4); //2nd Left Drive Motor
  private VictorSPX RDriveV1 = new VictorSPX(5); //1st Right Drive Motor
  private VictorSPX RDriveV2 = new VictorSPX(7); //2nd Right Drive Motor
  //private VictorSPX Intake = new VictorSPX(); //Intake Motor
  //private VictorSPX ConveyorBelt = new VictorSPX(); //Conveyor Belt Motor
  //private VictorSPX Flywheel = new VictorSPX(); //Flywheel Motor
  private XboxController xbox; // XBOX Controller
  private XboxController xbox2; //xbox
  private TalonSRX Outtake = new TalonSRX(1); //outtake motor
  private TalonSRX Intake = new TalonSRX(3);
  private final I2C.Port i2cPort = I2C.Port.kOnboard; //Color Sensor Port Object
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); //Color Sensor Object
  private final ColorMatch m_colorMatcher = new ColorMatch(); //Color Matcher Object (matches Colors with Color Sensor Output)
  private final Color blue = Color.kBlue; //The Color Blue
  private final Color red = Color.kRed; //The Color Red
  private Counter m_encoder_1; //Left Wheel Rotation Counter
  private Counter m_encoder_2; //Right Wheel Rotation Counter
  //private String previousColor = ""; //usteless atm
  private boolean safety = true;
  private boolean outtakeSlow = false;
  private boolean outtakeFast = false;
  private boolean outtakeOn = false;
  private boolean intakeOn = false;
  private Thread m_visionThread_limelight;
  private boolean isTankControls = false;
  private Thread m_visionThread_webcam;
  private int forward = 1;
  private double leftTurn;
  private double rightTurn;
  private boolean intakeFast = false;
  private double intakeSpeed = -.4;
  private Timer autonTimer;
  private double autonSpeed = .5;
  private double corrector = .85;
  ;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Drive", kDefaultAuto);
    m_chooser.addOption("Drive-Turn-Drive", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("distanceToDrive", .85);
    SmartDashboard.putBoolean("Intake Fast", intakeFast);
    SmartDashboard.putBoolean("Safety", safety);
    //js1 = new Joystick(0);
    //js2 = new Joystick(1);
    xbox = new XboxController(2);//XBOX Controller is the 3rd controller after the 2 joysticks
    xbox2 = new XboxController(3);
    m_colorMatcher.addColorMatch(blue); //adds the blue profile to the color matcher
    m_colorMatcher.addColorMatch(red); // adds the red profile to the color matcher
    // m_encoder_1 = new Counter(); // instantiates a new counter for the first encoder
    // m_encoder_1.setUpSource(2);
    // m_encoder_1.setUpDownCounterMode();
    // m_encoder_1.setDistancePerPulse(Math.PI*6); //distance per pulse PI*diameter of wheel (6" in this case)
    // m_encoder_1.setMaxPeriod(.2); //max period to determine if stopped
    // m_encoder_1.reset();
    // m_encoder_2 = new Counter(); //instantiates a new counter for the second encoder
    // m_encoder_2.setUpSource(3);
    // m_encoder_2.setUpDownCounterMode();
    // m_encoder_2.setDistancePerPulse(Math.PI*6); //distance per pulse PI*diameter of wheel (6" in this case)
    // m_encoder_2.setMaxPeriod(.2); //max period to determine if stopped
    // m_encoder_2.reset();

    m_visionThread_limelight =
        new Thread(
          () -> {
            UsbCamera limelight = CameraServer.startAutomaticCapture();
            limelight.setResolution(1000, 750);
            // CvSink cvSink = CameraServer.getVideo();
            // CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
            // Mat mat = new Mat();

            // while(!Thread.interrupted()){
            //   if(cvSink.grabFrame(mat) == 0){
            //     outputStream.notifyError(cvSink.getError());
            //     continue;
            //   }
            //   Imgproc.rectangle(mat, new Point(100,100), new Point(400,400), new Scalar(255,255,255),5);
            //   outputStream.putFrame(mat);
            // }

            
          }
        );
        UsbCamera c = CameraServer.startAutomaticCapture();
        MjpegServer s = CameraServer.addServer("http://roborio-6220-frc.local:1181/?action=stream");
        s.setSource(c);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // int count1= m_encoder_1.get(); //get the number of encoder counts
    // double dist1 = m_encoder_1.getDistance(); //get the distance the wheel has spun
    // double rate1 = m_encoder_1.getRate(); //get the rate at which the wheel is spinning
    // SmartDashboard.putNumber("Encoder Counts", count1);
    // SmartDashboard.putNumber("Distance", dist1);
    // SmartDashboard.putNumber("Velocity (in/s)", rate1);
    // int count2= m_encoder_2.get(); //get the number of encoder counts
    // double dist2 = m_encoder_2.getDistance(); //get the distance the wheel has spun
    // double rate2 = m_encoder_2.getRate(); //get the rate at which the wheel is spinning
    // SmartDashboard.putNumber("Encoder Counts", count2);
    // SmartDashboard.putNumber("Distance", dist2);
    // SmartDashboard.putNumber("Velocity (in/s)", rate2);
    corrector = SmartDashboard.getNumber("distanceToDrive",  .85);
    Color detectedColor = m_colorSensor.getColor();
    String colorString; 
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);  
    if (match.color == blue) {
      colorString = "Blue";
    } else if (match.color == red) {
      colorString = "Red";
    } else {
      colorString = "Unknown";
    }
    
    //System.out.println(colorString);
    //previousColor = colorString;
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kCustomAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autonTimer = new Timer();
    autonTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //double corrector = .8;
    autonThree();//shoot then move then intake: takes 7.2 seconds
    //autonTwo();//shoot then move: takes 12.5 seconds
    //autonOne();//move: takes 3.5 seconds
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
  }

 /** This function is called periodically during operator control. */
 @Override
 public void teleopPeriodic() {
   if(safety && !isTankControls){//safety is off and using arcade controls
    //  //turn modifiers (high is more, left is negative)
    //  leftTurn = 0;
    //  rightTurn = 0;
      
    //  if(xbox.getRawAxis(1) < .2 || xbox.getRawAxis(1) > .2){ //going forward and backward
    //    leftTurn -= .75*xbox.getRawAxis(1);
    //    rightTurn += .75*xbox.getRawAxis(1);
    //  }
    //  if(xbox.getRawAxis(4) < -.2 || xbox.getRawAxis(4) > .2){ //turning
    //    leftTurn += .25*xbox.getRawAxis(4);
    //    rightTurn += .25*xbox.getRawAxis(4);
       
    //  }
    
    //  //move left
    //  LDriveV1.set(ControlMode.PercentOutput, leftTurn*forward / corrector);
    //  LDriveV2.set(ControlMode.PercentOutput, leftTurn*forward / corrector);
    //  //move right
    //  RDriveV1.set(ControlMode.PercentOutput, rightTurn*forward * corrector);
    //  RDriveV2.set(ControlMode.PercentOutput, rightTurn*forward * corrector);
    double speed = -xbox.getRawAxis(1) * 0.6;
    double turn = xbox.getRawAxis(4) * 0.3;

    double left = (speed + turn) * corrector;//slow down left motors to make driving straighter
    double right = (speed - turn);
    if(xbox.getRawAxis(1)< .25 && xbox.getRawAxis(1) > -.25){
      left = (speed + turn);
      right = (speed - turn) * 1.5;
    }
    //set motors according to xbox input
    LDriveV1.set(ControlMode.PercentOutput, left);
    LDriveV2.set(ControlMode.PercentOutput, left);
    RDriveV1.set(ControlMode.PercentOutput, -right);
    RDriveV2.set(ControlMode.PercentOutput, -right);
    //Outtake.set(ControlMode.PercentOutput, xbox.getRawAxis(3));

   } else if(safety) {

       if(!(xbox.getRawAxis(1)<.2 && xbox.getRawAxis(1)>-.2)){
        LDriveV1.set(ControlMode.PercentOutput, xbox.getRawAxis(1) * -1 * .9);
        LDriveV2.set(ControlMode.PercentOutput, xbox.getRawAxis(1) * -1 * .9);
      } else {
        LDriveV1.set(ControlMode.PercentOutput, 0);
        LDriveV2.set(ControlMode.PercentOutput, 0);
     }

     if(!(xbox.getRawAxis(5)<.2 && xbox.getRawAxis(5)>-.2)){
       RDriveV1.set(ControlMode.PercentOutput, xbox.getRawAxis(5));
       RDriveV2.set(ControlMode.PercentOutput, xbox.getRawAxis(5));
     } else {
       RDriveV1.set(ControlMode.PercentOutput, 0);
       RDriveV2.set(ControlMode.PercentOutput, 0);
     }

   } else {
     LDriveV1.set(ControlMode.PercentOutput, 0);
     LDriveV2.set(ControlMode.PercentOutput, 0);
     RDriveV1.set(ControlMode.PercentOutput, 0);
     RDriveV2.set(ControlMode.PercentOutput, 0);
     Outtake.set(ControlMode.PercentOutput, 0);
     Intake.set(ControlMode.PercentOutput, 0);
   }

   //intake on
   if(xbox2.getRawAxis(2) > .3){
     Intake.set(ControlMode.PercentOutput, -.5);
   } else {
     Intake.set(ControlMode.PercentOutput, 0);
   }

   //outtake controls
   Outtake.set(ControlMode.PercentOutput, xbox2.getRawAxis(3));


 if(xbox2.getRightBumper()){
   Outtake.set(ControlMode.PercentOutput, -.5);
 }
 if(xbox2.getLeftBumper()){
   Intake.set(ControlMode.PercentOutput, .7);
 }


 //intake controllers
 if(xbox.getAButtonPressed()){
   forward *= -1;
 }
 if(xbox.getBButtonPressed()){
   safety = !safety;
 }
 if(xbox.getYButtonPressed()){
   isTankControls = !isTankControls;
 }

}




  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public void autonOne(){
    int firstRamp = 1;//1
    double leave = (double)firstRamp + 1.5;//2.5
    double firstDown = leave + 1;//3.5
    //speed up forward
    if(autonTimer.get()< firstRamp){
      RDriveV1.set(ControlMode.PercentOutput, (autonTimer.get()) * autonSpeed * corrector * -1);
      RDriveV2.set(ControlMode.PercentOutput, (autonTimer.get()) * autonSpeed * corrector * -1);
      LDriveV1.set(ControlMode.PercentOutput, (autonTimer.get()) * autonSpeed);
      LDriveV2.set(ControlMode.PercentOutput, (autonTimer.get()) * autonSpeed);
    }
    //drive forward
    if(autonTimer.get() >= firstRamp && autonTimer.get() < leave){
      RDriveV1.set(ControlMode.PercentOutput, autonSpeed * corrector * -1);
      RDriveV2.set(ControlMode.PercentOutput, autonSpeed * corrector * -1);
      LDriveV1.set(ControlMode.PercentOutput, autonSpeed);
      LDriveV2.set(ControlMode.PercentOutput, autonSpeed);
    }
    //slow down forward
    if(autonTimer.get() >= leave && autonTimer.get() < firstDown){
      RDriveV1.set(ControlMode.PercentOutput, (firstDown - autonTimer.get()) * autonSpeed * corrector * -1);
      RDriveV2.set(ControlMode.PercentOutput, (firstDown - autonTimer.get()) * autonSpeed * corrector * -1);
      LDriveV1.set(ControlMode.PercentOutput, (firstDown - autonTimer.get()) * autonSpeed);
      LDriveV2.set(ControlMode.PercentOutput, (firstDown - autonTimer.get()) * autonSpeed);
    }
    //stop
    else{
      RDriveV1.set(ControlMode.PercentOutput, 0);
      RDriveV2.set(ControlMode.PercentOutput, 0);
      LDriveV1.set(ControlMode.PercentOutput, 0);
      LDriveV2.set(ControlMode.PercentOutput, 0);
      Outtake.set(ControlMode.PercentOutput, 0);
      Intake.set(ControlMode.PercentOutput, 0);
    }
  }

  public void autonTwo(){
    autonSpeed = .45;
    //int wait = 2;//2 
    // int firstRamp = 1;//1
    // int ramHub = firstRamp + 2;//5
    double rampShoot =.5;
    double shoot = rampShoot + 1;//1.5
    double wait = rampShoot + 7;//8.5
    double secondRamp = shoot + 1;//9.5
    double leave = secondRamp + 2;//11.5
    double secondDown = leave + 1;//12.5
    // //speed up backward
    // if(autonTimer.get() >= wait && autonTimer.get() < firstRamp){
    //   RDriveV1.set(ControlMode.PercentOutput, autonTimer.get() * autonSpeed * corrector);
    //   RDriveV2.set(ControlMode.PercentOutput, autonTimer.get() * autonSpeed * corrector);
    //   LDriveV1.set(ControlMode.PercentOutput, autonTimer.get() * autonSpeed * -1 / corrector);
    //   LDriveV2.set(ControlMode.PercentOutput, autonTimer.get() * autonSpeed * -1 / corrector);
    // }
    // //drive backward
    // if(autonTimer.get() >= firstRamp && autonTimer.get() < ramHub){
    //   RDriveV1.set(ControlMode.PercentOutput, autonSpeed * corrector);
    //   RDriveV2.set(ControlMode.PercentOutput, autonSpeed * corrector);
    //   LDriveV1.set(ControlMode.PercentOutput, autonSpeed * -1 / corrector);
    //   LDriveV2.set(ControlMode.PercentOutput, autonSpeed * -1 / corrector);
    // }
    //if(autonTimer.get() >= ramHub && autonTimer.get() < rampShoot){
    //speed up belt
    if(autonTimer.get() < rampShoot){
      Outtake.set(ControlMode.PercentOutput, 2*(autonTimer.get()));
    }
    //full speed belt
    if(autonTimer.get() >= rampShoot && autonTimer.get() < shoot){
      Outtake.set(ControlMode.PercentOutput, 1);
      RDriveV1.set(ControlMode.PercentOutput, 0);
      RDriveV2.set(ControlMode.PercentOutput, 0);
      LDriveV1.set(ControlMode.PercentOutput, 0);
      LDriveV2.set(ControlMode.PercentOutput, 0);
    }
    //stop and shoot
    // if(autonTimer.get() >= ramHub && autonTimer.get() < shoot){
    //   Outtake.set(ControlMode.PercentOutput, 1);
    //   RDriveV1.set(ControlMode.PercentOutput, 0);
    //   RDriveV2.set(ControlMode.PercentOutput, 0);
    //   LDriveV1.set(ControlMode.PercentOutput, 0);
    //   LDriveV2.set(ControlMode.PercentOutput, 0);
    // }
    //wait for other bots to get out of the way
    if(autonTimer.get() >= shoot && autonTimer.get() < wait){
      Outtake.set(ControlMode.PercentOutput, 0);
      RDriveV1.set(ControlMode.PercentOutput, 0);
      RDriveV2.set(ControlMode.PercentOutput, 0);
      LDriveV1.set(ControlMode.PercentOutput, 0);
      LDriveV2.set(ControlMode.PercentOutput, 0);
      Outtake.set(ControlMode.PercentOutput, 0);
      Intake.set(ControlMode.PercentOutput, 0);
    }
    //speed up forward (away from hub)
    if(autonTimer.get() >= wait && autonTimer.get() <= secondRamp){
      RDriveV1.set(ControlMode.PercentOutput, (autonTimer.get()-shoot) * autonSpeed  * -1);
      RDriveV2.set(ControlMode.PercentOutput, (autonTimer.get()-shoot) * autonSpeed  * -1);
      LDriveV1.set(ControlMode.PercentOutput, (autonTimer.get()-shoot) * autonSpeed* corrector);
      LDriveV2.set(ControlMode.PercentOutput, (autonTimer.get()-shoot) * autonSpeed* corrector);
    }
    //drive forward
    if(autonTimer.get() >= secondRamp && autonTimer.get()<leave){
      RDriveV1.set(ControlMode.PercentOutput, autonSpeed  * -1);
      RDriveV2.set(ControlMode.PercentOutput, autonSpeed  * -1);
      LDriveV1.set(ControlMode.PercentOutput, autonSpeed * corrector);
      LDriveV2.set(ControlMode.PercentOutput, autonSpeed * corrector);
    }
    //slow down
    if(autonTimer.get() >= leave && autonTimer.get()<secondDown){
      RDriveV1.set(ControlMode.PercentOutput, (secondDown - autonTimer.get()) * autonSpeed * -1);
      RDriveV2.set(ControlMode.PercentOutput, (secondDown - autonTimer.get()) * autonSpeed * -1);
      LDriveV1.set(ControlMode.PercentOutput, (secondDown - autonTimer.get()) * autonSpeed * corrector);
      LDriveV2.set(ControlMode.PercentOutput, (secondDown - autonTimer.get()) * autonSpeed* corrector) ;
    }
    //stop
    if(autonTimer.get() >= secondDown){
      RDriveV1.set(ControlMode.PercentOutput, 0);
      RDriveV2.set(ControlMode.PercentOutput, 0);
      LDriveV1.set(ControlMode.PercentOutput, 0);
      LDriveV2.set(ControlMode.PercentOutput, 0);
      Outtake.set(ControlMode.PercentOutput, 0);
      Intake.set(ControlMode.PercentOutput, 0);
    }
  }
  public void autonThree(){
    autonSpeed = .45;
    double firstSpeed = .6;
    // int wait = 2;//2 
    // int firstRamp = wait+ 1;//1
    // int ramHub = firstRamp + 2;//5
    double rampShoot =.5;
    double shoot = rampShoot + 1;//1.5
    double secondRamp = shoot + .75;//2.25
    double leave = secondRamp + .2;//2.45
    double stopFast = leave + .5;//2.95
    double rampUp = stopFast + .5;//3.45
    double pickup = rampUp + 1;//4.45
    double secondDown = pickup + .75;//5.2
    double finishIntake = secondDown + 2;//7.2
    double tempcorr = .9;
    // //speed up backward
    // if(autonTimer.get() >= wait && autonTimer.get() < firstRamp){
    //   RDriveV1.set(ControlMode.PercentOutput, autonTimer.get() * autonSpeed * corrector);
    //   RDriveV2.set(ControlMode.PercentOutput, autonTimer.get() * autonSpeed * corrector);
    //   LDriveV1.set(ControlMode.PercentOutput, autonTimer.get() * autonSpeed * -1 / corrector);
    //   LDriveV2.set(ControlMode.PercentOutput, autonTimer.get() * autonSpeed * -1 / corrector);
    // }
    // //drive backward
    // if(autonTimer.get() >= firstRamp && autonTimer.get() < ramHub){
    //   RDriveV1.set(ControlMode.PercentOutput, autonSpeed * corrector);
    //   RDriveV2.set(ControlMode.PercentOutput, autonSpeed * corrector);
    //   LDriveV1.set(ControlMode.PercentOutput, autonSpeed * -1 / corrector);
    //   LDriveV2.set(ControlMode.PercentOutput, autonSpeed * -1 / corrector);
    // }
    //if(autonTimer.get() >= ramHub && autonTimer.get() < rampShoot){
    if(autonTimer.get() < rampShoot){
      Outtake.set(ControlMode.PercentOutput, 2*(autonTimer.get()));
    }
    if(autonTimer.get() >= rampShoot && autonTimer.get() < shoot){
      Outtake.set(ControlMode.PercentOutput, 1);
      RDriveV1.set(ControlMode.PercentOutput, 0);
      RDriveV2.set(ControlMode.PercentOutput, 0);
      LDriveV1.set(ControlMode.PercentOutput, 0);
      LDriveV2.set(ControlMode.PercentOutput, 0);
    }
    //stop and shoot
    // if(autonTimer.get() >= ramHub && autonTimer.get() < shoot){
    //   Outtake.set(ControlMode.PercentOutput, 1);
    //   RDriveV1.set(ControlMode.PercentOutput, 0);
    //   RDriveV2.set(ControlMode.PercentOutput, 0);
    //   LDriveV1.set(ControlMode.PercentOutput, 0);
    //   LDriveV2.set(ControlMode.PercentOutput, 0);
    // }
    //stop shooting and speed up forward
    if(autonTimer.get() >= shoot && autonTimer.get() < secondRamp){
      Outtake.set(ControlMode.PercentOutput, 0);
      RDriveV1.set(ControlMode.PercentOutput, (autonTimer.get()-shoot) * firstSpeed  * -1);
      RDriveV2.set(ControlMode.PercentOutput, (autonTimer.get()-shoot) * firstSpeed  * -1);
      LDriveV1.set(ControlMode.PercentOutput, (autonTimer.get()-shoot) * firstSpeed * tempcorr);
      LDriveV2.set(ControlMode.PercentOutput, (autonTimer.get()-shoot) * firstSpeed * tempcorr);
    }
    //drive forward
    if(autonTimer.get() >= secondRamp && autonTimer.get()<leave){
      RDriveV1.set(ControlMode.PercentOutput, firstSpeed  * -1);
      RDriveV2.set(ControlMode.PercentOutput, firstSpeed  * -1);
      LDriveV1.set(ControlMode.PercentOutput, firstSpeed * tempcorr);
      LDriveV2.set(ControlMode.PercentOutput, firstSpeed * tempcorr);
    }
    //stop driving to lower intake
    if(autonTimer.get() >= leave && autonTimer.get() < stopFast){
      RDriveV1.set(ControlMode.PercentOutput, 0);
      RDriveV2.set(ControlMode.PercentOutput, 0);
      LDriveV1.set(ControlMode.PercentOutput, 0);
      LDriveV2.set(ControlMode.PercentOutput, 0);
    }
    //start driving again while spinnning intake
    if(autonTimer.get() >= stopFast && autonTimer.get() < pickup){
      Intake.set(ControlMode.PercentOutput, -.5);
      RDriveV1.set(ControlMode.PercentOutput, (autonTimer.get()-stopFast) * autonSpeed  * -1);
      RDriveV2.set(ControlMode.PercentOutput, (autonTimer.get()-stopFast) * autonSpeed  * -1);
      LDriveV1.set(ControlMode.PercentOutput, (autonTimer.get()-stopFast) * autonSpeed* tempcorr);
      LDriveV2.set(ControlMode.PercentOutput, (autonTimer.get()-stopFast) * autonSpeed* tempcorr);
    }
    //slow down
    if(autonTimer.get() >= pickup && autonTimer.get()<secondDown){
      RDriveV1.set(ControlMode.PercentOutput, (secondDown - autonTimer.get()) * autonSpeed * -1);
      RDriveV2.set(ControlMode.PercentOutput, (secondDown - autonTimer.get()) * autonSpeed * -1);
      LDriveV1.set(ControlMode.PercentOutput, (secondDown - autonTimer.get()) * autonSpeed * tempcorr);
      LDriveV2.set(ControlMode.PercentOutput, (secondDown - autonTimer.get()) * autonSpeed * tempcorr) ;
    }
    //stop driving and keep intake running
    if(autonTimer.get() >= secondDown && autonTimer.get() < finishIntake){
      RDriveV1.set(ControlMode.PercentOutput, 0);
      RDriveV2.set(ControlMode.PercentOutput, 0);
      LDriveV1.set(ControlMode.PercentOutput, 0);
      LDriveV2.set(ControlMode.PercentOutput, 0);
    }
    //stop all
    if(autonTimer.get() >= finishIntake){
      RDriveV1.set(ControlMode.PercentOutput, 0);
      RDriveV2.set(ControlMode.PercentOutput, 0);
      LDriveV1.set(ControlMode.PercentOutput, 0);
      LDriveV2.set(ControlMode.PercentOutput, 0);
      Outtake.set(ControlMode.PercentOutput, 0);
      Intake.set(ControlMode.PercentOutput, 0);
    }
  }
}

 
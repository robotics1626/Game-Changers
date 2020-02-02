package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// Below: Limelight Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// Color sensor imports
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

import java.lang.reflect.InvocationTargetException;

// Talon SRX and FX imports 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // public static OI m_oi;
  private WPI_TalonSRX RobotShooter;

  // defining controllers
  private XboxController xbox;
  private Joystick driverLeft;
  private Joystick driverRight;
  private DriverStation driverStation;

  // defining tank drive
  private TalonFX frontLeftSpeed;
  private TalonFX frontRightSpeed;
  private TalonFX backLeftSpeed;
  private TalonFX backRightSpeed;
  private boolean toggleLT;
  private boolean toggleRT;

  // air pressure
  private boolean gearShift;
  private DoubleSolenoid shifter;
  private Compressor compressor;
  private AnalogInput pressureSensor;

  // defining color sensor
  private TalonSRX colorMotor;

  // belt
  private double beltSpeed;
  private TalonSRX beltMotor;

  // turret spinner
  private TalonSRX spinMotor;

  // defining Limelight Variables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  private boolean toggleY;

  // defining color sensor variables
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private int color = 0;
  private boolean toggleX;
  private int colorrecord = 0;
  private String fieldColor = DriverStation.getInstance().getGameSpecificMessage();

  // action recorder
  private ActionRecorder actions;

  // toggles

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // drive train assignments
    frontLeftSpeed = new TalonFX(1);
    frontRightSpeed = new TalonFX(2);
    backLeftSpeed = new TalonFX(3);
    backRightSpeed = new TalonFX(4);

    // color sensor
    colorMotor = new TalonSRX(5);

    // I beat my kids with a
    beltMotor = new TalonSRX(6);
    beltSpeed = -1;

    // turret spinner
    spinMotor = new TalonSRX(7);

    // controller and joystick init
    driverLeft = new Joystick(0);
    driverRight = new Joystick(1);
    xbox = new XboxController(2);

    // input declare
    DriverInput.nameInput("LDrive");
    DriverInput.nameInput("RDrive");
    DriverInput.nameInput("AButton");

    // pressure declares
    gearShift = true;
    shifter = new DoubleSolenoid(0, 1);
    pressureSensor = new AnalogInput(0);
    compressor = new Compressor();
    compressor.start();

    // toggle declare
    toggleX = true;
    toggleY = true;
    toggleLT = true;
    toggleRT = true;

    // action recorder setup
    actions = new ActionRecorder().setMethod(this, "robotOperation", DriverInput.class).setUpButton(xbox, 1)
        .setDownButton(xbox, 2).setRecordButton(xbox, 3);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //pressure check
    double pressure = (250 * (pressureSensor.getVoltage() / 5.0)) - 13;
    SmartDashboard.putString("Pressure Sensor", String.format("%.0f", pressure));

    // limelight variable check
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // limelight target distance from crosshair in pixels
    double xydistance = Math.sqrt(x * x + y * y);

    // pushing limelight vars
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // defining color vars
    Color detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    try {
      actions.input(new DriverInput()
          .withInput("LDrive", driverLeft.getRawAxis(1)) // used - drives left side
          .withInput("RDrive", driverRight.getRawAxis(1)) // used - drives right side
          .withInput("AButton", xbox.getAButton()) // used - test color
          .withInput("XButton", xbox.getXButton()) // used - color sensor
      );
    } catch (IllegalAccessException e) {
      e.printStackTrace();
    } catch (IllegalArgumentException e) {
      e.printStackTrace();
    } catch (InvocationTargetException e) {
      e.printStackTrace();
    }
  }
  public void robotOperation(DriverInput input){
    // drive train
    double leftaxis = driverLeft.getRawAxis(1);
    if(Math.abs(leftaxis) > .13){
      frontLeftSpeed.set(ControlMode.PercentOutput, (leftaxis * -1 ));
      backLeftSpeed.set(ControlMode.PercentOutput, (leftaxis * -1));  
    } else {
      frontLeftSpeed.set(ControlMode.PercentOutput, (0));
      backLeftSpeed.set(ControlMode.PercentOutput, (0));
    }
    double rightaxis = driverRight.getRawAxis(1);
    if(Math.abs(rightaxis) > .13){
      frontRightSpeed.set(ControlMode.PercentOutput, (rightaxis));
      backRightSpeed.set(ControlMode.PercentOutput, (rightaxis));
    } else {
      frontRightSpeed.set(ControlMode.PercentOutput, (0));
      backRightSpeed.set(ControlMode.PercentOutput, (0));
    }
    
    // shifter set
    if((driverLeft.getRawButton(1) && toggleLT) || (driverRight.getRawButton(1) && toggleRT)) {
      if(gearShift){
        gearShift = false;
      } else {
        gearShift = true;
      }
      toggleLT = false;
    } if(!(driverLeft.getRawButton(1)) && !(toggleLT)) {
      toggleLT = true;
    } if(!(driverRight.getRawButton(1) && toggleRT)) {
      toggleRT = true;
    }
    
    if(gearShift){
      shifter.set(Value.kForward);
    } else {
      shifter.set(Value.kReverse);
    }
    

  /* test color detector input
    Color detectedColor = m_colorSensor.getColor();
    if(xbox.getAButton()){
      if(!(detectedColor.red > 0.47)){
        colorMotor.set(ControlMode.PercentOutput, .1);
      }
    }
  */

    // test belt
    if(xbox.getBumper(Hand.kLeft)){
      beltSpeed = -1; // intake
    }
    if(xbox.getBumper(Hand.kRight)){
      beltSpeed = 1; // reverse
    }
    if(xbox.getBButton()){
      beltMotor.set(ControlMode.PercentOutput, beltSpeed);
    } else {
      beltMotor.set(ControlMode.PercentOutput, 0);
    }

    // color sensor import from field.
    if(fieldColor.charAt(0) == 'B'){
      color = 1;
    } if(fieldColor.charAt(0) == 'Y'){
      color = 2;
    } if(fieldColor.charAt(0) == 'B'){
      color = 3;
    } if(fieldColor.charAt(0) == 'B'){
      color = 4;
    }

    /*
    Color sensor just sets itself to be 2 colors behind the actual.
    Driver must set up the sensor to be on the midpoint color.
    */

    // color sensor manual
    if(driverRight.getRawButton(6)){
      color = 1; // red
    } if(driverRight.getRawButton(7)){
      color = 2; // green
    } if(driverRight.getRawButton(10)){
      color = 3; // blue
    } if(driverRight.getRawButton(11)){
      color = 4; // yellow
    }

    // color sensor pressing
    if(xbox.getXButton() && toggleX) {
      spin(color);
      toggleX = false;
    } if(!(xbox.getXButton() && !(toggleX))){
      toggleX = true;
    }

    // limelight sensing
    if(xbox.getYButton() && toggleY) {
      limeRun();
      toggleY = false;
    } if(!(xbox.getYButton() && !(toggleY))){
      toggleY = true;
    }

  }
  
  private void spin(int ColorSense) {
    boolean colorLoop = true;
    int colorcount = 0;
    boolean dupecheck = true;
    if(ColorSense == 1){
      while(colorLoop){
        Color detectedColor = m_colorSensor.getColor();
        colorMotor.set(ControlMode.PercentOutput, .1);
        if(detectedColor.red > 0.47 && dupecheck) {
          colorcount = colorcount + 1;
          System.out.println(colorcount);
          dupecheck = false;
        }
        if(detectedColor.red <= 0.47) {
          dupecheck = true;
        }
        if(colorcount >= 8) {
          colorLoop = false;
        }
      }
    } else if(ColorSense == 2){
      while(colorLoop){
        Color detectedColor = m_colorSensor.getColor();
        colorMotor.set(ControlMode.PercentOutput, .1);
        if(detectedColor.green > 0.47 && dupecheck) {
          colorcount = colorcount + 1;
          System.out.println(colorcount);
          dupecheck = false;
        }
        if(detectedColor.green <= 0.47) {
          dupecheck = true;
        }
        if(colorcount >= 8) {
          colorLoop = false;
        }
      }
    } else if(ColorSense == 3){
      while(colorLoop){
        Color detectedColor = m_colorSensor.getColor();
        colorMotor.set(ControlMode.PercentOutput, .1);
        if(detectedColor.blue > 0.47 && dupecheck) {
          colorcount = colorcount + 1;
          System.out.println(colorcount);
          dupecheck = false;
        }
        if(detectedColor.blue <= 0.47) {
          dupecheck = true;
        }
        if(colorcount >= 8) {
          colorLoop = false;
        }
      }
    } else if(ColorSense == 4){
      while(colorLoop){
        Color detectedColor = m_colorSensor.getColor();
        colorMotor.set(ControlMode.PercentOutput, .1);
        if(detectedColor.red > 0.27 && detectedColor.green > 0.27 && dupecheck) {
          colorcount = colorcount + 1;
          System.out.println(colorcount);
          dupecheck = false;
        }
        if(detectedColor.red <= 0.47 && detectedColor.green <= 0.27) {
          dupecheck = true;
        }
        if(colorcount >= 8) {
          colorLoop = false;
        }
      }
    } 
  }

  // limelight scan class
  public void limeRun(){
    // init limelight values
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    // variables
    double xVal = tx.getDouble(0.0);
    double area = ta.getDouble(0.0);
    boolean runCheck = true;
    boolean fire = false;
    boolean areaCheck = false;

    // checking to make sure the target exists
    if(area > 0){
      areaCheck = true;
    }
    // spin loop
    if(areaCheck){
      while(runCheck){
        xVal = tx.getDouble(0.0);
        if(xVal > 0.05 && xVal <= 0.2){
          spinMotor.set(ControlMode.PercentOutput, .1);
        } else if(xVal > 0.2){
          spinMotor.set(ControlMode.PercentOutput, .4);
        } else if(Math.abs(xVal) < 0.05){
          spinMotor.set(ControlMode.PercentOutput, 0);
          fire = true;
          runCheck = false;
        } else if(xVal < -0.05 && xVal >= -0.2){
          spinMotor.set(ControlMode.PercentOutput, -.1);
        } else if(xVal < -0.2){
          spinMotor.set(ControlMode.PercentOutput, -.4);
        }
      }
    }
    // final area check
    if(!(area > 0)){
      fire = false;
    }
    // firing
    if(fire){
      // add code to fire turret
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

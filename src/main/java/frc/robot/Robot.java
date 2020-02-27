/*
Controls:
X button - back intake up
B button - back intake down
A button - intake toggle
LT - Limelight
RT - Turret fire
DPad Up = Whinch Up
DPad Down = Whinch Down
Menu button - Color sensor
Xbox bumpers - turret spin
Xbox right stick - intake belt
driver joysticks - drive
driver triggers - gear shift
*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.GenericHID.*;
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
import com.revrobotics.SparkMax;

import java.lang.reflect.InvocationTargetException;

// Talon SRX and FX imports 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

// sparkmax imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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


  // defining controllers
  private XboxController xbox;
  private Joystick driverLeft;
  private Joystick driverRight;

  // defining tank drive
  private TalonFX frontLeftSpeed;
  private TalonFX frontRightSpeed;
  private TalonFX backLeftSpeed;
  private TalonFX backRightSpeed;
  private boolean toggleLT;
  private boolean toggleRT;

  // air pressure
  private boolean gearShift;
  private boolean intakeShift;
  private boolean intakeCheck;
  private DoubleSolenoid shifter;
  private DoubleSolenoid intakeButton;
  private DoubleSolenoid intakeButton2;
  private Compressor compressor;
  private AnalogInput pressureSensor;

  // defining color sensor
  private VictorSPX colorMotor;

  // intake
  private TalonSRX beltMotor;
  private VictorSPX intakeMotor;
  private SpeedController beltTopMotor;
  private SpeedController beltSecondaryMotor;
  private TalonFX winch;
  private boolean toggleB;

  // turret spinner
  private TalonSRX spinMotor;
  // private double spinValue;
  // private CANCoder spinEncoder;

  // turret fire
  private TalonFX turretFire;

  // defining Limelight Variables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  private boolean toggleStart;
  private boolean limeToggle;

  // defining color sensor variables
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private int color = 0;
  private boolean toggleMenu;
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
    colorMotor = new VictorSPX(5);

    // I beat my kids with a
    beltMotor = new TalonSRX(6);
    beltTopMotor = new CANSparkMax(11, MotorType.kBrushless);
    beltSecondaryMotor = new CANSparkMax(12, MotorType.kBrushless);
    winch = new TalonFX(8);
    intakeMotor = new VictorSPX(9);

    // turret spinner
    spinMotor = new TalonSRX(7);
    // spinEncoder = new CANCoder(7);
    // spinValue = 0;

    //turret fire
    turretFire = new TalonFX(15);

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
    intakeShift = true;
    shifter = new DoubleSolenoid(2, 3);
    intakeButton = new DoubleSolenoid(4, 5);
    intakeButton2 = new DoubleSolenoid(6, 7);
    pressureSensor = new AnalogInput(0);
    shifter.set(Value.kReverse);
    intakeButton.set(Value.kForward);
    intakeButton2.set(Value.kForward);
    intakeCheck = true;
    compressor = new Compressor(13);
    compressor.start();

    // turret RPM
    // turretFire.configForwardSoftLimitThreshold(6000, 0);
    // turretFire.configReverseSoftLimitThreshold(-6000, 0);
    turretFire.configForwardSoftLimitEnable(false, 0);
    turretFire.configReverseSoftLimitEnable(false, 0);

    // toggle declare
    toggleMenu = true;
    toggleStart = true;
    toggleB = true;
    toggleLT = true;
    toggleRT = true;
    limeToggle = true;

    // action recorder setup
    /*actions = new ActionRecorder().setMethod(this, "robotOperation", DriverInput.class).setUpButton(xbox, 1)
    .setDownButton(xbox, 2).setRecordButton(xbox, 3);*/
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

    // pushing limelight vars
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    // defining color vars
    Color detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);

    // encoder class
    // spinValue = spinEncoder.getPosition();
    // SmartDashboard.putNumber("Spin Encoder", spinValue);
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
    boolean detectState = true;
    while(detectState){
      if(ta.getDouble(0.0) > 0) {
        detectState = false;
        frontLeftSpeed.set(ControlMode.PercentOutput, 0);
        backLeftSpeed.set(ControlMode.PercentOutput, 0);
        frontRightSpeed.set(ControlMode.PercentOutput, 0);
        backRightSpeed.set(ControlMode.PercentOutput, 0);
      } else {
        frontLeftSpeed.set(ControlMode.PercentOutput, -1);
        backLeftSpeed.set(ControlMode.PercentOutput, -1);
        frontRightSpeed.set(ControlMode.PercentOutput, 1);
        backRightSpeed.set(ControlMode.PercentOutput, 1);
      }
    }
    limeRun();
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

  // }

  // public void robotOperation(DriverInput input){
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
      toggleRT = false;
    } if(!(driverLeft.getRawButton(1)) && !(toggleLT)) {
      toggleLT = true;
    } if(!(driverRight.getRawButton(1)) && !(toggleRT)) {
      toggleRT = true;
    }
  
    if(gearShift){
      shifter.set(Value.kReverse);
    } else {
      shifter.set(Value.kForward);
    }

    // turret spin
    if(xbox.getBumper(Hand.kRight)){
      spinMotor.set(ControlMode.PercentOutput, -1);
    } else if(xbox.getBumper(Hand.kLeft)){
      spinMotor.set(ControlMode.PercentOutput, 1);
    } else {
      spinMotor.set(ControlMode.PercentOutput, 0);
    }

    // belt + intake
    if(xbox.getY(Hand.kRight) > 0 && Math.abs(xbox.getY(Hand.kRight)) > 0.2){
      beltMotor.set(ControlMode.PercentOutput, -1); // in
      intakeMotor.set(ControlMode.PercentOutput, 1);
      beltTopMotor.set(.5);
    } else if(xbox.getY(Hand.kRight) < 0 && Math.abs(xbox.getY(Hand.kRight)) > 0.2){
      beltMotor.set(ControlMode.PercentOutput, 1); // out
      intakeMotor.set(ControlMode.PercentOutput, -1);
      beltTopMotor.set(-.5);
    } else {
      beltMotor.set(ControlMode.PercentOutput, 0);
      intakeMotor.set(ControlMode.PercentOutput, 0);
      beltTopMotor.set(0);
    }

    // // color sensor import from field.
    // if(fieldColor.charAt(0) == 'B'){
    //   color = 1;
    // } if(fieldColor.charAt(0) == 'Y'){
    //   color = 2;
    // } if(fieldColor.charAt(0) == 'R'){
    //   color = 3;
    // } if(fieldColor.charAt(0) == 'G'){
    //   color = 4;
    // }

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
    if(xbox.getRawButton(7) && toggleMenu) {
      spin(color);
      toggleMenu = false;
    } if(!(xbox.getRawButton(7) && !(toggleMenu))){
      toggleMenu = true;
    }

    // limelight?
    if(xbox.getTriggerAxis(Hand.kLeft) > 0.1){
      turretFire.set(ControlMode.PercentOutput, 1);
      limeToggle = false; // limelight is disabled, remove when it isn't
      if(limeToggle) {
        double xVal = tx.getDouble(0.0);
        if(xVal > 0.05 && xVal <= 0.2){
          spinMotor.set(ControlMode.PercentOutput, .1);
        } else if(xVal > 0.2){
          spinMotor.set(ControlMode.PercentOutput, .4);
        } else if(Math.abs(xVal) < 0.05){
          spinMotor.set(ControlMode.PercentOutput, 0);
        } else if(xVal < -0.05 && xVal >= -0.2){
          spinMotor.set(ControlMode.PercentOutput, -.1);
        } else if(xVal < -0.2){
          spinMotor.set(ControlMode.PercentOutput, -.4);
        }
      }
    } else {
      turretFire.set(ControlMode.PercentOutput, 0);
    }

    // intake shifters
    if(xbox.getAButton() && intakeShift) {
      intakeShift = false;
      if(intakeCheck){
        intakeButton.set(Value.kReverse);
        intakeButton2.set(Value.kReverse);
        intakeCheck = false;
      } else {
        intakeButton.set(Value.kForward);
        intakeButton2.set(Value.kForward);
        intakeCheck = true;
      }  
    } if(!(xbox.getAButton() && !(intakeShift))){
      intakeShift = true;
    }

    // winch updown
    if(xbox.getXButton()) {
      winch.set(ControlMode.PercentOutput, (-1));
    } else if(xbox.getBButton()) {
      winch.set(ControlMode.PercentOutput, (1));
    } else {
      winch.set(ControlMode.PercentOutput, (0));
    }

    // intake back
    if(xbox.getTriggerAxis(Hand.kRight) > 0.2){
      beltSecondaryMotor.set(1);
    } else {
      beltSecondaryMotor.set(0);
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
          runCheck = false;
        } else if(xVal < -0.05 && xVal >= -0.2){
          spinMotor.set(ControlMode.PercentOutput, -.1);
        } else if(xVal < -0.2){
          spinMotor.set(ControlMode.PercentOutput, -.4);
        }
      }
    }

/* // I have no clue what any of this is so I'm commenting it out. - Nokes
    // whinch + dpad set
    int dpadA = xbox.getPOV();
    int dpadI = 0;
    if (dpadA == 0)   { dpadI = 1; }
    if (dpadA == 90)  { dpadI = 2; }
    if (dpadA == 180) { dpadI = 3; }
    if (dpadA == 270) { dpadI = 4; }
    if (dpadI == 1) { whinchMotor.set(ControlMode.PercentOutput, 0.5); }
    // if (dpadI == 2) {}
    if (dpadI == 3) { whinchMotor.set(ControlMode.PercentOutput, -0.5); }
    // if (dpadI == 4) { fire = true; } <-- Will be fixed later to be used as an override if Limelight Fails - Dan
    */
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  System.out.println("My first code!");
  }

}

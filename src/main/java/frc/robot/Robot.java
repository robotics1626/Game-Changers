package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// Below: Limelight Imports
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
// Color sensor imports
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;

import java.util.Map;

// Talon SRX and FX imports 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

// sparkmax imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private int intakeCheck;
  private DoubleSolenoid shifter;
  private DoubleSolenoid winchPistons;
  private DoubleSolenoid intakeButton;
  private DoubleSolenoid intakeButton2;
  private Compressor compressor;
  private AnalogInput pressureSensor;

  // defining color sensor
  private VictorSPX colorMotor;

  // intake
  private TalonSRX beltMotor;
  private VictorSPX intakeMotor;
  private TalonSRX beltTopMotor;
  private SpeedController beltSecondaryMotor;
  private TalonFX winch;

  // turret spinner
  private CANSparkMax spinMotor;
  private CANEncoder spinEncoder;
  private double spinPos;

  // turret fire
  private NetworkTableEntry rpmSet;
  private TalonFX turretFire;
  private double rpmFinal;

  // defining Limelight Variables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ledEntry = table.getEntry("ledMode");
  private double x = tx.getDouble(0.0);
  private double y = ty.getDouble(0.0);
  private double area = ta.getDouble(0.0);
  private boolean toggleStart;
  private boolean limeToggle;
  private boolean fireCheck;
  private int limeInt;

  // defining color sensor variables
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private int color = 0;
  private boolean toggleMenu;
  private String fieldColor = DriverStation.getInstance().getGameSpecificMessage();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    rpmSet = Shuffleboard.getTab("SmartDashboard").add("Initial", 4500)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 3500, "max", 5500, "block increment", 100)).getEntry();

    // drive train assignments
    frontLeftSpeed = new TalonFX(1);
    frontRightSpeed = new TalonFX(2);
    backLeftSpeed = new TalonFX(3);
    backRightSpeed = new TalonFX(4);

    // color sensor
    colorMotor = new VictorSPX(5);

    // I beat my kids with a
    beltMotor = new TalonSRX(6);
    beltTopMotor = new TalonSRX(11);
    beltSecondaryMotor = new CANSparkMax(12, MotorType.kBrushless);
    winch = new TalonFX(8);
    intakeMotor = new VictorSPX(9);

    // turret spinner
    spinMotor = new CANSparkMax(7, MotorType.kBrushless);
    spinEncoder = new CANEncoder(spinMotor);
    spinPos = 0;

    // turret fire
    turretFire = new TalonFX(15);
    rpmFinal = 4500;

    // controller and joystick init
    driverLeft = new Joystick(0);
    driverRight = new Joystick(1);
    xbox = new XboxController(2);

    // current peaks
    beltMotor.configPeakCurrentLimit(40, 6);
    beltMotor.configContinuousCurrentLimit(40, 6);
    beltMotor.enableCurrentLimit(true);

    // input declare
    DriverInput.nameInput("LDrive");
    DriverInput.nameInput("RDrive");
    DriverInput.nameInput("AButton");

    // pressure declares
    gearShift = true;
    intakeShift = true;
    shifter = new DoubleSolenoid(13, 6, 7);
    intakeButton = new DoubleSolenoid(13, 4, 5);
    intakeButton2 = new DoubleSolenoid(13, 2, 3);
    winchPistons = new DoubleSolenoid(13, 0, 1);
    pressureSensor = new AnalogInput(0);
    shifter.set(Value.kForward);
    winchPistons.set(Value.kForward);
    intakeButton.set(Value.kForward);
    intakeButton2.set(Value.kForward);
    intakeCheck = 1;
    compressor = new Compressor(13);
    compressor.setClosedLoopControl(true);
    compressor.start();

    // turret RPM
    turretFire.configFactoryDefault();
    
    /* Config neutral deadband to be the smallest possible */
    turretFire.configNeutralDeadband(0.001);

    /* Config sensor used for Primary PID [Velocity] */
    turretFire.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    /* Config the peak and nominal outputs */
    turretFire.configNominalOutputForward(0, Constants.kTimeoutMs);
    turretFire.configNominalOutputReverse(0, Constants.kTimeoutMs);
    turretFire.configPeakOutputForward(1, Constants.kTimeoutMs);
    turretFire.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Config the Velocity closed loop gains in slot0 */
    turretFire.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
    turretFire.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
    turretFire.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
    turretFire.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);

    // toggle declare
    toggleMenu = true;
    toggleStart = true;
    toggleLT = true;
    toggleRT = true;
    limeToggle = true;
    limeInt = 1;

    ledEntry.setNumber(1);
  }

  @Override
  public void robotPeriodic() {
    // pressure check
    double pressure = (250 * (pressureSensor.getVoltage() / 5.0)) - 13;
    SmartDashboard.putString("Pressure Sensor", String.format("%.0f", pressure));

    // turret check
    spinPos = spinEncoder.getPosition();
    SmartDashboard.putNumber("Turret Raw Position", spinPos);
    if (spinPos > 320) {
      SmartDashboard.putString("Turret Spinner Position", "Right Bound");
    } else if (spinPos < 25) {
      SmartDashboard.putString("Turret Spinner Position", "Left Bound");
    } else {
      SmartDashboard.putString("Turret Spinner Position", "Middle");
    }
    SmartDashboard.putNumber("Turret RPM", turretFire.getSelectedSensorVelocity() / 3.41);

    // limelight variable check
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double localTx = limelightTable.getEntry("tx").getDouble(0);

    // pushing limelight vars
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Local Tx", localTx);
    if (limeToggle) {
      SmartDashboard.putString("Limelight Status", "Enabled");
    } else {
      SmartDashboard.putString("Limelight Status", "Disabled");
    }

    // turret RPM output
    SmartDashboard.putNumber("RPM Input", (rpmSet.getDouble(0.0)));
    if(rpmSet.getName() != null){
      rpmFinal = (rpmSet.getDouble(0.0)) * 3.41;
    }

    // defining color vars
    Color detectedColor = m_colorSensor.getColor();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    if(detectedColor.green > 0.47){
      SmartDashboard.putString("Color", "Green");
    } else if(detectedColor.red > 0.27 && detectedColor.green > 0.27) {
      SmartDashboard.putString("Color", "Yellow");
    } else if(detectedColor.red > 0.47){
      SmartDashboard.putString("Color", "Red");
    } else if(detectedColor.blue > 0.47){
      SmartDashboard.putString("Color", "Blue");
    } else {
      SmartDashboard.putString("Color", "N/A");
    }
  }

  @Override
  public void disabledInit() {
    turretFire.set(ControlMode.PercentOutput, 0);
    ledEntry.setNumber(1);
  }

  @Override
  public void autonomousInit() {
    fireCheck = false;
    m_autoSelected = m_chooser.getSelected();
    intakeButton.set(Value.kReverse);
    intakeButton2.set(Value.kReverse);
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
      if(spinPos < 250){
        ledEntry.setNumber(1);
        spinMotor.set(.5);
      } else {
        ledEntry.setNumber(3);
        if(area > 0){
          if(x > 0.2 && x <= 0.4){
            if(spinPos < 332.3752) spinMotor.set(-.05);
          } else if(x > 0.4){
            if(spinPos < 332.3752) spinMotor.set(-.25);
          } else if(Math.abs(x) < 0.2){
            spinMotor.set(0);
            fireCheck = true;
          } else if(x < -0.2 && x >= -0.5){
            if(spinPos > 13.4284) spinMotor.set(.05);                 
          } else if(x < -0.4){
            if(spinPos > 13.4284) spinMotor.set(.25);
          }
        }
      }

      rpmFinal = 4500 * 3.41;
      turretFire.set(TalonFXControlMode.Velocity, rpmFinal);

      if((turretFire.getSelectedSensorVelocity() - 341) < rpmFinal && (turretFire.getSelectedSensorVelocity() + 341) > rpmFinal && fireCheck){
        beltSecondaryMotor.set(1);
        beltMotor.set(ControlMode.PercentOutput, 1);
        beltTopMotor.set(ControlMode.PercentOutput, .5);
      } else {
        beltSecondaryMotor.set(0);
        beltMotor.set(ControlMode.PercentOutput, 0);
        beltTopMotor.set(ControlMode.PercentOutput, 0);
      }
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
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
      shifter.set(Value.kForward);
    } else {
      shifter.set(Value.kReverse);
    }

    // turret spin
    if(xbox.getBumper(Hand.kRight)){
      if(spinPos > 13.4284){
        spinMotor.set(-.25);
      }
    } else if(xbox.getBumper(Hand.kLeft)){
      if(spinPos < 348.3752){
        spinMotor.set(.25);
      }
    } else {
      spinMotor.set(0);
    }

    // belt + intake
    if(xbox.getY(Hand.kRight) > 0 && Math.abs(xbox.getY(Hand.kRight)) > 0.2){
      beltMotor.set(ControlMode.PercentOutput, -1); // in 
      beltSecondaryMotor.set(-.4);
    } else if(xbox.getY(Hand.kRight) < 0 && Math.abs(xbox.getY(Hand.kRight)) > 0.2){
      beltMotor.set(ControlMode.PercentOutput, 1); // out      
      beltSecondaryMotor.set(.4);
    } else {
      beltMotor.set(ControlMode.PercentOutput, 0);
    }

    // intake motor on top of the arm thingy
    if(driverLeft.getRawButton(2)){
      intakeMotor.set(ControlMode.PercentOutput, -.8);
      beltTopMotor.set(ControlMode.PercentOutput, .8);
    } else if(driverRight.getRawButton(2)){
      intakeMotor.set(ControlMode.PercentOutput, .8);
      beltTopMotor.set(ControlMode.PercentOutput, -.8);
    } else {
      intakeMotor.set(ControlMode.PercentOutput, 0);
      beltTopMotor.set(ControlMode.PercentOutput, 0);
    }

     // color sensor import from field.
    if(fieldColor.length() > 0){
      if(fieldColor.charAt(0) == 'B'){
         color = 1;
      } if(fieldColor.charAt(0) == 'Y'){
         color = 2;
      } if(fieldColor.charAt(0) == 'R'){
         color = 3;
      } if(fieldColor.charAt(0) == 'G'){
         color = 4;
      }
    } 

    /*
    Color sensor just sets itself to be 2 colors behind the actual.
    Driver must set up the sensor to be on the midpoint color.
    */

    // color sensor pressing
    if(xbox.getRawButton(6) && toggleMenu) {
      spin(color);
      toggleMenu = false;
    } if(!(xbox.getRawButton(6) && !(toggleMenu))){
      toggleMenu = true;
    }

    // limelight disable
    if(xbox.getRawButton(7) && toggleStart) {
      limeInt = limeInt * -1;
      toggleStart = false;
    } if(!(xbox.getRawButton(7) && !(toggleStart))){
      toggleStart = true;
    }

    if(limeInt == 1){
      limeToggle = true;
    } else {
      limeToggle = false;
    }

    // limelight // turret fire
    if(xbox.getTriggerAxis(Hand.kLeft) > 0.1){
      turretFire.set(TalonFXControlMode.Velocity, rpmFinal);
      if(limeToggle) {
        ledEntry.setNumber(3);
        if(x > 0.2 && x <= 0.4){
          if(spinPos < 332.3752) spinMotor.set(-.05);
        } else if(x > 0.4){
          if(spinPos < 332.3752) spinMotor.set(-.25);
        } else if(Math.abs(x) < 0.2 && Math.abs(x) > 0.0){
          spinMotor.set(0);
        } else if(x < -0.2 && x >= -0.5){
          if(spinPos > 13.4284) spinMotor.set(.05);                 
        } else if(x < -0.4){
          if(spinPos > 13.4284) spinMotor.set(.25);
        }
      }
    } else {
      ledEntry.setNumber(1);
      turretFire.set(ControlMode.PercentOutput, 0);
    }

    // intake shifters
    if(xbox.getAButton() && intakeShift) {
      intakeCheck = intakeCheck * -1;
      intakeShift = false;
    } if(!(xbox.getAButton() && !(intakeShift))){
      intakeShift = true;
    }

    if(intakeCheck == 1){
      intakeButton.set(Value.kReverse);
      intakeButton2.set(Value.kReverse);
    } else {
      intakeButton.set(Value.kForward);
      intakeButton2.set(Value.kForward);
    }  

    // winch updown
    if(xbox.getXButton()) {
      winch.set(ControlMode.PercentOutput, (-1));
      winchPistons.set(Value.kReverse);
    } else if(xbox.getBButton()) {
      winch.set(ControlMode.PercentOutput, (1));
      winchPistons.set(Value.kReverse);
    } else {
      winch.set(ControlMode.PercentOutput, (0));
      winchPistons.set(Value.kForward);
    }

    // intake back
    if(xbox.getTriggerAxis(Hand.kRight) > 0.2 && (turretFire.getSelectedSensorVelocity() - 682) < rpmFinal && (turretFire.getSelectedSensorVelocity() + 682) > rpmFinal){
      beltSecondaryMotor.set(1);
      beltMotor.set(ControlMode.PercentOutput, 1);
      beltTopMotor.set(ControlMode.PercentOutput, .5);
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

   private DoubleSolenoid.Value solenoidPrev = DoubleSolenoid.Value.kOff;
  @Override
  public void testPeriodic() {
    if (xbox.getAButton()) {
      shifter.set(Value.kForward);
    } else {
      shifter.set(Value.kReverse);
    }
    DoubleSolenoid.Value curr = shifter.get();
    if (curr != solenoidPrev) {
      System.out.println("Shifter is " + shifter.get());
      solenoidPrev = curr;
    }
  }

  @Override
  public void testInit() {
    compressor.start();
    compressor.setClosedLoopControl(true);
  
  }
}

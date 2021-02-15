/* C O N T R O L  L I S T

JOYSTICKS:
Left and Right Sticks: Drive Train
Left and Right Triggers: Gear Shift

XBOX CONTROLLER
Left Trigger: Fire Turret
Right Trigger: Feed Intake into Turret.
Left and Right Bumpers: Rotate Turret
Start Button: Limelight Toggle
A Button: Arm Toggle Up/Down
X and B Buttons: Winch
Right Stick Y-Axis: Intake
*/

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

import java.lang.reflect.InvocationTargetException;
import java.util.Map;

// Talon SRX and FX imports 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  private long secondCheck;

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
  private double spinPos_Starter;

  // turret fire
  private NetworkTableEntry rpmSet;
  private TalonFX turretFire;
  private double rpmFinal;
  private boolean turretFire_exception = true;

  // defining Limelight Variables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ledEntry = table.getEntry("ledMode");

  private double x = tx.getDouble(0.0);
  private double y = ty.getDouble(0.0);
  private double area = ta.getDouble(0.0);
  private boolean limeToggle;
  private boolean fireCheck;
  private boolean limeToggle_Check;

  // echo code declarations
  ActionRecorder actions;

  // defining color sensor variables
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  /*
  private int color = 0;
  private boolean toggleColor;
  private final String fieldColor = DriverStation.getInstance().getGameSpecificMessage();
  */ 

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    rpmSet = Shuffleboard.getTab("SmartDashboard").add("Initial", 4500)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 3500, "max", 6000, "block increment", 100)).getEntry();

    // drive train assignments
    frontLeftSpeed = new TalonFX(1);
    frontRightSpeed = new TalonFX(2);
    backLeftSpeed = new TalonFX(3);
    backRightSpeed = new TalonFX(4);

    frontLeftSpeed.setNeutralMode(NeutralMode.Coast);
    frontRightSpeed.setNeutralMode(NeutralMode.Coast);
    backLeftSpeed.setNeutralMode(NeutralMode.Coast);
    backRightSpeed.setNeutralMode(NeutralMode.Coast);

    // color sensor
    colorMotor = new VictorSPX(5);

    // I beat my kids with a
    beltMotor = new TalonSRX(6);
    beltTopMotor = new TalonSRX(11);
    beltSecondaryMotor = new CANSparkMax(12, MotorType.kBrushless);
    winch = new TalonFX(8);
    winch.setNeutralMode(NeutralMode.Brake);
    intakeMotor = new VictorSPX(9);

    // turret spinner
    spinMotor = new CANSparkMax(7, MotorType.kBrushless);
    spinEncoder = new CANEncoder(spinMotor);
    spinPos_Starter = spinEncoder.getPosition();

    // turret fire
    try {
      turretFire = new TalonFX(15);
    } 
    catch(Exception e) {
      System.out.println("TURRET FIRE MECHANISM DISCONNECTED.");
      turretFire_exception = false;
    }
    rpmFinal = 4500;

    // controller and joystick init
    driverLeft = new Joystick(0);
    driverRight = new Joystick(1);
    xbox = new XboxController(2);

    // current peaks
    beltMotor.configPeakCurrentLimit(40, 6);
    beltMotor.configContinuousCurrentLimit(40, 6);
    beltMotor.enableCurrentLimit(true);

    // input declare for recording
    /*
    DriverInput.nameInput("LDrive");
    DriverInput.nameInput("RDrive");
    DriverInput.nameInput("AButton");
    */

    // pressure declares
    gearShift = true;
    intakeShift = true;
    shifter = new DoubleSolenoid(13, 6, 7);
    intakeButton = new DoubleSolenoid(13, 4, 5);
    intakeButton2 = new DoubleSolenoid(13, 2, 3);
    winchPistons = new DoubleSolenoid(13, 0, 1);
    pressureSensor = new AnalogInput(0);
    shifter.set(Value.kReverse);
    winchPistons.set(Value.kForward);
    intakeButton.set(Value.kForward);
    intakeButton2.set(Value.kForward);
    intakeCheck = 1;
    compressor = new Compressor(13);
    compressor.setClosedLoopControl(true);
    compressor.start();

    // turret RPM
    if(turretFire_exception) { 
      /* if statement exists because of the try-catch.
      Not sure if I need to do this, but better safe than
      sorry, and there's no harm in running a little extra
      if statement. - Nokes
      */
      turretFire.configFactoryDefault();

      /* Config neutral deadband to be the smallest possible */
      turretFire.configNeutralDeadband(0.001);

      /* Config sensor used for Primary PID [Velocity] */
      turretFire.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0); // constants here have been moved from Constants.java

      /* Config the peak and nominal outputs */
      turretFire.configNominalOutputForward(0, 0);
      turretFire.configNominalOutputReverse(0, 0);
      turretFire.configPeakOutputForward(1, 0);
      turretFire.configPeakOutputReverse(-1, 0);

      /* Config the Velocity closed loop gains in slot0 */
      turretFire.config_kF(0, Constants.kGains_Velocit.kF, 0);
      turretFire.config_kP(0, Constants.kGains_Velocit.kP, 0);
      turretFire.config_kI(0, Constants.kGains_Velocit.kI, 0);
      turretFire.config_kD(0, Constants.kGains_Velocit.kD, 0);

      turretFire.configVoltageCompSaturation(12.0);
      turretFire.enableVoltageCompensation(true);
    }

    // toggle declare = true;
    toggleLT = true;
    toggleRT = true;
    limeToggle = false;
    limeToggle_Check = true;

    ledEntry.setNumber(1);

    actions = new ActionRecorder().
				setMethod(this, "robotOperation", frc.robot.DriverInput.class).
				setUpButton(xbox, 1).
				setDownButton(xbox, 2).
				setRecordButton(xbox, 4);
		
		DriverInput.nameInput("Driver-Left");
		DriverInput.nameInput("Driver-Right");
		DriverInput.nameInput("Driver-Left-Trigger");
    DriverInput.nameInput("Driver-Right-Trigger");
    DriverInput.nameInput("Driver-Intake-Arm");
		DriverInput.nameInput("Operator-Left-Stick");
		DriverInput.nameInput("Operator-Left-Bumper");
		DriverInput.nameInput("Operator-Left-Trigger");
		DriverInput.nameInput("Operator-Right-Stick");
		DriverInput.nameInput("Operator-Right-Bumper");
		DriverInput.nameInput("Operator-Right-Trigger");
		DriverInput.nameInput("Operator-X-Button");
		DriverInput.nameInput("Operator-Y-Button");
		DriverInput.nameInput("Operator-A-Button");
		DriverInput.nameInput("Operator-B-Button");
		DriverInput.nameInput("Operator-Start-Button");
    DriverInput.nameInput("Operator-Back-Button");
  }

  @Override
  public void robotPeriodic() {
    // pressure check
    final double pressure = (250 * (pressureSensor.getVoltage() / 5.0)) - 13;
    SmartDashboard.putString("Pressure Sensor", String.format("%.0f", pressure));

    // turret check
    spinPos = spinEncoder.getPosition() - spinPos_Starter;
    SmartDashboard.putNumber("Turret Raw Position", spinPos);
    if (spinPos > 320) {
      SmartDashboard.putString("Turret Spinner Position", "Right Bound");
    } else if (spinPos < 25) {
      SmartDashboard.putString("Turret Spinner Position", "Left Bound");
    } else {
      SmartDashboard.putString("Turret Spinner Position", "Middle");
    }
    if(turretFire_exception) SmartDashboard.putNumber("Turret RPM", turretFire.getSelectedSensorVelocity() / 3.41);

    // limelight variable check
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    // pushing limelight vars
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
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
    final Color detectedColor = m_colorSensor.getColor();
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
    if(turretFire_exception) turretFire.set(ControlMode.PercentOutput, 0);
    frontLeftSpeed.set(ControlMode.PercentOutput, 0);
    backLeftSpeed.set(ControlMode.PercentOutput, 0);
    frontRightSpeed.set(ControlMode.PercentOutput, 0);
    backRightSpeed.set(ControlMode.PercentOutput, 0);
    winch.set(ControlMode.PercentOutput, 0);
    beltMotor.set(ControlMode.PercentOutput, 0);
    beltTopMotor.set(ControlMode.PercentOutput, 0);
    colorMotor.set(ControlMode.PercentOutput, 0);
    intakeMotor.set(ControlMode.PercentOutput, 0);
    spinMotor.set(0);
    beltSecondaryMotor.set(0);
    ledEntry.setNumber(1);
    limeToggle=true;
    actions.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    actions.disabledPeriodic();
    ledEntry.setNumber(1);
  }

  @Override
  public void autonomousInit() {
    fireCheck = false;
    m_autoSelected = m_chooser.getSelected();
    intakeButton.set(Value.kForward);
    intakeButton2.set(Value.kForward);
    secondCheck = System.currentTimeMillis();
    actions.autonomousInit("");
  }

  @Override
  public void autonomousPeriodic() {
    /*
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      if(System.currentTimeMillis() - 1000 >= secondCheck){
        if(spinPos < 175){
          ledEntry.setNumber(1);
          spinMotor.set(.5);
        } else {
          ledEntry.setNumber(3);
          if(area > 0){
            if(x > 1 && x <= 3){
              if(spinPos < 332.3752) spinMotor.set(-.05);
            } else if(x > 3){
              if(spinPos < 332.3752) spinMotor.set(-.25);
            } else if(Math.abs(x) < 1 && Math.abs(x) > 0.0){
              spinMotor.set(0);
              fireCheck = true;
            } else if(x < -1 && x >= 3){
              if(spinPos > 0) spinMotor.set(.05);                 
            } else if(x < -3){
              if(spinPos > 0) spinMotor.set(.25);
            }
          }
        }
      }

      rpmFinal = 4500 * 3.41;
      if(turretFire_exception) turretFire.set(TalonFXControlMode.Velocity, rpmFinal);

      if(fireCheck){
        beltSecondaryMotor.set(1);
        beltMotor.set(ControlMode.PercentOutput, 1);
        beltTopMotor.set(ControlMode.PercentOutput, .5);
      } else {
        beltSecondaryMotor.set(0);
        beltMotor.set(ControlMode.PercentOutput, 0);
        beltTopMotor.set(ControlMode.PercentOutput, 0);
      }

      if(System.currentTimeMillis() - 8000 < secondCheck && System.currentTimeMillis() - 11000 > secondCheck){
        frontLeftSpeed.set(ControlMode.PercentOutput, .25);
        backLeftSpeed.set(ControlMode.PercentOutput, .25);
        frontRightSpeed.set(ControlMode.PercentOutput, -.25);
        backRightSpeed.set(ControlMode.PercentOutput, -.25);
      }
      break;
    }
    */
    try{
			if (actions != null){
				actions.longPlayback(this, -1);
			}else{
				Timer.delay(0.010);
			}
		}catch (Exception e){
			System.out.println("AP: " + e.toString());
		}
  }

  @Override
	public void teleopInit() {
		DriverInput.setRecordTime();
		actions.teleopInit();
  }
  
  @Override
  public void teleopPeriodic() {

    try {
			actions.input(new DriverInput()
					.withInput("Operator-X-Button",		      xbox.getXButton())
					.withInput("Operator-Y-Button",		      xbox.getYButton())
					.withInput("Operator-A-Button", 	      xbox.getAButton())
					.withInput("Operator-B-Button",		      xbox.getBButton())
					.withInput("Operator-Start-Button",	    xbox.getStartButton())
					.withInput("Operator-Left-Trigger",  		xbox.getTriggerAxis(Hand.kLeft))
          .withInput("Operator-Right-Trigger",		xbox.getTriggerAxis(Hand.kRight))
          .withInput("Operator-Right-Stick",      xbox.getY(Hand.kRight)) // y-axis only
					.withInput("Driver-Left", 			        driverLeft.getRawAxis(1))
					.withInput("Driver-Right", 			        driverRight.getRawAxis(1))
					.withInput("Driver-Left-Trigger", 	    driverLeft.getRawButton(1))
          .withInput("Driver-Right-Trigger", 	    driverRight.getRawButton(1))
          .withInput("Driver-Intake-Motor-Left",  driverLeft.getRawButton(2))
          .withInput("Driver-Intake-Motor-Right", driverRight.getRawButton(2))
					);					
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
		} catch (InvocationTargetException e) {
      e.printStackTrace();
    }
  }
  
  private void spin(final int ColorSense) {
    boolean colorLoop = true;
    int colorcount = 0;
    boolean dupecheck = true;
    if(ColorSense == 1){
      while(colorLoop){
        final Color detectedColor = m_colorSensor.getColor();
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
        final Color detectedColor = m_colorSensor.getColor();
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
        final Color detectedColor = m_colorSensor.getColor();
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
        final Color detectedColor = m_colorSensor.getColor();
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
    if(xbox.getAButton()) {
      shifter.set(Value.kForward);
    } else {
      shifter.set(Value.kReverse);
    }
    final DoubleSolenoid.Value curr = shifter.get();
    if (curr != solenoidPrev) {
      System.out.println("Shifter is " + shifter.get());
      solenoidPrev = curr;
    }

    limeToggle = handleLimeButton(7);

    ledEntry.setNumber(limeToggle? 3 : 1);
  }

  @Override
  public void testInit() {


    compressor.start();
    compressor.setClosedLoopControl(true);
  }
 
  private boolean handleLimeButton(final int button) {
    if(xbox.getRawButtonPressed(button)) {
      limeToggle = limeToggle ? false : true;
      System.out.println("Button Pressed - now LED is " + limeToggle); 
    }
    return limeToggle;
  }

  public void robotOperation(DriverInput input) {

    // drive train
    double leftaxis = input.getAxis("Driver-Left");
    if(Math.abs(leftaxis) > .13){
      frontLeftSpeed.set(ControlMode.PercentOutput, (leftaxis * -1 ));
      backLeftSpeed.set(ControlMode.PercentOutput, (leftaxis * -1));  
    } else {
      frontLeftSpeed.set(ControlMode.PercentOutput, (0));
      backLeftSpeed.set(ControlMode.PercentOutput, (0));
    }
    double rightaxis = input.getAxis("Driver-Right"); // I don't know why, but these were marked as final - Nokes
    if(Math.abs(rightaxis) > .13){
      frontRightSpeed.set(ControlMode.PercentOutput, (rightaxis));
      backRightSpeed.set(ControlMode.PercentOutput, (rightaxis));
    } else {
      frontRightSpeed.set(ControlMode.PercentOutput, (0));
      backRightSpeed.set(ControlMode.PercentOutput, (0));
    }
    
    // shifter set
    if((input.getButton("Driver-Left-Trigger") && toggleLT) || (input.getButton("Driver-Right-Trigger") && toggleRT)) {
      if(gearShift){
        gearShift = false;
      } else {
        gearShift = true;
      }
      toggleLT = false;
      toggleRT = false;
    } if(!(input.getButton("Driver-Left-Trigger")) && !(toggleLT)) {
      toggleLT = true;
    } if(!(input.getButton("Driver-Right-Trigger")) && !(toggleRT)) {
      toggleRT = true;
    }
  
    if(gearShift){
      shifter.set(Value.kReverse);
    } else {
      shifter.set(Value.kForward);
    }

    // turret spin
    if(input.getButton("Operator-Right-Bumper")){
      if(spinPos > 0){
        spinMotor.set(-.25);
      }
    } else if(input.getButton("Operator-Left-Bumper")){
      if(spinPos < 348.3752){
        spinMotor.set(.25);
      }
    } else {
      spinMotor.set(0);
    }

    // belt + intake
    if(input.getAxis("Operator-Right-Stick") > 0.2){
      beltMotor.set(ControlMode.PercentOutput, -1); // in 
      beltSecondaryMotor.set(-.4);
    } else if(input.getAxis("Operator-Right-Stick") < 0.2){
      beltMotor.set(ControlMode.PercentOutput, 1); // out      
      beltSecondaryMotor.set(.4);
    } else {
      beltMotor.set(ControlMode.PercentOutput, 0);
    }

    // intake motor on top of the arm thingy
    if(input.getButton("Driver-Intake-Motor-Left")){
      intakeMotor.set(ControlMode.PercentOutput, -.8);
      beltTopMotor.set(ControlMode.PercentOutput, .8);
    } else if(input.getButton("Driver-Intake-Motor-Right")){
      intakeMotor.set(ControlMode.PercentOutput, .8);
      beltTopMotor.set(ControlMode.PercentOutput, -.8);
    } else {
      intakeMotor.set(ControlMode.PercentOutput, 0);
      beltTopMotor.set(ControlMode.PercentOutput, 0);
    }

    /*
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

    
    Color sensor just sets itself to be 2 colors behind the actual.
    Driver must set up the sensor to be on the midpoint color.
    
    
    
    // color sensor pressing
    if(xbox.getStartButton() && toggleColor) {
      spin(color);
      toggleColor = false;
    } if(!(xbox.getStartButton() && !(toggleColor))){
      toggleColor = true;
    }
    

    // color spinner manual
    if(xbox.getRawButton(6)){
      colorMotor.set(ControlMode.PercentOutput, .1);
    } else {
      colorMotor.set(ControlMode.PercentOutput, 0);
    }
    */

    // turret position reset
    if(xbox.getStartButton()) spinPos_Starter = spinEncoder.getPosition();

    // limelight enable/disable
    if(xbox.getBackButton() && limeToggle_Check) {
      if(limeToggle) limeToggle = false;
      else limeToggle = true;
      limeToggle_Check = false;
    } else if(!xbox.getBackButton() && !limeToggle_Check) limeToggle_Check = true;

    // limelight // turret fire
    if(input.getAxis("Operator-Left-Trigger") > 0.1){
      if(turretFire_exception) turretFire.set(TalonFXControlMode.Velocity, rpmFinal);
      if(limeToggle) {
        ledEntry.setNumber(3);
        if(x > 1 && x <= 3){
          if(spinPos < 332.3752) spinMotor.set(-.05);
        } else if(x > 3){
          if(spinPos < 332.3752) spinMotor.set(-.25);
        } else if(Math.abs(x) < 1 && Math.abs(x) > 0.0){
          spinMotor.set(0);
        } else if(x < -1 && x >= -3){
          if(spinPos > 0) spinMotor.set(.05);                 
        } else if(x < -3){
          if(spinPos > 0) spinMotor.set(.25);
        }
      }
    } else {
      ledEntry.setNumber(1);
      if(turretFire_exception) turretFire.set(ControlMode.PercentOutput, 0);
    }

    // intake shifters
    if(input.getButton("Operator-A-Button") && intakeShift) {
      intakeCheck = intakeCheck * -1;
      intakeShift = false;
    } if(!(input.getButton("Operator-A-Button") && !(intakeShift))){
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
    if(input.getButton("Operator-X-Button")) {
      winch.set(ControlMode.PercentOutput, (-1));
      winchPistons.set(Value.kReverse);
    } else if(input.getButton("Operator-B-Button")) {
      winch.set(ControlMode.PercentOutput, (1));
      winchPistons.set(Value.kReverse);
    } else {
      winch.set(ControlMode.PercentOutput, (0));
      winchPistons.set(Value.kForward);
    }

    // intake back
    if(input.getAxis("Operator-Right-Trigger") > 0.2 && (turretFire.getSelectedSensorVelocity() - 682) < rpmFinal && (turretFire.getSelectedSensorVelocity() + 682) > rpmFinal){
      beltSecondaryMotor.set(1);
      beltMotor.set(ControlMode.PercentOutput, 1);
      beltTopMotor.set(ControlMode.PercentOutput, .5);
    } else {
      beltSecondaryMotor.set(0);
    }
  }
}

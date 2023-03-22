
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;

public class Robot extends TimedRobot {
  private static final double kUnitsPerRevolution = Constants.unitsPerRevolution;
  public Command m_autonomousCommand;
  public RobotContainer m_robotContainer;
  public Timer timer;
  public Timer disabledTimer;
  public static final XboxController _operator = new XboxController(Constants.operatorControllerPort);
  public static final TalonFX elevator_crude = new TalonFX(Constants.elevatorCrudeTalonID);
  public static final TalonFX _gearbox = new TalonFX(Constants.gearboxTalonID);
  public static final TalonFX wrist = new TalonFX(Constants.wristTalonID);
  public static final I2C.Port i2cPort = I2C.Port.kOnboard;
  public static final DigitalInput bottomlimit = new DigitalInput(Constants.bottomLimitSwitchPort);
  public static final TalonFX intake_batman = new TalonFX(Constants.intakeBatmanTalonID);
  public static final TalonFX intake_robin = new TalonFX(Constants.intakeRobinTalonID);
  public static final PneumaticHub phub = new PneumaticHub(Constants.pneumaticHubID);
  public static final Solenoid batsolenoid = new Solenoid(Constants.pneumaticHubID, PneumaticsModuleType.REVPH, Constants.batSolenoidID);
  public static final Solenoid robsolenoid = new Solenoid(Constants.pneumaticHubID, PneumaticsModuleType.REVPH, Constants.robSolenoidID);
  public double _pos = wrist.getSelectedSensorPosition();
  public static final DutyCycleEncoder gearabsolute = new DutyCycleEncoder(Constants.gearAbsoluteEncoderPort);
  public static final DutyCycleEncoder armabsolute = new DutyCycleEncoder(Constants.armAbsoluteEncoderPort);
  public static final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  public static final SN_Blinkin SN_Blinkin = new SN_Blinkin(Constants.blinkinLEDPort); 

  @Override
  public void robotInit() {
    // Initialize motor controllers and sensors
    phub.enableCompressorAnalog(80, 100);
    PathPlannerServer.startServer(5811);
    wrist.clearStickyFaults();
    intake_batman.clearStickyFaults();
    intake_robin.clearStickyFaults();
    _gearbox.clearStickyFaults();
    elevator_crude.clearStickyFaults();

    // Set up camera server
    CameraServer.startAutomaticCapture("driver_camera", 0);

    // Set up encoders
    armabsolute.setDistancePerRotation(360.0);
    gearabsolute.setDistancePerRotation(360.0);

    // Set up robot container and timers
    m_robotContainer = new RobotContainer();
    timer = new Timer();
    disabledTimer = new Timer();

    // Set motor controller neutral modes
    intake_batman.setNeutralMode(NeutralMode.Brake);
    intake_robin.setNeutralMode(NeutralMode.Brake);
    elevator_crude.setNeutralMode(NeutralMode.Brake);
    _gearbox.setNeutralMode(NeutralMode.Brake);
    wrist.configFactoryDefault();
    intake_batman.configFactoryDefault();
    intake_robin.configFactoryDefault();
    elevator_crude.configFactoryDefault();
    _gearbox.configFactoryDefault();
  }

  @Override
  public void robotPeriodic() {
    // Run command scheduler
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    // Set motor brake and start disabled timer
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    // No action needed
  }

  @Override
  public void autonomousInit() {
    // Set wrist neutral mode to coast and start autonomous command
    wrist.setNeutralMode(NeutralMode.Coast);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }
  @Override
  public void autonomousPeriodic() {
    // Move wrist to position if sensor position is at 12000
    if (_pos == 12000) {
      wrist.set(ControlMode.PercentOutput , 0.05);
}
}

@Override
public void teleopInit() {
// Enable compressor and set solenoids to default positions
phub.enableCompressorAnalog(80, 100);
batsolenoid.set(false);
robsolenoid.set(true);

    // Cancel autonomous command if running
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  
// Set wrist neutral mode to brake and reset absolute encoders
wrist.setNeutralMode(NeutralMode.Brake);
armabsolute.reset();
armabsolute.getPositionOffset();
armabsolute.setPositionOffset(0);
gearabsolute.reset();
gearabsolute.getPositionOffset();
gearabsolute.setPositionOffset(0);

}

@Override
public void teleopPeriodic() {
// Move arm to target position based on operator input
double targetPositionDegrees = 0;
if (_operator.getStartButton()) {
    targetPositionDegrees = 0; // set target position to 0 degrees for the towed configuration
} else if (_operator.getBackButton()) {
    targetPositionDegrees = 45; // set target position to 45 degrees for the extended configuration
}
armabsolute.setDistancePerRotation(360.0); // set the distance per rotation to 360 degrees
double targetPositionEncoderUnits = targetPositionDegrees / armabsolute.getDistancePerRotation(); // convert the target position from degrees to encoder units
double kP = 0.1; // proportional gain
double error = targetPositionEncoderUnits - armabsolute.get(); // calculate the error between the current position and the target position
double output = error * kP; // calculate the output using proportional control
wrist.set(ControlMode.PercentOutput, output); // set the wrist motor to the output

// Toggle intake solenoids based on operator input
boolean toggle = false;
if (_operator.getAButton()) {
  if (toggle == false) {
    toggle = true;
  } else {
    toggle = false;
  }
}

if(toggle == true){
  batsolenoid.set(true);
  robsolenoid.set(false);
} else {
  batsolenoid.set(false);
  robsolenoid.set(true);
}

// Control intake motors based on operator input
if (_operator.getRightBumper()) {
  intake_batman.set(TalonFXControlMode.PercentOutput, .2);
  intake_robin.set(TalonFXControlMode.PercentOutput, -.2);
} else if (_operator.getLeftBumper()) {
  intake_batman.set(TalonFXControlMode.PercentOutput, -.5);
  intake_robin.set(TalonFXControlMode.PercentOutput, .5);
} else if (_operator.getStartButton()) {
  intake_batman.set(TalonFXControlMode.PercentOutput, -1);
  intake_robin.set(TalonFXControlMode.PercentOutput, 1);
} else if (_operator.getBackButton()) {
  intake_batman.set(TalonFXControlMode.PercentOutput, .2);
  intake_robin.set(TalonFXControlMode.PercentOutput, -.2);
} else {
  intake_batman.set(TalonFXControlMode.PercentOutput, 0);
  intake_robin.set(TalonFXControlMode.PercentOutput, 0);
}

// Control wrist motor based on operator input
if (_operator.getLeftTriggerAxis() > 0) {
  wrist.set(ControlMode.PercentOutput, _operator.getLeftTriggerAxis() / 4);
} else if (_operator.getRightTriggerAxis() > 0) {
  wrist.set(ControlMode.PercentOutput, -_operator.getRightTriggerAxis() / 4);
} else {
  wrist.set(ControlMode.Position, wrist.getSelectedSensorPosition() + 20);
}

// Set elevator motor based on operator input, with limit switch protection
if (bottomlimit.get()) {
  elevator_crude.set(TalonFXControlMode.PercentOutput, _operator.getRawAxis(5) / 3);
} else {
  elevator_crude.setSelectedSensorPosition(0);
  elevator_crude.set(TalonFXControlMode.PercentOutput,-.1 + (- (_operator.getRawAxis(5) / 3)));
}

// Set LED pattern based on color sensor input
Color detectedColor = m_colorSensor.getColor();
if (detectedColor.blue > .3) {
  SN_Blinkin.setPattern(PatternType.BPMPartyPalette);
} else if (detectedColor.blue > .32 && detectedColor.red <= 0.35) {
  SN_Blinkin.setPattern(PatternType.EndToEndBlendToBlackC2P);
} else {
  SN_Blinkin.setPattern(PatternType.HotPink);
}

// Display sensor values on SmartDashboard
SmartDashboard.putNumber("Red", detectedColor.red);
SmartDashboard.putNumber("Green", detectedColor.green);
SmartDashboard.putNumber("Blue", detectedColor.blue);
SmartDashboard.putNumber("armduty", armabsolute.getDistance());
SmartDashboard.putNumber("gearduty", gearabsolute.getDistance());
SmartDashboard.putNumber("Wrist", wrist.getSelectedSensorPosition());
}

@Override
public void testInit() {
// Cancel all commands when entering test mode
CommandScheduler.getInstance().cancelAll();
}

@Override
public void testPeriodic() {
// No action needed
}

@Override
public void simulationInit() {
// No action needed
}

@Override
public void simulationPeriodic() {
// No action needed
}
}
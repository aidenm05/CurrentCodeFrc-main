
package frc.robot;
// Import necessary libraries and classes
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
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
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.frcteam3255.components.SN_Blinkin;
import com.frcteam3255.components.SN_Blinkin.PatternType;
import com.revrobotics.CANSparkMax;

public class Robot extends TimedRobot {

  // Declare constants and variables
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
  //public static final CANSparkMax second  = new CANSparkMax(Constants.intakeBatmanTalonID, MotorType.kBrushed);

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
    //second.clearStickyFaults();
    wrist.configNominalOutputForward(0, Constants.kTimeoutMs);
    wrist.configNominalOutputReverse(0, Constants.kTimeoutMs);
    wrist.configPeakOutputForward(1, Constants.kTimeoutMs);
    wrist.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    wrist.config_kF(Constants.kSlotIdx0, 0.02, Constants.kTimeoutMs);
    wrist.config_kP(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
    wrist.config_kI(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
    wrist.config_kD(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
    wrist.config_IntegralZone(Constants.kSlotIdx0, 200);
    wrist.configAllowableClosedloopError(Constants.kSlotIdx0, 400);
    // Set up camera server
    CameraServer.startAutomaticCapture("driver_camera", 0);
    wrist.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      0,
      30
    );
    wrist.setInverted(TalonFXInvertType.Clockwise);
    wrist.setNeutralMode(NeutralMode.Brake);
    wrist.configNeutralDeadband(0.001);
    wrist.setStatusFramePeriod(
      StatusFrameEnhanced.Status_13_Base_PIDF0,
      10,
      Constants.kTimeoutMs
    );

    // Set up rev encoders
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

    // Reset wrist absolute encoder and set neutral mode to brake
    wrist.setSelectedSensorPosition(0);
    wrist.setNeutralMode(NeutralMode.Brake);

    // Reset arm and gear absolute encoders
    armabsolute.reset();
    armabsolute.getPositionOffset();
    armabsolute.setPositionOffset(0);
    gearabsolute.reset();
    //gearabsolute.getPositionOffset();
    //gearabsolute.setPositionOffset(0);
}

@Override
public void teleopPeriodic() {
    // Move arm to target position based on operator input
    double targetPositionDegrees = 0;
    // Set distance per rotation to 360 degrees for arm absolute encoder
    armabsolute.setDistancePerRotation(360.0);
    // Convert target position in degrees to encoder units
    double targetPositionEncoderUnits = targetPositionDegrees / armabsolute.getDistancePerRotation();
    // Proportional gain for arm position control
    double kP = 0.1;
    // Calculate error between current position and target position
    double error = targetPositionEncoderUnits - armabsolute.get();
    // Calculate output using proportional control
    double output = error * kP;
    // Set wrist motor to output
    wrist.set(ControlMode.PercentOutput, output);
    double currentArmPos = armabsolute.get();
  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if (_operator.getStartButton()) {
  // Set up timer and start it
  Timer timeoutTimer = new Timer();
  timeoutTimer.reset();
  timeoutTimer.start();

  // Loop until arm absolute encoder reaches 200 or 2 seconds have elapsed
  while (armabsolute.get() < 200 && timeoutTimer.get() < 1.0) {
    wrist.set(ControlMode.PercentOutput, 0.05);
  }

  // Stop the wrist motor after 2 seconds have elapsed
  wrist.set(ControlMode.PercentOutput, 0);
}
  
  if (_operator.getBackButton()) {
  // Set up timer and start it
  Timer timeoutTimer = new Timer();
  timeoutTimer.reset();
  timeoutTimer.start();

  // Loop until gearbox absolute encoder reaches 200 or 2 seconds have elapsed
  while (gearabsolute.getDistance() < 200 && timeoutTimer.get() < 1.0) {
    _gearbox.set(ControlMode.PercentOutput, 0.5);
  }

  // Stop the gearbox motor after 2 seconds have elapsed
  _gearbox.set(ControlMode.PercentOutput, 0);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
//     // Toggle intake solenoids based on operator input
//    boolean toggle = false;
//     if (_operator.getYButton()) {
//         if (toggle == false) {
//             toggle = true;
//         } else if (_operator.getYButtonPressed()) {
//             toggle = false;
//         }
//     }

//     if (toggle == true) {
//         batsolenoid.set(true);
//         robsolenoid.set(false);
//     } else {
//         batsolenoid.set(false);
//         robsolenoid.set(true);
//     }

//     if (_operator.getXButton()) {
//         batsolenoid.set(true);
//         robsolenoid.set(false);
//     } else if (_operator.getAButton()) {
//         batsolenoid.set(false);
//         robsolenoid.set(true);
//     }

    // Toggle intake solenoids based on operator input
    boolean toggle = false;
  
    if (_operator.getRawButtonPressed(XboxController.Button.kY.value)) {
        toggle = !toggle;
    }

    if (toggle == true) {
        batsolenoid.set(true);
        robsolenoid.set(false);
    } else {
        batsolenoid.set(false);
        robsolenoid.set(true);
    }
    // Control intake motors based on operator input
 if (_operator.getRightBumper()) {
        intake_batman.set(TalonFXControlMode.PercentOutput, -1);
        intake_robin.set(TalonFXControlMode.PercentOutput, 1);
        // second.set(-1);
    } else if (_operator.getLeftBumper()) {
        intake_batman.set(TalonFXControlMode.PercentOutput, .2);
        intake_robin.set(TalonFXControlMode.PercentOutput, -.2);
        // second.set(1);
    } else {
        intake_batman.set(TalonFXControlMode.PercentOutput, 0);
        intake_robin.set(TalonFXControlMode.PercentOutput, 0);
        // second.set(0);
    }

    // Control wrist motor based on operator input
   // wrist.set(ControlMode.PercentOutput, _operator.getRawAxis(1) / 3);
  if (Math.abs(_operator.getRawAxis(1)) > 0.1) {
    wrist.set(TalonFXControlMode.PercentOutput, _operator.getRawAxis(1) / 2);
} else {
    wrist.set(TalonFXControlMode.Position, elevator_crude.getSelectedSensorPosition(0));
}
  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//     if (!(_operator.getRawAxis(1) == 0)) {
//         wrist.set(TalonFXControlMode.PercentOutput, _operator.getRawAxis(1) / 2);
//     } else {
//         wrist.set(TalonFXControlMode.Position, elevator_crude.getSelectedSensorPosition(0));
//     }
  
    // Hold wrist position using last arm absolute position after trigger is released
  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//     if (_operator.getRawAxis(1) == 0) {
//         wrist.set(TalonFXControlMode.Position, currentArmPos);
//     }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    // Set elevator motor based on operator input, with limit switch protection
    if (bottomlimit.get()) {
        if (!(_operator.getRawAxis(5) == 0))
            elevator_crude.set(TalonFXControlMode.PercentOutput, _operator.getRawAxis(5) / 3);
        else {
            elevator_crude.set(TalonFXControlMode.PercentOutput, .3);
        }
    } else {
        elevator_crude.setSelectedSensorPosition(0);
        elevator_crude.set(TalonFXControlMode.PercentOutput, -.2);
    }
    
    // Control gearbox motor based on operator input
    if (_operator.getLeftTriggerAxis() > 0.1) {
        _gearbox.set(ControlMode.PercentOutput, _operator.getLeftTriggerAxis());
    } else if (_operator.getRightTriggerAxis() > 0.1) {
        _gearbox.set(ControlMode.PercentOutput, -_operator.getRightTriggerAxis());
    } else {
        _gearbox.set(ControlMode.PercentOutput, 0);
    }

    // Set LED pattern based on color sensor input
    Color detectedColor = m_colorSensor.getColor();
    if (detectedColor.blue > .3) {
        SN_Blinkin.setPattern(PatternType.BPMPartyPalette);
    } else if (((detectedColor.red > 0.30) && (detectedColor.red < 0.40)) || ((detectedColor.blue > 0.30) && (detectedColor.blue < 0.50))) {
        SN_Blinkin.setPattern(PatternType.BPMLavaPalette);
        toggle = true;
    } else {
        SN_Blinkin.setPattern(PatternType.HotPink);
    }

    // Display sensor values on SmartDashboard
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("armduty", armabsolute.getDistance());
    SmartDashboard.putNumber("gearduty", gearabsolute.getDistance());
} 
// if (_operator.getStartButtonPressed() && armabsolute.getDistance() > 10 ){
//   wrist.set(ControlMode.PercentOutput, .05);
// } else if ( armabsolute.getDistance() < 15 ){
//   wrist.set(ControlMode.PercentOutput, -.2);
//}
// else if ( armabsolute.getDistance() > targetvari ){
//   wrist.set(ControlMode.PercentOutput, .05);
// }
//  else if (armabsolute.getDistance() < targetvari ) {
//   wrist.set(ControlMode.PercentOutput, -.05);
// }


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

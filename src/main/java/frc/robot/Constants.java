
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;


public final class Constants {
// Talon IDs
public static final int elevatorCrudeTalonID = 4;
public static final int gearboxTalonID = 18;
public static final int wristTalonID = 19;
public static final int intakeBatmanTalonID = 23;
public static final int intakeRobinTalonID = 20;
// Controller Ports
public static final int operatorControllerPort = 2;
// Sensor Ports
public static final int bottomLimitSwitchPort = 0;
public static final int gearAbsoluteEncoderPort = 1;
public static final int armAbsoluteEncoderPort = 2;
// Pneumatics
public static final int pneumaticHubID = 15;
public static final int batSolenoidID = 7;
public static final int robSolenoidID = 0;
// LED Port
public static final int blinkinLEDPort = 6;
 // Other Constants
public static final double unitsPerRevolution = 360.0;
//need to add shufflebaord constant editing
  public static final double LIMELIGHT_DEADBAND = 0.005;
  public static final double MIN_STEER_K = 0.05;
  // elevator constants
  public static final int elevatorUpperLimit = 130000;
  // this is used to set a threshold where the shoulder position needs to be considered
  public static final int elevatorLowerThreshold = 60000;
  // set height for elevator pos 1
  public static final int elevatorTopCone = 129200;
  // set height for elevator pos 2
  public static final int elevatorMidCone = 79960;
  public static final int elevatorTopCube = 97542;
  public static final int elevatorMidCube = 41381;
  public static final int elevatorStow = 0;
  public static final int elevatorFloor = 21477;
  public static final int elevatorShelf = 91340;
  // shoulder constants
  public static final int shoulderUpperLimit = 633840; // DO NOT TOUCH
  // this is used to set a threshold of where the elevator postion needs to be
  public static final int shoulderLowerThreshold = 800;
  // set height for shoulder pos 1
  public static final int shoulderTopCone = 992;
  // set height for shoulder pos 2
  public static final int shoulderMidCone = 1254;
  public static final int shoulderTopCube = 1013;
  public static final int shoulderMidCube = 1334;
  public static final int shoulderStow = 1675;
  public static final int shoulderFloor = 461;
  public static final int shoulderShelf = 1040;
  // shoulder feed forward
  public static final int horizontalPos = 870;
  public static final int ticksPerDegrees = 4096 / 360;
  public static final double maxFF = .03;
  // shoulder encoder
  public static final double shoulderEncoderOffset = 312.02;
// weird motion magic
  public static final int kSlotIdx0 = 0;
  public static final int kSlotIdx1 = 1;
  public final int test = 0;
  public static final int kPIDLoopIdx = 0;
  public static int kTimeoutMs = 30;
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double CHASSIS_MASS = ROBOT_MASS;
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13;
  // use this to not slam into an april tag. Based on half our length.
  public static final double xOffset = .8;
  public static final boolean OUTPUT_DEBUG_VALUES = true;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double LEFT_X_DEADBAND = 0.00;
    public static final double LEFT_Y_DEADBAND = 0.00;
  }

  public static class shoulderConstants {
  }

  public static final class Auton {

    public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0.0);
    public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0.0);
    public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.0);

    // public static final double MAX_SPEED = 4;
    // public static final double MAX_ACCELERATION = 2;
  }

  public static final class constshoulder { 
  }

  public static final class Drivebase {
    public static final double WHEEL_LOCK_TIME = 9; // seconds
  }
}

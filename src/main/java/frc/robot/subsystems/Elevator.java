package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
/* 
This code is for the Elevator subsystem of a robot. It uses two TalonFX motors, one for the elevator and one for the shoulder. 
The elevator motor is configured to use Motion Magic control mode, which allows for precise control of the motor's position. 
The shoulder motor is also configured to use Motion Magic control mode.
The code sets various motor configuration values, such as the PID gains, motion cruise velocity, and motion acceleration. 
These values are used to tune the motor's behavior and ensure that it moves smoothly and accurately.
The code also sets soft limits for the motors, which prevent them from moving beyond certain positions. 
In the periodic() method, the code updates the SmartDashboard with the current encoder values for both motors and the active trajectory position for the shoulder motor.
To tune the motor configuration values, the team would need to experiment with different values and observe how the motor behaves. 
They can adjust the gains to improve the motor's response and stability, and adjust the motion cruise velocity and acceleration to control how quickly the motor moves to its target position. The team can also adjust the soft limits to ensure that the motors do not move beyond safe positions.
*/
public class Elevator extends SubsystemBase {
  public WPI_TalonFX mainMotor;
  public double calculatedPOutput = 0;
  public double motorPosition;
  public int smoothing = 0;
  int upTargetPos = 10000;
  int downTargetPosition = 100;
  int count = 0;
  public WPI_TalonFX shoulderMotor;

  public Elevator() {
    SupplyCurrentLimitConfiguration elevatorSupplyLimit = new SupplyCurrentLimitConfiguration(
      true,
      25,
      40,
      .1
    );

    mainMotor = new WPI_TalonFX(Constants.elevatorCrudeTalonID); 
    shoulderMotor = new WPI_TalonFX(Constants.gearboxTalonID);

//need to add second motor configured simmlar to main that acts as the shoulder in conjunction with the eleavyion motor

shoulderMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        0,
        30
      );

  
      shoulderMotor.setInverted(TalonFXInvertType.Clockwise);
      shoulderMotor.setNeutralMode(NeutralMode.Brake);
      shoulderMotor.configNeutralDeadband(0.001);

      shoulderMotor.configSupplyCurrentLimit(elevatorSupplyLimit);
      shoulderMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0,
        10,
        Constants.kTimeoutMs
      );

      shoulderMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic,
        10,
        Constants.kTimeoutMs
      );
      shoulderMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
      shoulderMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
      shoulderMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
      shoulderMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
      shoulderMotor.config_kF(Constants.kSlotIdx0, 0.060176, Constants.kTimeoutMs);
      shoulderMotor.config_kP(Constants.kSlotIdx0, 0.2, Constants.kTimeoutMs);
      shoulderMotor.config_kI(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      shoulderMotor.config_kD(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      shoulderMotor.config_IntegralZone(Constants.kSlotIdx0, 200);
      shoulderMotor.configAllowableClosedloopError(Constants.kSlotIdx0, 400);
      /* Set Motion Magic gains in slot1 - see documentation */
      shoulderMotor.selectProfileSlot(Constants.kSlotIdx1, Constants.kPIDLoopIdx);
      shoulderMotor.config_kF(Constants.kSlotIdx1, 0.060176, Constants.kTimeoutMs);
      shoulderMotor.config_kP(Constants.kSlotIdx1, 0.2, Constants.kTimeoutMs);
      shoulderMotor.config_kI(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
      shoulderMotor.config_kD(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
      shoulderMotor.config_IntegralZone(Constants.kSlotIdx1, 200);
      shoulderMotor.configAllowableClosedloopError(Constants.kSlotIdx1, 100);
      /* Set acceleration and vcruise velocity - see documentation */
      shoulderMotor.configMotionCruiseVelocity(17000, Constants.kTimeoutMs);
      shoulderMotor.configMotionAcceleration(17000, Constants.kTimeoutMs);
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
      shoulderMotor.configForwardSoftLimitEnable(true);
      shoulderMotor.configForwardSoftLimitThreshold(Constants.elevatorUpperLimit);
        
  
mainMotor.configSelectedFeedbackSensor(
        TalonFXFeedbackDevice.IntegratedSensor,
        0,
        30
      );


      mainMotor.setInverted(TalonFXInvertType.Clockwise);
      mainMotor.setNeutralMode(NeutralMode.Brake);
      mainMotor.configNeutralDeadband(0.001);

      mainMotor.configSupplyCurrentLimit(elevatorSupplyLimit);
      mainMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_13_Base_PIDF0,
        10,
        Constants.kTimeoutMs
      );

      mainMotor.setStatusFramePeriod(
        StatusFrameEnhanced.Status_10_MotionMagic,
        10,
        Constants.kTimeoutMs
      );
      mainMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
      mainMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
      mainMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
      mainMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);
      mainMotor.config_kF(Constants.kSlotIdx0, 0.060176, Constants.kTimeoutMs);
      mainMotor.config_kP(Constants.kSlotIdx0, 0.2, Constants.kTimeoutMs);
      mainMotor.config_kI(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      mainMotor.config_kD(Constants.kSlotIdx0, 0, Constants.kTimeoutMs);
      mainMotor.config_IntegralZone(Constants.kSlotIdx0, 200);
      mainMotor.configAllowableClosedloopError(Constants.kSlotIdx0, 400);
      /* Set Motion Magic gains in slot1 - see documentation */
      mainMotor.selectProfileSlot(Constants.kSlotIdx1, Constants.kPIDLoopIdx);
      mainMotor.config_kF(Constants.kSlotIdx1, 0.060176, Constants.kTimeoutMs);
      mainMotor.config_kP(Constants.kSlotIdx1, 0.2, Constants.kTimeoutMs);
      mainMotor.config_kI(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
      mainMotor.config_kD(Constants.kSlotIdx1, 0, Constants.kTimeoutMs);
      mainMotor.config_IntegralZone(Constants.kSlotIdx1, 200);
      mainMotor.configAllowableClosedloopError(Constants.kSlotIdx1, 100);
      /* Set acceleration and vcruise velocity - see documentation */
      mainMotor.configMotionCruiseVelocity(17000, Constants.kTimeoutMs);
      mainMotor.configMotionAcceleration(17000, Constants.kTimeoutMs);
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
      mainMotor.configForwardSoftLimitEnable(true);
      mainMotor.configForwardSoftLimitThreshold(Constants.elevatorUpperLimit);
        }
  
  public CommandBase resetElevatorEncoder() {
    return run(() -> mainMotor.setSelectedSensorPosition(0));
  }

  public CommandBase runDown() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, -.2))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.03))
      .withName("runDown");
  }

  public CommandBase runUp() {
    return run(() -> mainMotor.set(TalonFXControlMode.PercentOutput, 0.27))
      .finallyDo(interrupted -> mainMotor.set(ControlMode.PercentOutput, 0.03))
      .withName("runUp");
  }

  public CommandBase shoulderdown() {
    return run(() -> shoulderMotor.set(TalonFXControlMode.PercentOutput, -0.1))
      .finallyDo(interrupted ->
        shoulderMotor.set(ControlMode.PercentOutput, (-0.1))
      )
      .withName("shoulderdown");
  }

  public CommandBase shoulderUp() {
    return run(() -> shoulderMotor.set(TalonFXControlMode.PercentOutput, 0.1))
      .finallyDo(interrupted ->
        shoulderMotor.set(ControlMode.PercentOutput, (-0.1))
      )
      .withName("shoulderUp");
  }

  public void shoulderAndElevatorStopPercentMode() {
    // if (!DriverStation.isAutonomous()) {
    shoulderMotor.set(TalonFXControlMode.PercentOutput, (0.03));
    mainMotor.set(TalonFXControlMode.PercentOutput, (0.03));
    // }
  }

  public CommandBase sequentialSetPositions(
    final int elevatorPosition,
    int shoulderPosition
  ) {
    mainMotor.selectProfileSlot(Constants.kSlotIdx0, Constants.kPIDLoopIdx);
    return runOnce(() ->
        shoulderMotor.set(TalonFXControlMode.MotionMagic, Constants.shoulderUpperLimit)
      )
      .andThen(
        Commands
          .waitUntil(() ->
            shoulderMotor.getActiveTrajectoryPosition() >
            Constants.shoulderUpperLimit -
            100
          )
          .withTimeout(1)
      )
      .andThen(
        runOnce(() ->
          mainMotor.set(
            TalonFXControlMode.MotionMagic,
            elevatorPosition,
            DemandType.ArbitraryFeedForward,
            0.03
          )
        )
      )
      .andThen(
        Commands
          .waitUntil(() ->
            mainMotor.getActiveTrajectoryPosition() < elevatorPosition + 30000 &&
            mainMotor.getActiveTrajectoryPosition() > elevatorPosition - 30000
          )
          .withTimeout(1.5)
      )
      .andThen(
        runOnce(() -> shoulderMotor.set(TalonFXControlMode.MotionMagic, shoulderPosition))
      )
      .andThen(
        Commands
          .waitUntil(() ->
            shoulderMotor.getActiveTrajectoryPosition() < shoulderPosition + 20 &&
            shoulderMotor.getActiveTrajectoryPosition() > shoulderPosition - 20
          )
          .withTimeout(1)
      )
      .andThen(runOnce(() -> this.shoulderAndElevatorStopPercentMode()));
  }

  // Test this
  public CommandBase setStow() {
    mainMotor.selectProfileSlot(Constants.kSlotIdx1, Constants.kPIDLoopIdx);
    return runOnce(() ->
        shoulderMotor.set(TalonFXControlMode.MotionMagic, Constants.shoulderUpperLimit)
      )
      // .andThen(
      //   Commands
      // .waitUntil(() ->
      //   shoulderMotor.getActiveTrajectoryPosition() >
      //   Constants.shoulderUpperLimit -
      //   100 &&
      //   shoulderMotor.getActiveTrajectoryPosition() <
      //   Constants.shoulderUpperLimit +
      //   100
      // )
      // .withTimeout(1)
      //) // set to current upperlimit
      .andThen(
        runOnce(() ->
          shoulderMotor.configForwardSoftLimitThreshold(Constants.shoulderStow)
        )
      ) // set soft limit to be stow position
      .andThen(
        runOnce(() ->
          mainMotor.set(
            TalonFXControlMode.MotionMagic,
            Constants.elevatorStow,
            DemandType.ArbitraryFeedForward,
            0.03
          )
        )
      ) // set elevator to 0
      .andThen(
        Commands
          .waitUntil(() ->
            mainMotor.getActiveTrajectoryPosition() <
            Constants.elevatorStow +
            5000 &&
            mainMotor.getActiveTrajectoryPosition() >
            Constants.elevatorStow -
            5000
          )
          .withTimeout(2.25)
      )
      .andThen(
        runOnce(() ->
          shoulderMotor.set(TalonFXControlMode.MotionMagic, Constants.shoulderStow)
        )
      ) //^wait until finished, set shoulder to stow
      .andThen(
        Commands
          .waitUntil(() ->
            shoulderMotor.getActiveTrajectoryPosition() > Constants.shoulderStow - 20 &&
            shoulderMotor.getActiveTrajectoryPosition() < Constants.shoulderStow + 20
          )
          .withTimeout(3)
      ) //wait until finished
      .andThen(
        runOnce(() ->
          shoulderMotor.configForwardSoftLimitThreshold(Constants.shoulderUpperLimit)
        )
      )
      // ) //set soft limit back to what it was
      .andThen(runOnce(() -> this.shoulderAndElevatorStopPercentMode()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
      "elevatorEncoderVal",
      mainMotor.getSelectedSensorPosition()
    );
    SmartDashboard.putNumber(
      "shoulderEncoderVal",
      shoulderMotor.getSelectedSensorPosition()
    );
    SmartDashboard.putNumber(
      "Active Trajectory Position",
      shoulderMotor.getActiveTrajectoryPosition()
    );
  
// Main Motor FPID tuning

// Main Motor FPID tuning

// Main Motor integral zone and limit tuning
  }
}

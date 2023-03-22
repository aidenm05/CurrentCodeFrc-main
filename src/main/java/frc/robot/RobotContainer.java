// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.drivebase.TeleopDrive;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private static final SendableChooser<String> AutoPath = new SendableChooser<>();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));
  public final Elevator m_Elevator = new Elevator();

  public final GenericHID driverXbox = new GenericHID(0);
  // private final GenericHID conOperator = new GenericHID(2);
  public final GenericHID operator = new GenericHID(1);

  private static AprilTagFieldLayout aprilTagField = null;
  private final JoystickButton yButton1 = new JoystickButton(operator,XboxController.Button.kY.value);
  private final JoystickButton aButton1 = new JoystickButton(operator,XboxController.Button.kA.value);
  private final JoystickButton bButton1 = new JoystickButton(operator,XboxController.Button.kB.value);
  private final JoystickButton xButton1 = new JoystickButton(operator,XboxController.Button.kX.value);
  private final JoystickButton leftStickButton1 = new JoystickButton(operator,XboxController.Button.kLeftStick.value);
  private final JoystickButton rightStickButton1 = new JoystickButton(operator,XboxController.Button.kRightStick.value);
  private final JoystickButton start = new JoystickButton(operator,XboxController.Button.kStart.value);
  private final JoystickButton leftBumper1 = new JoystickButton( operator,XboxController.Button.kLeftBumper.value);
  private final JoystickButton back = new JoystickButton(operator,XboxController.Button.kBack.value);
  private final JoystickButton rightBumper1 = new JoystickButton( operator,XboxController.Button.kRightBumper.value);
  private final POVButton b7 = new POVButton(operator, 0);
  private final POVButton b8 = new POVButton(operator, 90);
  private final POVButton b9 = new POVButton(operator, 180);
  private final POVButton b10 = new POVButton(operator, 270);


  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // poser.addDashboardWidgets(Shuffleboard.getTab("Vision"));

    AutoPath.addOption("Two Piece Blue NB", "Two Piece Blue NB");
    AutoPath.addOption("Two Piece Red NB", "Two Piece Red NB");
    AutoPath.addOption("Two Piece Red Balance", "Two Piece Red Balance");
    AutoPath.addOption("Two Piece Blue Balance", "Two Piece Blue Balance");
    AutoPath.setDefaultOption("Middle Blue", "Middle Blue");
    AutoPath.addOption("Middle Red", "Middle Red");
    AutoPath.addOption("Blue Right", "Blue Right");
    AutoPath.addOption("Red Left Balance", "Red Left Balance");

    SmartDashboard.putData("Auto Pathing", AutoPath);

    TeleopDrive closedFieldRel = new TeleopDrive(
        drivebase,
        () -> (Math.abs(driverXbox.getRawAxis(2)) > OperatorConstants.LEFT_Y_DEADBAND) ? driverXbox.getRawAxis(2) : 0,
        () -> (Math.abs(-driverXbox.getRawAxis(0)) > OperatorConstants.LEFT_X_DEADBAND) ? -driverXbox.getRawAxis(0) : 0,
        () -> -driverXbox.getRawAxis(3), () -> true, false, false);

    drivebase.setDefaultCommand(closedFieldRel);

  }

  public void configureNeutralModes() {
  }

  public void resetToAbsolutePositions() {
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

 //  AButton.whileTrue(new RepeatCommand(new InstantCommand(drivebase::lock, drivebase)));

    aButton1.onTrue(
        m_Elevator.sequentialSetPositions(
            Constants.elevatorFloor,
           Constants.shoulderFloor
          ));

    bButton1.onTrue(m_Elevator.setStow());

    // Elevator shoulder Presets
    xButton1.onTrue(
        m_Elevator.sequentialSetPositions(
            Constants.elevatorTopCone,
            Constants.shoulderTopCone
            ));
    yButton1.onTrue(
        m_Elevator.sequentialSetPositions(
            Constants.elevatorMidCone,
            Constants.shoulderMidCone
            ));
    start.onTrue(
        m_Elevator.sequentialSetPositions(
            Constants.elevatorTopCube,
            Constants.shoulderTopCube
            ));
    back.onTrue(
        m_Elevator.sequentialSetPositions(
            Constants.elevatorMidCube,
            Constants.shoulderMidCube
            ));
    rightStickButton1.onTrue(
        m_Elevator.sequentialSetPositions(
            Constants.elevatorShelf,
            Constants.shoulderShelf
            ));
    //b7.whileTrue(m_Elevator.shoulderUp());
    //b8.whileTrue(m_Elevator.shoulderDown());
    b9.whileTrue(m_Elevator.runUp());
    b10.whileTrue(m_Elevator.runDown());

    SmartDashboard.putData("Drive to 1", driveToAprilTag(drivebase, 1, Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(0), new Translation2d((-Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 2", driveToAprilTag(drivebase, 2, Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(0), new Translation2d((-Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 3", driveToAprilTag(drivebase, 3, Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(0), new Translation2d((-Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 4", driveToAprilTag(drivebase, 4, Rotation2d.fromDegrees(0),
        Rotation2d.fromDegrees(0), new Translation2d(-Constants.xOffset, (Units.inchesToMeters(0)))));

    SmartDashboard.putData("Drive to 5", driveToAprilTag(drivebase, 5, Rotation2d.fromDegrees(180),
        Rotation2d.fromDegrees(180), new Translation2d((Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 6", driveToAprilTag(drivebase, 6, Rotation2d.fromDegrees(180),
        Rotation2d.fromDegrees(180), new Translation2d((Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 7", driveToAprilTag(drivebase, 7, Rotation2d.fromDegrees(180),
        Rotation2d.fromDegrees(180), new Translation2d((Constants.xOffset), 0)));

    SmartDashboard.putData("Drive to 8", driveToAprilTag(drivebase, 8, Rotation2d.fromDegrees(180),
        Rotation2d.fromDegrees(180), new Translation2d((Constants.xOffset), 0)));

    // auto.onTrue(Commands.sequence(autoToConePickup(drivebase, 4, new
    // Translation2d(Units.inchesToMeters(-36), 0)),
    // IntakeOutwards,collectgamepiececommand));

    // new JoystickButton(driverXbox, 3).onTrue(new
    // InstantCommand(drivebase::addFakeVisionReading));

    new JoystickButton(driverXbox, 4).onTrue(driveToAprilTag(drivebase, 4,
    Rotation2d.fromDegrees(0),
    Rotation2d.fromDegrees(0), new Translation2d(Units.inchesToMeters(-36), 0)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    Command reachout = new RunCommand(() -> Robot.wrist.set(ControlMode.PercentOutput, -.3));
    Command zero = new RunCommand(() -> Robot.wrist.setSelectedSensorPosition(0.0));
    Command stop = new RunCommand(() -> Robot.wrist.set(ControlMode.PercentOutput, .09));
    Command fart = new RunCommand(() -> Robot.intake_robin.set(ControlMode.PercentOutput, 0.5));
    Command poop = new RunCommand(() -> Robot.intake_batman.set(ControlMode.PercentOutput, -.5));

    return Commands.sequence(
        zero.withTimeout(1.2),
        reachout.withTimeout(1.40),
        stop.withTimeout(.25),
        fart.withTimeout(.05),
        poop.withTimeout(.05));

    // Autos.exampleAuto(drivebase, AutoPath.getSelected()));
  }

  public void setDriveMode() {
    // drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  /**
   * Create a {@link FollowTrajectory} command to go to the April Tag from the
   * current position.
   *
   * @param swerve            Swerve drive subsystem.
   * @param id                April Tag ID to go to.
   * @param rotation          Rotation to go to.
   * @param holonomicRotation Holonomic rotation to be at.
   * @param offset            Offset from the April Tag.
   * @return {@link FollowTrajectory} command. May return null if cannot load
   *         field.
   */
  public CommandBase driveToAprilTag(SwerveSubsystem swerve, int id, Rotation2d rotation,
      Rotation2d holonomicRotation, Translation2d offset) {

    if (aprilTagField == null) {
      try {
        aprilTagField = new AprilTagFieldLayout(
            Filesystem.getDeployDirectory() + "/apriltags/2023-chargedup.json");
      } catch (Exception ignored) {
        return null;
      }
    }
    PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(4, 3), false,
        PathPoint.fromCurrentHolonomicState(swerve.getPose(), swerve.getRobotVelocity()),
        // new PathPoint(new Translation2d(10,7), Rotation2d.fromDegrees(0),
        // Rotation2d.fromDegrees(0)),
        new PathPoint(aprilTagField.getTagPose(id).get().getTranslation()
            .toTranslation2d().plus(offset),
            rotation, holonomicRotation));
    swerve.postTrajectory(path);
    return Commands.sequence(new FollowTrajectory(swerve, path, false));
  }

  public CommandBase autoToConePickup(SwerveSubsystem swerve, int id, Translation2d offset) {

    if (aprilTagField == null) {
      try {
        aprilTagField = new AprilTagFieldLayout(
            Filesystem.getDeployDirectory() + "/apriltags/2023-chargedup.json");
      } catch (Exception ignored) {
        return null;
      }
    }

    PathPlannerTrajectory path = PathPlanner.generatePath(new PathConstraints(4, 3), false,
        PathPoint.fromCurrentHolonomicState(swerve.getPose(), swerve.getRobotVelocity()),
        new PathPoint(new Translation2d(10, 7), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        new PathPoint(aprilTagField.getTagPose(id).get().getTranslation()
            .toTranslation2d().plus(offset),
            Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
     swerve.postTrajectory(path);
    return Commands.sequence(new FollowTrajectory(swerve, path, false)
        // new RunCommand(() -> shoulder.setIntakeSpeed(-.5)),
        // new RunCommand(() -> shoulder.setshoulderPosition(47, 75))
        );
  }

}

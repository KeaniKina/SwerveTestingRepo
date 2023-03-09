package frc.robot;

import frc.robot.Constants.AutoConsts;
import frc.robot.Constants.DriverControlConsts;
import frc.robot.Constants.SwerveConsts;
import frc.robot.commands.AutonomousCommands.*;
import frc.robot.commands.ClawCommands.*;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.PivotCommands.*;
import frc.robot.commands.LED_Commands.*;
import frc.robot.subsystems.*;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  public static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static PivotSubsystem pivotSubsystem = new PivotSubsystem();
  public static ClawSubsystem clawSubsystem = new ClawSubsystem();
  public static Lights lights = new Lights();

  private XboxController xbox = new XboxController(DriverControlConsts.XBOX_CONTROLLER_PORT);
  private Joystick joystick = new Joystick(DriverControlConsts.JOYSTICK_PORT);

  //AUTONOMOUS CHOICES 
  private Command highMobility = new HighMobility(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command hybridMobility = new HybridMobility(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command hybridBal = new HybridBal(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command highBal = new HighBal(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command highMobility20 = new HighMobility20(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command doNothing = new DoNothing();
  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new FieldOriented(swerveSubsystem,
        () -> xbox.getLeftY() * 0.95,
        () -> xbox.getLeftX() * 0.95,
        () -> -xbox.getRightX() * 0.95));

    selectAuto();
    configureBindings();
  }


  private void configureBindings() {

    /* SWERVE */
    new JoystickButton(xbox, 1).toggleOnTrue(
        new DriverControl(swerveSubsystem,
            () -> -xbox.getLeftY() * 0.35,
            () -> -xbox.getLeftX() * 0.35,
            () -> -xbox.getRightX() * 0.35));
    new JoystickButton(xbox, 6).toggleOnTrue(
        new DriverControl(swerveSubsystem,
            () -> -xbox.getLeftY() * 0.75,
            () -> -xbox.getLeftX() * 0.75,
            () -> -xbox.getRightX() * 0.75));
    new JoystickButton(xbox, 2).toggleOnTrue(new Lock(swerveSubsystem));
    // new JoystickButton(xbox, 4).toggleOnTrue(new Endgame(swerveSubsystem, () -> xbox.getLeftY()));
    new JoystickButton(xbox, 4).toggleOnTrue(new TankEndgame(swerveSubsystem, () -> xbox.getLeftY(), () -> xbox.getRightY()));

    // FOR TESTING
    new JoystickButton(xbox, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));
    new JoystickButton(xbox, 3).onTrue(new LandingGearIn(swerveSubsystem));

    /* CLAW */
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));

    new JoystickButton(joystick, 8).onTrue(new Go90Clockwise(clawSubsystem));
    new JoystickButton(joystick, 10).onTrue(new ToStartingPosition(clawSubsystem));
    new JoystickButton(joystick, 12).onTrue(new Go90Counterclockwise(clawSubsystem));

    new JoystickButton(joystick, 2).whileTrue(new ManualClaw(clawSubsystem, () -> joystick.getX()));

    /* PIVOT */
    new JoystickButton(joystick, 11).onTrue(new LowPickUp(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(joystick, 9).onTrue(new ParallelCommandGroup(new PivotMiddleCommand(pivotSubsystem), new MidPosition(elevatorSubsystem)));
    new JoystickButton(joystick, 7).onTrue(new TopNode(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(joystick, 5).onTrue(Tucked.getCommand(pivotSubsystem, elevatorSubsystem, clawSubsystem));
    new JoystickButton(joystick, 1).whileTrue(new PivotJoystickCommand(pivotSubsystem, () -> -joystick.getY()));

    /* ELEVATOR */
    new POVButton(joystick, 0).whileTrue(new ManualElevatorDrive(elevatorSubsystem, 0.75));
    new POVButton(joystick, 180).whileTrue(new ManualElevatorDrive(elevatorSubsystem, -0.75));

    /* LIGHTS */
    new JoystickButton(joystick, 6).toggleOnTrue(new Yellow(lights));
    new JoystickButton(joystick, 4).toggleOnTrue(new Violet(lights));

  }

  public Command getAutonomousCommand() {
    //return autoChooser.getSelected();

    // FIXME work on max accel, not final
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConsts.DRIVE_TRANSLATION_SPEED, AutoConsts.DRIVE_MAX_ACCELERATION).setKinematics(SwerveConsts.DRIVE_KINEMATICS);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
        new Translation2d(1, 1),
        new Translation2d(1, -1)
      ),
      new Pose2d(2, 0, new Rotation2d(0)),
      trajectoryConfig);

    return null;
  }

  public void selectAuto() {
    autoChooser.setDefaultOption("Do Nothing", doNothing);
    autoChooser.addOption("High Mobility 20", highMobility20);
    //autoChooser.setDefaultOption("TEST** New High Mobility", highMobileTracking);
    autoChooser.addOption("High Mobility", highMobility);
    autoChooser.addOption("Hybrid Mobility", hybridMobility);
    autoChooser.addOption("High Balance", highBal);
    autoChooser.addOption("Hybrid Balance", hybridBal);

    SmartDashboard.putData(autoChooser);
  }

}

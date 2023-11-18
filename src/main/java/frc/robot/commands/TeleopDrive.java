// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.TunableNumber;

/**
 * Drive the robot using the driver controller.
 * Left bumper slows robot down, right bumper speeds it up.
 * Precision/Boost amount can be adjusted through NetworkTables.
 */
public class TeleopDrive extends Command {
  private final boolean isFieldRelative;
  private final double deadband = RobotContainer.drive.drive.swerveController.config.angleJoyStickRadiusDeadband;
  private final TunableNumber normalMultiplier = new TunableNumber("swerve/teleop/Normal Speed Multiplier", 0.6);
  private final TunableNumber boostIncrease = new TunableNumber("swerve/teleop/Boost Speed Multiplier Increase", 0.4);
  private final TunableNumber precisionReduction = new TunableNumber("swerve/teleop/Precision Speed Multiplier Reduction", 0.4);

  /** Creates a new TeleopDrive. */
  public TeleopDrive(boolean isFieldRelative) {
    this.isFieldRelative = isFieldRelative;
    addRequirements(RobotContainer.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // controller is +ve backwards, field coordinates are +ve forward
    final double desiredXTranslation = MathUtil.applyDeadband(-RobotContainer.driverController.getLeftY(), deadband);
    // controller is +ve right, field coordinates are +ve left
    final double desiredYTranslation = MathUtil.applyDeadband(-RobotContainer.driverController.getLeftX(), deadband);
    // controller is +ve right (CW+), YAGSL expects CCW+ (+ve left)
    final double desiredAngularVelocity = MathUtil.applyDeadband(-RobotContainer.driverController.getRightX(), deadband);

    final boolean boost = RobotContainer.driverController.rightBumper().getAsBoolean();
    final boolean precision = RobotContainer.driverController.leftBumper().getAsBoolean();
    // add boost/subtract precision according to button state
    final double multiplier = normalMultiplier.get() + (boost ? boostIncrease.get() : 0) - (precision ? precisionReduction.get() : 0);
    final double maxSpeed = RobotContainer.drive.getMaxLinearSpeed() * multiplier;

    RobotContainer.drive.drive.drive(
      new Translation2d(desiredXTranslation, desiredYTranslation).times(maxSpeed),
      desiredAngularVelocity * RobotContainer.drive.getMaxAngularSpeed(),
      isFieldRelative,
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

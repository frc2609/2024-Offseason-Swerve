// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import frc.robot.RobotContainer;
import frc.robot.utils.TunableNumber;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drive extends SubsystemBase {
  /** The multiplier to apply to 'Swerve.maxAttainableLinearSpeed'. */
  public final TunableNumber linearSpeedMultiplier = new TunableNumber("swerve/teleop/Linear Speed Multiplier", 1.0);
  /** The multiplier to apply to 'Swerve.maxAttainableAngularSpeed'. */
  public final TunableNumber angularSpeedMultiplier = new TunableNumber("swerve/teleop/Angular Speed Multiplier", 1.0);
  /** The multiplier applied by default to getTeleopMaxLinearSpeed(). */
  private final TunableNumber normalMultiplier = new TunableNumber("swerve/teleop/Normal Speed Multiplier", 0.9);
  /** How much to increase the speed multiplier in getTeleopMaxLinearSpeed(). */
  private final TunableNumber boostIncrease = new TunableNumber("swerve/teleop/Boost Speed Multiplier Increase", 0.1);
  /** How much to decrease the speed multiplier in getTeleopMaxLinearSpeed(). */
  private final TunableNumber precisionReduction = new TunableNumber("swerve/teleop/Precision Speed Multiplier Reduction", 0.6);
  public final double originalMaxAngularVelocity;
  public final SwerveDrive drive;

  /** Creates a new Drive. */
  public Drive() {
    // uncomment to add human-readable data to NetworkTables
    // SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    try {
      drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(getMaxLinearSpeed(), Swerve.angleConversionFactor, Swerve.driveConversionFactor);
      // drive.setMaximumSpeeds(getMaxLinearSpeed(), getMaxLinearSpeed(), getMaxAngularSpeed());
      originalMaxAngularVelocity = drive.getSwerveController().config.maxAngularVelocity;
    } catch (IOException e) {
      DataLogManager.log("Swerve Drive File Error: " + e.getMessage());
      throw new RuntimeException("Swerve Drive failed to initialize.");
    }
  }

  @Override
  public void periodic() {
    if (linearSpeedMultiplier.hasChanged(hashCode())) {
      drive.setMaximumSpeed(MathUtil.clamp(getMaxLinearSpeed(), 0, Swerve.maxAttainableLinearSpeed));
    }
    if (angularSpeedMultiplier.hasChanged(hashCode())) {
      drive.swerveController.setMaximumAngularVelocity(originalMaxAngularVelocity * angularSpeedMultiplier.get());
    }
  }

  public double getMaxAngularSpeed() {
    return drive.getSwerveController().config.maxAngularVelocity;
  }

  public double getMaxLinearSpeed() {
    // calculate the value when called because we can't get it from YAGSL
    return Swerve.maxAttainableLinearSpeed * linearSpeedMultiplier.get();
  }

  /**
   * Calculate the robot's max linear speed during teleop based on the status
   * of the boost/precision buttons.
   * @return Max linear speed * overall boost/precision multiplier.
   */
  public double getTeleopMaxLinearSpeed() {
    final boolean boost = RobotContainer.driverController.rightBumper().getAsBoolean();
    final boolean precision = RobotContainer.driverController.leftBumper().getAsBoolean();
    // add boost/subtract precision according to button state
    final double multiplier = normalMultiplier.get() + (boost ? boostIncrease.get() : 0) - (precision ? precisionReduction.get() : 0);
    return getMaxLinearSpeed() * multiplier;
  }
}

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
import frc.robot.utils.TunableNumber;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class Drive extends SubsystemBase {
  /** The multiplier to apply to 'Swerve.maxAttainableLinearSpeed'. */
  public final TunableNumber linearSpeedMultiplier = new TunableNumber("swerve/teleop/Linear Speed Multiplier", 1.0);
  /** The multiplier to apply to 'Swerve.maxAttainableAngularSpeed'. */
  public final TunableNumber angularSpeedMultiplier = new TunableNumber("swerve/teleop/Angular Speed Multiplier", 1.0);
  public final SwerveDrive drive;

  /** Creates a new Drive. */
  public Drive() {
    // uncomment to add human-readable data to NetworkTables
    // SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
    try {
      drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive(getMaxLinearSpeed(), Swerve.angleConversionFactor, Swerve.driveConversionFactor);
      // drive.setMaximumSpeeds(getMaxLinearSpeed(), getMaxLinearSpeed(), getMaxAngularSpeed());
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
      drive.swerveController.setMaximumAngularVelocity(getMaxAngularSpeed());
    }
  }

  public double getMaxAngularSpeed() {
    return Swerve.maxAttainableAngularSpeed * angularSpeedMultiplier.get();
  }

  public double getMaxLinearSpeed() {
    return Swerve.maxAttainableLinearSpeed * linearSpeedMultiplier.get();
  }
}

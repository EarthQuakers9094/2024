// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import kotlin.math.sqrt

/**
 * An example command that uses an example subsystem.
 */
public class TeleopDrive(val swerve: Swerve, val vX: DoubleSupplier, val vY: DoubleSupplier, val omega: DoubleSupplier, val driveMode: BooleanSupplier): Command()
{
  private val controller = swerve.getSwerveController();

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
   init {
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  override fun initialize()
  {}

  // Called every time the scheduler runs while the command is scheduled.
  override fun execute() {
    val xVelocityRaw   = vX.getAsDouble()
    val yVelocityRaw   = vY.getAsDouble()
    val magnitude = sqrt(Math.pow(xVelocityRaw, 2.0) + Math.pow(yVelocityRaw, 2.0));

    val xVelocity = Math.pow(magnitude, 2.0) * xVelocityRaw;

    val yVelocity = Math.pow(magnitude, 2.0) * yVelocityRaw;

    val angVelocity = Math.pow(omega.getAsDouble(), 3.0);

    SmartDashboard.putNumber("vX", xVelocity);
    SmartDashboard.putNumber("vY", yVelocity);
    SmartDashboard.putNumber("omega", angVelocity);

    // Drive using raw values.
    swerve.drive(Translation2d(xVelocity * swerve.maximumSpeed, yVelocity * swerve.maximumSpeed),
                 angVelocity * controller.config.maxAngularVelocity,
                 driveMode.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  override fun end(interrupted: Boolean)
  {
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean
  {
    return false;
  }
}
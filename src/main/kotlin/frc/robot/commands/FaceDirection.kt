package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.controller.PIDController
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import swervelib.SwerveDrive

class FaceDirection(private val swerveDrive: Swerve, private val rotation: () -> Rotation2d, private val follow:Boolean) : Command() {

    var angle = rotation();
    var pid = PIDController(
        Constants.Drivebase.ROTATION_PID_TELEOP.kP,
        Constants.Drivebase.ROTATION_PID_TELEOP.kI,
        Constants.Drivebase.ROTATION_PID_TELEOP.kD
    );

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveDrive);
    }

    override fun initialize() {
        angle = rotation();
    }


    override fun execute() {
        val desired = if (follow) {rotation()} else {angle};

        swerveDrive.drive(
            Translation2d(0.0,0.0), 
            pid.calculate(swerveDrive.getPos().rotation.radians, desired.radians) * Constants.Drivebase.MAX_TURNING_SPEEDS,
            true);
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return (swerveDrive.getPos().rotation.radians - angle.radians) <= 0.01 && !follow;
    }

    override fun end(interrupted: Boolean) {}
}

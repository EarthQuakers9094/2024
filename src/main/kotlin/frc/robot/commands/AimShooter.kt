package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import kotlin.math.atan2
import swervelib.SwerveDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class AimShooter( private val shooter: Shooter, private val swerveDrive: Swerve, private val terminate: Boolean) : Command() {
    var height = Constants.Camera.shootElevation;
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(shooter)
    }

    override fun initialize() {
        SmartDashboard.putNumber("aiming height", height);
        SmartDashboard.putBoolean("aim shooter running", true);
    }


    override fun execute() {
        height = SmartDashboard.getNumber("aiming height", height);
        
        // val ydif = Constants.Camera.yPositionOfSpeaker-location.getY();
        // val xdif = Constants.Camera.xPositionOfSpeaker()-location.getX();

        // val distance = Math.sqrt( ydif*ydif + 
        //                           xdif*xdif );

        val distance = swerveDrive.speakerDistance()

        val angle = atan2(Constants.Camera.shootElevation,distance);

        // both are in radians? https://kotlinlang.org/api/latest/jvm/stdlib/kotlin.math/atan2.html#:~:text=Returns%20the%20angle%20theta%20of,from%20%2DPI%20to%20PI%20radians.
        shooter.setAngle(angle.coerceIn(0.0, 0.887))
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return shooter.atAngle() && terminate;
    }

    override fun end(interrupted: Boolean) {
        SmartDashboard.putBoolean("aim shooter running", false);
    }
}

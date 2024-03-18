package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import kotlin.math.atan2
import kotlin.math.pow
import swervelib.SwerveDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class AimShooter( private val shooter: Shooter, private val swerveDrive: Swerve, private val terminate: Boolean, private val camera: PhotonCamera) : Command() {
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
        //height = SmartDashboard.getNumber("aiming height", height);
        
        // val ydif = Constants.Camera.yPositionOfSpeaker-location.getY();
        // val xdif = Constants.Camera.xPositionOfSpeaker()-location.getX();

        // val distance = Math.sqrt( ydif*ydif + 
        //                           xdif*xdif );

        val distance = camera.latestResult.targets.filter { it.fiducialId == 4 }.get(0)?.area!!//swerveDrive.speakerDistance()

        // v1 val angle = -0.978752 + (8.42889 * distance) - (14.1454 * distance.pow(2)) + (8.09307 * (distance.pow(3)))//atan2(Constants.Camera.shootElevation,distance) - 3.5 * Math.PI/180.0;
        // v2 val angle = 1.83801 - (18.1636 * distance) +( 75.4928 *  distance.pow(2)) - (118.954 * distance.pow(3)) + (63.5236 * distance.pow(4))
        // v3 val angle = 2.3925 - (26.171 * distance) + (119.313 * distance.pow(2)) - (232.193 *  distance.pow(3)) + (201.078 * distance.pow(4)) - (62.5248 * distance.pow(5))
        // v4 val angle = 4.26707 - (46.9928 * distance) + (208.464 * distance.pow(2)) - (416.168 * distance.pow(3)) + (384.272 * distance.pow(4)) - (132.984 * distance.pow(5))
        val angle = 7.50023 - (189.565 * distance) + (1999.42 * distance.pow(2)) - (10831.8 * distance.pow(3)) + (32686.6 * distance.pow(4)) - (55084.5 * distance.pow(5)) + (48226.9 * distance.pow(6)) - (17006.9 * distance.pow(7))
        //0.83, 0.9
        //0.53, 0.72 
        //0.36, 0.6
        //0.28,0.45
        //     ///////////////////0.27, 0.424617 // untested please test
        //0.20, 0.45
        //0.60, 0.686734

        //0.15 ,0.38
        //0.13, 0.362000


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

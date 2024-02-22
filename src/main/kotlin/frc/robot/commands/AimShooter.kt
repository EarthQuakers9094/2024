package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import swervelib.SwerveDrive

class AimShooter(private val camera: PhotonCamera, private val shooter: Shooter, private val swerveDrive: Swerve) : Command() {
    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(shooter)
    }

    override fun initialize() {}


    override fun execute() {
        val location = swerveDrive.getPos();
        
        val ydif = Constants.Camera.yPositionOfSpeaker-location.getY();
        val xdif = Constants.Camera.xPositionOfSpeaker-location.getX();

        val distance = Math.sqrt( ydif*ydif + 
                                  xdif*xdif );

        val angle = atan2(Constants.Camera.shootElevation,distance);

        // both are in radians? https://kotlinlang.org/api/latest/jvm/stdlib/kotlin.math/atan2.html#:~:text=Returns%20the%20angle%20theta%20of,from%20%2DPI%20to%20PI%20radians.
        shooter.setAngle(angle)
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return shooter.atAngle();
    }

    override fun end(interrupted: Boolean) {}
}

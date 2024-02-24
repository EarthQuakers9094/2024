package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.controller.PIDController
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Swerve
import frc.robot.utils.MovingAverage
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import swervelib.SwerveDrive
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class FaceDirection(private val swerveDrive: Swerve, private val rotation: () -> Rotation2d, private val follow:Boolean) : Command() {

    var angle = rotation();

    var pid = PIDController(
        Constants.Drivebase.ROTATION_PID_TELEOP.kP,
        Constants.Drivebase.ROTATION_PID_TELEOP.kI,
        Constants.Drivebase.ROTATION_PID_TELEOP.kD
    );

    var angleRollingAverage = MovingAverage(40);

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerveDrive);

        pid.enableContinuousInput(-Math.PI, Math.PI);
        SmartDashboard.putData("face direction pid", pid);
    }

    override fun initialize() {
        angle = rotation();
        SmartDashboard.putBoolean("face direction running", true);
    }


    override fun execute() {
        val desired = if (follow) {rotation()} else {angle};

        SmartDashboard.putNumber("desired rotation", desired.radians);
        
        val angle = swerveDrive.getPos().rotation.radians
        
        SmartDashboard.putNumber("my angle", angle);

        val pidoutput = pid.calculate(angle, desired.radians);

        SmartDashboard.putNumber("pidoutput", pidoutput);

        angleRollingAverage.addValue(angle)

        swerveDrive.drive(
            Translation2d(0.0,0.0), 
            pidoutput * Constants.Drivebase.MAX_TURNING_SPEEDS,
            true);
    }

    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return Math.abs(angleRollingAverage.getAverage() - angle.radians) <= 0.06 && !follow;
    }

    override fun end(interrupted: Boolean) {
        swerveDrive.drive(Translation2d(0.0,0.0), 0.0, true);
        SmartDashboard.putBoolean("face direction running", false);
        SmartDashboard.putBoolean("face direction interupted", interrupted);
    }
}

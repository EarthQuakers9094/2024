package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Elevator
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.math.geometry.Rotation2d
import swervelib.SwerveDrive
import edu.wpi.first.wpilibj.DriverStation


class SetValue(private val subsystem: SubsystemBase, 
               private val initial: () -> Unit, 
               private val finished: () -> Boolean) : Command() {
    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        DriverStation.reportError("hello :3 from set value", true);
        initial()
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return finished();
    }

    override fun end(interrupted: Boolean) {}

    companion object {
        fun setShootingAngle(shooter: Shooter, angle: Double): SetValue {
            return SetValue(shooter, {shooter.setAngle(angle)}) 
                { shooter.atAngle() };
        }
        fun setHeight(elevator: Elevator, height: Double): SetValue {
            return SetValue(elevator, {elevator.setPosition(height)})
                { elevator.atPosition() };
        }
    }
}

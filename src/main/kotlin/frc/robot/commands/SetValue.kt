package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Elevator
import org.photonvision.PhotonCamera
import kotlin.math.atan2
import edu.wpi.first.wpilibj2.command.SubsystemBase

class SetValue(private val subsystem: SubsystemBase, 
               private val require: Boolean,
               private val initial: () -> Unit, 
               private val finished: () -> Boolean) : Command() {
    init {
        if (require) {
            addRequirements(subsystem)
        }
    }

    override fun initialize() {
        initial()
    }

    override fun execute() {
    }

    override fun isFinished(): Boolean {
        return finished();
    }

    override fun end(interrupted: Boolean) {}

    companion object {
        fun setShootingAngle(shooter: Shooter, require:Boolean,angle: Double): SetValue {
            return SetValue(shooter, require, {shooter.setAngle(angle)}) 
                { shooter.atAngle() };
        }
        fun setHeight(elevator: Elevator, require:Boolean, height: Double): SetValue {
            return SetValue(elevator, require, {elevator.setPosition(height)})
                { elevator.atPosition() };
        }
    }
}

package frc.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.Elevator

class SetValue(
        private val subsystem: SubsystemBase,
        private val require: Boolean,
        private val initial: () -> Unit,
        private val finished: () -> Boolean
) : Command() {
    init {
        addRequirements(subsystem)
    }

    override fun initialize() {
        DriverStation.reportError("hello :3 from set value", true)
        initial()
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        return finished()
    }

    override fun end(interrupted: Boolean) {}

    companion object {
        fun setShootingAngle(shooter: Shooter, require: Boolean, angle: Double): Command {
            return SetValue(shooter, require, { shooter.setAngle(angle) }) { shooter.atAngle() }
        }
        fun setHeight(elevator: Elevator, require: Boolean, height: Double): Command {
            return SetValue(elevator, require, { elevator.setPosition(height) }) {
                elevator.atPosition()
            }
        }
        fun climb(elevator: Elevator, require: Boolean): Command {
            return SequentialCommandGroup(
                    SetValue(
                            elevator,
                            require,
                            { elevator.setPosition(Constants.Elevator.maxHeight) }
                    ) { elevator.atPosition() },
                    InstantCommand({ elevator.climbing = true }, elevator),
                    SetValue(
                            elevator,
                            require,
                            { elevator.setPosition(Constants.Elevator.minHeight) }
                    ) { elevator.atPosition() },
                    InstantCommand({ elevator.climbing = false }, elevator)
            )
        }
    }
}

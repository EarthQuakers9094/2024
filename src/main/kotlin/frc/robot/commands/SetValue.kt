package frc.robot.commands

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Shooter

class SetValue(
        private val subsystem: SubsystemBase,
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
        fun setShootingAngle(shooter: Shooter, angle: Double): Command {
            return SetValue(shooter, { shooter.setAngle(angle) }) { shooter.atAngle() }
        }

        fun setHeight(elevator: Elevator, height: Double): Command {
            return SetValue(elevator, { elevator.setPosition(height) }) {
                elevator.atPosition()
            }
        }

        // fun climb(elevator: Elevator): Command {
        //     return SequentialCommandGroup(
        //             SetValue(
        //                     elevator,
        //                     { elevator.setPosition(Constants.Elevator.maxHeight) }
        //             ) { elevator.atPosition() },
        //             InstantCommand({ elevator.setClimbing(true) }, elevator),
        //             SetValue(
        //                     elevator,
        //                     { elevator.setPosition(Constants.Elevator.minHeight) }
        //             ) { elevator.atPosition() },
        //             InstantCommand({ elevator.setClimbing(false) }, elevator)
        //     )
        // }
    }
}

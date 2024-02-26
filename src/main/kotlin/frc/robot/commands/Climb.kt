package frc.robot.commands

import frc.robot.subsystems.Elevator
import frc.robot.commands.SetValue
import frc.robot.Constants
import frc.robot.commands.CommandSequence
import edu.wpi.first.wpilibj2.command.InstantCommand


class Climb(private val elevator: Elevator):CommandSequence() {
    override val commands = listOf (
        SetValue(elevator,
            { elevator.setPosition(Constants.Elevator.maxHeight) }) { elevator.atPosition() },
        InstantCommand({ elevator.setClimbing(true) }, elevator),
        SetValue( elevator, { elevator.setPosition(Constants.Elevator.minHeight) }) { elevator.atPosition() });

    override fun finally(interrupted: Boolean) {
        elevator.setClimbing(false);
        elevator.setPosition(Constants.Elevator.minHeight);
    }

}
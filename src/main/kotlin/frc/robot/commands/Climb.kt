package frc.robot.commands

import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Shooter
import frc.robot.commands.SetValue
import frc.robot.Constants
import frc.robot.commands.CommandSequence
import edu.wpi.first.wpilibj2.command.InstantCommand


class Climb(private val elevator: Elevator, private val shooter: Shooter):CommandSequence() {
    override val commands = listOf (
        InstantCommand({ elevator.setClimbing(true) }, elevator),
        SetValue( elevator, { elevator.setPosition(Constants.Elevator.minHeight) }) { elevator.atPosition() });

    override fun finally(interrupted: Boolean) {
        elevator.setClimbing(false);
        elevator.setPosition(Constants.Elevator.minHeight);
    }

}
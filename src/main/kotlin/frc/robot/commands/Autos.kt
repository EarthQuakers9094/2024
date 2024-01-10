package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Command

class Autos private constructor() {
    init {
        throw UnsupportedOperationException("This is a utility class!")
    }

    companion object {
        /** Example static factory for an autonomous command.  */
        fun exampleAuto(): Command {
            return Commands.none();
        }
    }
}

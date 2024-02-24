

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.Command

abstract class CommandSequence() {
    abstract val commands: List<Command>;
    abstract fun finally(interrupted: Boolean);

    fun build(): Command {

        var sequence = SequentialCommandGroup();
        
        for (i in commands) {
            sequence.addCommands(i);
        }

        return sequence.finallyDo({b -> finally(b)});
    }
}
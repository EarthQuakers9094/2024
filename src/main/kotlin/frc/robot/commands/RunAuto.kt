import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup

class RunAuto(auto: String) : SequentialCommandGroup() {
    init {

        // Create a path following command using AutoBuilder. This will also trigger event markers.

        addCommands(PathPlannerAuto(auto))
    }
}

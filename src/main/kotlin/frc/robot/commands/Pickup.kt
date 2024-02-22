
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.Timer
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Intake
import frc.robot.Constants
import frc.robot.commands.SetValue
import frc.robot.commands.GotoPose
import java.util.function.BooleanSupplier
import frc.robot.subsystems.Elevator

class Pickup(private val shooter: Shooter, elevator: Elevator,private val intake: Intake) : SequentialCommandGroup() {

    init {
        addRequirements(shooter);   

        val supplier:BooleanSupplier = BooleanSupplier {shooter.noteIn()};
        
        addCommands(
            GotoPose(shooter,elevator,Constants.Poses.pickup,false),          
            InstantCommand(object: Runnable {
                    override fun run() {
                        shooter.intake();
                        intake.startIntaking();
                    }
            }),
            WaitUntilCommand(supplier),
            InstantCommand(object: Runnable {
                override fun run() {
                    intake.stopIntaking();
                    shooter.stopIntaking()
                }
            })
        );

        finallyDo(
            object: Runnable {
                override fun run() {
                    intake.stopIntaking();
                    shooter.stopIntaking();
                    GotoPose(shooter, elevator, Constants.Poses.resting, true).schedule();
                }
            }
        );
    }
}

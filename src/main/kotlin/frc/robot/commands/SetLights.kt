package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lights.LightEffect
import frc.robot.subsystems.Lights

/** An example command that uses an example subsystem.  */
class SetLights(private val lights: Lights, private val effect: LightEffect) : Command() {

    /** Called when the command is initially scheduled.  */
    override fun initialize() {
        lights.currentEffect = effect
    }

    /** Returns true when the command should end.  */
    override fun isFinished(): Boolean {
        return true
    }
}

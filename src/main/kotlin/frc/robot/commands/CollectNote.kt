package frc.robot.commands

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Swerve
import frc.robot.utils.MovingAverage
import org.photonvision.PhotonCamera

class CollectNote(
    private val intake: Intake, private val swerve: Swerve,
    private val camera: PhotonCamera
) : Command() {

    private var updatesSinceLastTarget = 0
    private var updates = 0

    private val omegaPID =
            PIDController(
                    Constants.Auto.TARGET_ROTATION.kP,
                    Constants.Auto.TARGET_ROTATION.kI,
                    Constants.Auto.TARGET_ROTATION.kD
            )

    private var notePos = MovingAverage(10)

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(swerve)
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    override fun initialize() {}

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
     * called repeatedly until [isFinished] returns true.)
     */
    override fun execute() {
        updates++

        val res = camera.latestResult
        if(updatesSinceLastTarget < 50) {

        }

        if (res.hasTargets()) {
            // if (true) {

            val target = res.bestTarget
            if (target.fiducialId == 5) {
                updatesSinceLastTarget = 0
                DriverStation.reportWarning(
                        "We are currently using the fiducialId 5 for testing",
                        arrayOf()
                )
                //right yaw is negative
                val yaw = notePos.addValue(target.yaw)
                //positive is right
                val calc = omegaPID.calculate(yaw)
                // DriverStation.reportError("hello there this is me", arrayOf())
                SmartDashboard.putNumber("Note yaw", yaw)
                SmartDashboard.putNumber("Calculated omega", calc)

                swerve.drive(Translation2d(0.0, 0.0), calc, false)
            } else {
                updatesSinceLastTarget++
            }
        } else {
            updatesSinceLastTarget++
        }
    }

    /**
     * Returns whether this command has finished. Once a command finishes -- indicated by this
     * method returning true -- the scheduler will call its [end] method.
     *
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use [InstantCommand][edu.wpi.first.wpilibj2.command.InstantCommand] for such
     * an operation.
     *
     * @return whether this command has finished.
     */
    override fun isFinished(): Boolean {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return updatesSinceLastTarget >= 500 / 20// || (updates >= 15 && abs(lastTagPos) <= 0.4)
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally --
     * that is, it is called when [isFinished] returns true -- or when it is interrupted/canceled.
     * This is where you may want to wrap up loose ends, like shutting off a motor that was being
     * used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    override fun end(interrupted: Boolean) {}
}

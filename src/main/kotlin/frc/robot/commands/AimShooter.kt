package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.Shooter
import org.photonvision.PhotonCamera
import kotlin.math.atan2

class AimShooter(private val camera: PhotonCamera, private val shooter: Shooter) : Command() {



    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(shooter)
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    override fun initialize() {}

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
     * called repeatedly until [isFinished] returns true.)
     */
    override fun execute() {
        val res = camera.latestResult
        if (res.hasTargets()) {
            //big boy one-liner
            val preferredTarget = res.targets.filter {Constants.Shooter.validTargets.contains(it.fiducialId)}.sortedBy {it.fiducialId}[0]
            val camToTarget = preferredTarget.bestCameraToTarget
            val distance = camToTarget.x + Constants.Shooter.distanceOffset
            val height = camToTarget.z + Constants.Shooter.heightOffset

            // let's get triggy with it (the title of the worksheet I had to do in math today)
            val angle = atan2(height, distance) + Constants.Shooter.angleOffset

            // both are in radians? https://kotlinlang.org/api/latest/jvm/stdlib/kotlin.math/atan2.html#:~:text=Returns%20the%20angle%20theta%20of,from%20%2DPI%20to%20PI%20radians.
            shooter.setAngle(angle)

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
        return false
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

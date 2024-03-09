package frc.robot

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.wpilibj2.command.CommandScheduler

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
    private var autonomousCommand: Command? = null
    private var robotContainer: RobotContainer? = null
    private var disabledTimer = Timer();
    

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = RobotContainer()

        // Dean Kamen himself demands to know that we are using kotlin
        HAL.report(tResourceType.kResourceType_Language, tInstances.kLanguage_Kotlin);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     *
     * This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    override fun robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()


    }

    /** This function is called once each time the robot enters Disabled mode.  */
    override fun disabledInit() {
        robotContainer?.setMotorBrake(true);
        disabledTimer.reset();
        disabledTimer.start();
        robotContainer?.cleanUp()
    }

    /** This function is called periodically when disabled.  */
    override fun disabledPeriodic() {
        if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
        {
          robotContainer?.setMotorBrake(false);
          disabledTimer.stop();
        }
    }

    /** This autonomous runs the autonomous command selected by your [RobotContainer] class.  */
    override fun autonomousInit() {
        autonomousCommand = robotContainer?.autonomousCommand
        
        robotContainer?.setMotorBrake(true);

        // Schedule the autonomous command (example)
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before scheduling it
        autonomousCommand?.schedule()
        robotContainer?.shooter?.enable();
    }

    /** This function is called periodically during autonomous.  */
    override fun autonomousPeriodic() {}

    /** This function is called once when teleop is enabled.  */
    override fun teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // Note the Kotlin safe-call(?.), this ensures autonomousCommand is not null before cancelling it
        robotContainer?.shooter?.enable();
        autonomousCommand?.cancel()
        robotContainer?.setMotorBrake(true);
    }

    /** This function is called periodically during operator control.  */
    override fun teleopPeriodic() {
        robotContainer!!.teleperiodic()
    }

    /** This function is called once when test mode is enabled.  */
    override fun testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll()
    }

    /** This function is called periodically during test mode.  */
    override fun testPeriodic() {}

    /** This function is called once when the robot is first started up.  */
    override fun simulationInit() {}

    /** This function is called periodically whilst in simulation.  */
    override fun simulationPeriodic() {}
}

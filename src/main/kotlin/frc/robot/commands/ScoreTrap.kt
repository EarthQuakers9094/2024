package frc.robot.commands


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.Constants
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Shooter

// TODO: Add your sequential commands in the super constructor call,
//       e.g. SequentialCommandGroup(OpenClawCommand(), MoveArmCommand())
class ScoreTrap(elevator: Elevator, shooter: Shooter) : SequentialCommandGroup(

    SetValue.setHeight(elevator, true, Constants.Elevator.maxHeight),
    SetValue.setShootingAngle(shooter, true, Constants.Shooter.trapAngle)

)

package frc.robot.commands


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Shooter

// TODO: Add your sequential commands in the super constructor call,
//       e.g. SequentialCommandGroup(OpenClawCommand(), MoveArmCommand())
class ClimbAndTrap(elevator: Elevator, shooter: Shooter) : SequentialCommandGroup(SetValue.climb(elevator, true), ScoreTrap(elevator, shooter))

package first.robot;


import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.claw.Claw;
import first.robot.subsystems.claw.Tilt;
import first.robot.subsystems.elevator.Elevator;
import first.robot.subsystems.pivot.Pivot;

import first.robot.subsystems.drive.EasySwerveModuleIOReal;
import first.robot.subsystems.vision.VisionIOLimelight;
import first.robot.subsystems.claw.ClawIOReal;
import first.robot.subsystems.claw.TiltIOReal;
import first.robot.subsystems.elevator.ElevatorIOReal;
import first.robot.subsystems.pivot.PivotIOReal;
import first.robot.subsystems.drive.GyroIOPigeon2;

public class Subsystem {

    private static Subsystem instance = null;

    // Subsystems
    private final Drive drivetrain;
    private final Claw intake;
    private final Tilt tilt;
    //private final Vision vision;
    private final Elevator elevator;
    private final Pivot pivot;

    private Subsystem() 
    {
        drivetrain = new Drive(
            new GyroIOPigeon2(), 
            new EasySwerveModuleIOReal(0), 
            new EasySwerveModuleIOReal(1), 
            new EasySwerveModuleIOReal(2), 
            new EasySwerveModuleIOReal(3),
            pose ->{}
        );
        intake = new Claw(new ClawIOReal());
        tilt = new Tilt(new TiltIOReal());
        // vision = new Vision(new VisionIOLimelight());
        elevator = new Elevator(new ElevatorIOReal());
        pivot = new Pivot(new PivotIOReal());
    }

    public static Subsystem getInstance() 
    {
        if (instance == null) {
            instance = new Subsystem();
        }
        return instance;
    }

    public Drive getDrivetrain() 
    {
        return drivetrain;
    }

    public Claw getIntake() 
    {
        return intake;
    }

    public Tilt getTilt() 
    {
        return tilt;
    }

    public Elevator getElevator() 
    {
        return elevator;
    }

    public Pivot getPivot() 
    {
        return pivot;
    }

}

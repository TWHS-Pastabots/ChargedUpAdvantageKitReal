package first.robot.subsystems.claw;
 
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
 
public class Claw implements Mechanism {
    private final ClawIO io;
    private final ClawIOInputsAutoLogged inputs = new ClawIOInputsAutoLogged();
 

    private final Alert ClawIntakeDisconnectedAlert =
            new Alert("Intake Disconnected","Claw intake motor is disconnected.", Alert.Level.HIGH);
 
    public Claw(ClawIO io) {
        this.io = io;
        Scheduler.getDefault().addPeriodic(this::periodic);
    }
 
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Claw", inputs);
 
        // Update alerts
        ClawIntakeDisconnectedAlert.set(!inputs.intakeConnected);
    }
 
    /** Commands the claw to the given angle using onboard closed-loop control. */
    public void setPosition(Rotation2d position) {
        io.setPosition(position);
    } 
 
    /** Runs the claw at the specified open-loop voltage. */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void setPower(double power){
        io.setPower(power);
    }

    public double getTemperature(){
      return inputs.intakeTemperature;
    }

 
    /** Stops the claw. */
    public void stop() {
        io.stop();
    }
 
    /** Returns the current measured angle of the claw. */
    public Rotation2d getPosition() {
        return inputs.intakePosition;
    }
 
    /** Returns true if the claw is within the given tolerance of the target angle. */
    public boolean atSetpoint(Rotation2d target, Rotation2d tolerance) {
        return Math.abs(inputs.intakePosition.minus(target).getRadians()) < tolerance.getRadians();
    }
}
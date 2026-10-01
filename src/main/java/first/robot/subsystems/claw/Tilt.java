


package first.robot.subsystems.claw;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import first.robot.Constants.ClawConstants.TiltState;

import static first.robot.Constants.ClawConstants.TiltState;

import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;
 
public class Tilt implements Mechanism {
    private final TiltIO io;
    private final TiltIOInputsAutoLogged inputs = new TiltIOInputsAutoLogged();
    private TiltState state = TiltState.TRANSITION;

    private final Alert ClawTiltDisconnectedAlert =
            new Alert("Tilt Disconnected","Claw tilt motor is disconnected.", Alert.Level.HIGH);
 
    public Tilt(TiltIO io) {
        this.io = io;
        Scheduler.getDefault().addPeriodic(this::periodic);
    }
 
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tilt", inputs);
        io.setPosition(state.angle);
        // Update alerts
        ClawTiltDisconnectedAlert.set(!inputs.tiltConnected);
    }
 
    /** Commands the claw to the given angle using onboard closed-loop control. */
    public void setPosition(Rotation2d position) {
        io.setPosition(position);
    }
 
    /** Runs the claw at the specified open-loop voltage. */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
 
    /** Stops the claw. */
    public void stop() {
        io.stop();
    }
 
    /** Returns the current measured angle of the claw. */
    public Rotation2d getPosition() {
        return inputs.tiltPosition;
    }
 
    /** Returns true if the claw is within the given tolerance of the target angle. */
    public boolean atSetpoint(Rotation2d target, Rotation2d tolerance) {
        return Math.abs(inputs.tiltPosition.minus(target).getRadians()) < tolerance.getRadians();
    }

    public double getTemperature(){
      return inputs.tiltTemperature;
    }

      public void transitionState()
  {
    state = TiltState.TRANSITION;
  }
  public void scoreConeState()
  {
    state = TiltState.CONESCORE;
  }
  public void cubeIntakeFront()
  {
    state = TiltState.CUBEINTAKEFRONT;
  }
  public void cubeIntakeBack()
  {
    state = TiltState.CUBEINTAKEBACK;
  }
  public void coneIntakeFront()
  {
    state = TiltState.CONEINTAKEFRONT;
  }
  public void coneIntakeBack()
  {
    state = TiltState.CONEINTAKEBACK;
  }
  public void coneScoreState()
  {
    state = TiltState.CONESCORE;
  }
  public void cubeScoreState()
  {
    state = TiltState.CUBESCORE;
  }
}


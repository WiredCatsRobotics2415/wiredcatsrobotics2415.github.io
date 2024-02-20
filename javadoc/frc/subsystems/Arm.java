package frc.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Constants.Arm.EncoderOption;
import frc.sim.PhysicsSim;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private TalonFX leftMotor;
  private TalonFX rightMotor;
  private AnalogPotentiometer potentiometer;
  private ArmFeedforward ff = new ArmFeedforward(Constants.Arm.KA, 0, Constants.Arm.KV, Constants.Arm.KA);
  private ProfiledPIDController pid = new ProfiledPIDController(
      Constants.Arm.KP,
      0,
      Constants.Arm.KD,
      new TrapezoidProfile.Constraints(
          Constants.Arm.VELO_MAX,
          Constants.Arm.ACCEL_MAX));
  private double newGravityVolts = 0.0d;

  private MechanismLigament2d goalLigament;
  private MechanismLigament2d positionLigament;

  private double goalInRotations = Constants.Arm.MIN_ROTATIONS;
  private static Arm instance;

  public Arm() {
    potentiometer = new AnalogPotentiometer(RobotMap.Arm.ANALOG_POT_PORT, 0.75);

    configureMotors();

    Mechanism2d armMechanism2d = new Mechanism2d(3, 3);

    MechanismRoot2d armGoal2d = armMechanism2d.getRoot("armGoal", 1.5, 0);
    goalLigament = armGoal2d.append(new MechanismLigament2d("armGoal", 0.75, 0));
    goalLigament.setColor(new Color8Bit(Color.kPaleGreen));

    MechanismRoot2d armPosition2d = armMechanism2d.getRoot("armPosition", 1.5, 0);
    positionLigament = armPosition2d.append(new MechanismLigament2d("armPosition", 0.75, 0));
    positionLigament.setColor(new Color8Bit(Color.kGreen));

    SmartDashboard.putData("Arm Mechanism", armMechanism2d);

    if (Robot.isSimulation()) {
      PhysicsSim.getInstance().addTalonFX(leftMotor, 0.001);
      PhysicsSim.getInstance().addTalonFX(rightMotor, 0.001);
    }
  }

  /**
   * Configures the Arm's motors. The right motor is inverted and follows the left motor.
   */
  public void configureMotors() {
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    leftMotor = new TalonFX(RobotMap.Arm.LEFT_MOTOR_PORT);
    leftMotor.getConfigurator().apply(feedbackConfigs);

    rightMotor = new TalonFX(RobotMap.Arm.RIGHT_MOTOR_PORT);
    rightMotor.setControl(new StrictFollower(leftMotor.getDeviceID()));
    rightMotor.setInverted(true);

    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }

  /**
   * Sets the left arm's motor to the desired voltage, calculated by the
   * feedforward object and PID subsystem.
   * 
   * @param output   the output of the ProfiledPIDController
   * @param setpoint the setpoint state of the ProfiledPIDController, for
   *                 feedforward
   */
  private void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = ff.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    double voltOut = newGravityVolts + output + feedforward;
    leftMotor.setVoltage(voltOut);
    SmartDashboard.putNumber("Arm Volt out", voltOut);
  }

  /**
   * @return potentiometer value in rotations.
   */
  private double getPotRotations() {
    return (potentiometer.get() + Constants.Arm.POT_OFFSET);
  }

  /**
   * @return measurement in rotations. Handles selection of encoder defined in
   *         Constants.
   *         If in simulation, use the analog port window to control the simulated
   *         position.
   */
  private double getMeasurement() {
    if (Robot.isSimulation()) {
      return getPotRotations();
    }

    double rotations = 0.0d;
    if (Constants.Arm.ENCODER_TO_USE.equals(EncoderOption.ANALOG_POT)) {
      rotations = getPotRotations();
    } else {
      rotations = Constants.Arm.falconToRotations(leftMotor.getPosition().getValue());
    }
    newGravityVolts = (Constants.Arm.KG_PROPORTION * (rotations * 360)) * Constants.Arm.KG;
    return rotations;
  }

  /**
   * Sets the goal of the arm in rotations. Does NOT account for the goal being a
   * value
   * outside of the minimum or maximum rotations.
   * 
   * @param goalInRotations
   */
  public void setGoal(double goalInRotations) {
    this.goalInRotations = goalInRotations;
    pid.setGoal(new TrapezoidProfile.State(goalInRotations, 0));
  }

  /**
   * @return A command to increase the arm's current goal by one degree.
   *         Does not go above max rotations defined in constants.
   */
  public Command increaseGoal() {
    return new RepeatCommand(new InstantCommand(() -> {
      if (goalInRotations >= Constants.Arm.MAX_ROTATIONS) {
        goalInRotations = Constants.Arm.MAX_ROTATIONS;
        return;
      }
      goalInRotations += (1 / 360.0d);
      System.out.println("Goal increase: " + goalInRotations);
      this.setGoal(goalInRotations);
    }));
  }

  /**
   * @return A command to decrease the arm's current goal by one degree.
   *         Does not go below min rotations defined in constants.
   */
  public Command decreaseGoal() {
    return new RepeatCommand(new InstantCommand(() -> {
      if (goalInRotations <= Constants.Arm.MIN_ROTATIONS) {
        goalInRotations = Constants.Arm.MIN_ROTATIONS;
        return;
      }
      goalInRotations -= (1 / 360.0d);
      System.out.println("Goal decrease: " + goalInRotations);
      this.setGoal(goalInRotations);
    }));
  }

  @Override
  public void periodic() {
    double measurement = getMeasurement();
    SmartDashboard.putNumber("Arm Measurement", measurement);
    useOutput(pid.calculate(measurement), pid.getSetpoint());
    positionLigament.setAngle(measurement * 360);
    goalLigament.setAngle(goalInRotations * 360);
  }
}

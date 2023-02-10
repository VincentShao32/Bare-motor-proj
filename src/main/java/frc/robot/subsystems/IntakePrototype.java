package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class IntakePrototype implements Subsystem{

    private int timeOutMs = 10;
    private static IntakePrototype s_inPrototype;

    private Solenoid intakeSolenoid;
    private IntakeStates intakeState = IntakeStates.OFF_DEPLOYED;
    private TalonFX mIntakeMotor;
    private Compressor compressor;

    public static IntakePrototype getInstance(){
        if(s_inPrototype == null){
            s_inPrototype = new IntakePrototype();

        }
        return s_inPrototype;
    }

    public IntakePrototype(){
        intakeSolenoid = new Solenoid(
            16,
            PneumaticsModuleType.REVPH, 
            0
        );
        compressor = new Compressor(16, PneumaticsModuleType.REVPH);
        compressor.enableDigital();
        mIntakeMotor = new TalonFX(0);
        configureMotor(mIntakeMotor, false); 
        setState(IntakeStates.OFF_RETRACTED);

    }

    private void configureMotor(TalonFX talon, boolean inverted){
        talon.setInverted(inverted);
        talon.configVoltageCompSaturation(12.0, timeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
        talon.config_kF(0, 0.05, timeOutMs);
        talon.config_kP(0, 0.12, timeOutMs);
        talon.config_kI(0, 0, timeOutMs);
        talon.config_kD(0, 0, timeOutMs);
    }

    public enum IntakeStates {
        OFF_RETRACTED(false, 0),
        ON_RETRACTED(false, 1),
        REV_RETRACTED(false, -1),
        OFF_DEPLOYED(true, 0),
        ON_DEPLOYED(true, 1),
        REV_DEPLOYED(true, -1);

        boolean value;
        int direction;

        private IntakeStates(boolean value, int direction) {
            this.value = value;
            this.direction = direction;
        }
    } 

    public void setState(IntakeStates state) {
        this.intakeState = state;
        intakeSolenoid.set(intakeState.value);
        final int offset = 8000;
        mIntakeMotor.set(ControlMode.Velocity, offset * intakeState.direction);
    }

    public void testVelo(int direction) {
        mIntakeMotor.set(ControlMode.Velocity, 8000 * direction);
    }

    public void testSolenoid(boolean b) {
        intakeSolenoid.set(b);
    }

    public int getVelocityDirection() {
        return intakeState.direction;
    }

    public boolean getIntakeDeployed() {
        return intakeState.value;
    }

}

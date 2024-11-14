package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.State.iState;

import frc.lib.util.CANSparkFlexUtil;
import frc.lib.util.CANSparkFlexUtil.Usage;;


public class intakeSubsystem extends SubsystemBase {
    public CANSparkFlex m_wheelMotor;
    public CANSparkFlex m_floorMotor;
    public PWMSparkMax m_frontMotor;
    public iState Istate;

    public intakeSubsystem(){
        m_wheelMotor = new CANSparkFlex(Constants.IntakeSystem.wheelMotorID, MotorType.kBrushless);
        m_wheelMotor.setIdleMode(IdleMode.kBrake);

        m_floorMotor = new CANSparkFlex(Constants.IntakeSystem.floorMotorID, MotorType.kBrushless); //tennisgrip

        m_frontMotor = new PWMSparkMax(Constants.IntakeSystem.frontMotorID); //tennisgrip

        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_wheelMotor, Usage.kVelocityOnly);
        Istate = frc.robot.State.iState.STOP;

        m_wheelMotor.setInverted(false);
        //used to be false before prac match
        m_frontMotor.setInverted(false); //was true
        m_floorMotor.setInverted(false); //was true

        goIntakeWheelState(iState.STOP);

        
    }

    //INTAKE SPIN
    public void goIntakeWheelState(iState state){
        switch (state) {
            case IN:
                m_wheelMotor.set(-0.55); 
                m_frontMotor.set(-0.75); //was 0.25
                m_floorMotor.set(0.75);

                Istate = iState.IN;
                break;
            case OUT:
                m_wheelMotor.set(0.55);
                m_frontMotor.set(-0.50);
                m_floorMotor.set(0.50);

                Istate = iState.OUT;
                break;
            case STOP:
                m_wheelMotor.set(0);
                m_frontMotor.set(0);
                m_floorMotor.set(0);

                Istate = iState.STOP;
                break;
            case AMP_IN:
                m_wheelMotor.set(0.25); 
                m_frontMotor.set(0.75); //was 0.25
                m_floorMotor.set(0.75);

                Istate = iState.AMP_IN;
                break;

        }
    }

}
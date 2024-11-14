package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.eState;
import frc.robot.State.tState;

public class TrapAmpSubsystem extends SubsystemBase {
    public PWMSparkMax m_trapMotor;
    public CANSparkMax m_RightArmMotor; //swapped PWM to CAN
    public CANSparkMax m_LeftArmMotor; //swapped PWM to CAN
    
    public tState tState; //Spinner
    public eState eState; //Arm

    private double spinSpeed = 0;

    private DutyCycleEncoder t_Encoder;


    //private ArmFeedforward tFeedforward;

    private double tPV; //curr position
    private double tSetPoint; //destination we want to go to
    private PIDController armPid;
    double ArmOutput;

    //POSE PARAMETERS
    double MIN;
    double toHome;
    double toTrap;
    double toAmp;
    double toAim; //Arbitrary value based on distance, shoots
    double M_UP;
    double M_DOWN;
    double MAX;

    double IDLE;

    boolean OVERRIDE = false;



    
    // private SparkPIDController setPID(CANSparkMax m_motor) {
    //     // SparkPIDController m_pidController = m_motor.getPIDController();

    //     m_pidController.setP(0.1);
    //     m_pidController.setI(1e-5);
    //     m_pidController.setD(0.1);

    //     m_pidController.setOutputRange(0.1, 0.1);

    //     return m_pidController;
    // }


    public TrapAmpSubsystem(){

        m_trapMotor = new PWMSparkMax(frc.robot.Constants.AmpSystem.trapScorerID);
        m_RightArmMotor = new CANSparkMax(frc.robot.Constants.AmpSystem.RightAmArmID, MotorType.kBrushless);
        m_LeftArmMotor = new CANSparkMax(frc.robot.Constants.AmpSystem.LeftAmpArmID, MotorType.kBrushless);        //m_trapMotor.setIdleMode(IdleMode.kBrake);
        m_RightArmMotor.setIdleMode(IdleMode.kCoast);
        m_LeftArmMotor.setIdleMode(IdleMode.kCoast);

        tState = frc.robot.State.tState.STOP;

        t_Encoder = new DutyCycleEncoder(frc.robot.Constants.AmpSystem.ampEncoderID); //PWM Channel
        
        double ffP = 0.008; //was 0.01 //TODO: Tune PID
        double ffI = 0.0;
        double ffD = 0.0;

        armPid = new PIDController(ffP, ffI, ffD);

        //ARM SETPOINTS
        MIN = 10;
        toHome = 174; //TODO: calibrate Trap ARM Setpoints
        toTrap = 0; 
        toAim = 23.1; 
        //toAmp = 23.1;
        M_UP = tPos() + 1; //increment by 1 
        M_DOWN = tPos() - 1;
        IDLE = tPos();
        MAX = 180; //178


        
        m_RightArmMotor.setInverted(false); //was true
        m_LeftArmMotor.setInverted(true); //was false

        

        goTrapWheelState(frc.robot.State.tState.STOP);
        armPid.setSetpoint(toHome);

        
        //FAILSAFE


        //tPID_R.setReference(0.5, CANSparkBase.ControlType.kPosition);
    }

    private double tPos() {
        return t_Encoder.getAbsolutePosition() * 360;
    }

     @Override
    public void periodic(){
        m_trapMotor.set(spinSpeed);

        //ARM
        tPV = tPos();

        // // //if under
        //  if(tPV < MIN)
        //  {
        //      ArmOutput = 0.1; 
        //  }
        //  //if over
        //  if(tPV > MAX){
        //      ArmOutput = -0.1;
        //  }
        
        ArmOutput = armPid.calculate(tPV);



        
        


        if(OVERRIDE == true){
            
            switch(eState) {
                case IDLE:
                    m_LeftArmMotor.set(0.01);
                    m_RightArmMotor.set(0.01);
                break;
                case M_DOWN:
                    m_LeftArmMotor.set(-0.1);
                    m_RightArmMotor.set(-0.1);
                break;
                case M_UP:
                    m_LeftArmMotor.set(0.1);
                    m_RightArmMotor.set(0.1);
                break;
                default:
                break;
                
            }

        } else {
            //if within threshold
            if(tPV > MIN && tPV < MAX){
            m_LeftArmMotor.set(ArmOutput);
            m_RightArmMotor.set(ArmOutput); 
            } else {
            m_LeftArmMotor.set(0);
            m_RightArmMotor.set(0);
            }
        }
        

        // if(!OVERRIDE){ //Normal
        // m_LeftArmMotor.set(speed);
        // m_RightArmMotor.set(speed); 
        // }else{ //Override
        //     if(eState == frc.robot.State.eState.M_DOWN){
        //          m_LeftArmMotor.set(-0.1);
        //         m_RightArmMotor.set(-0.1);
        //     }
        //     if(eState == frc.robot.State.eState.M_UP){
        //          m_LeftArmMotor.set(0.1);
        //         m_RightArmMotor.set(0.1);
        //     }
    
        
        
        

        //temp comment
        //m_LeftArmMotor.set(speed);
        // //if(tPV > MIN && tPV <= MAX){
        //     //postive power goes up 
        //m_LeftArmMotor.set(tOutput);
        // m_LeftArmMotor.setVoltage(3);
        // m_RightArmMotor.setVoltage(3);
        
        // //}else{ //Failsafe
        //     // m_LeftArmMotor.set(0);
        //     // m_RightArmMotor.set(0);
        //     // m_LeftArmMotor.disable();
        //     // m_RightArmMotor.disable();
        // //}
        
        SmartDashboard.putNumber("Trap Encoder DIO#", t_Encoder.getSourceChannel());
        SmartDashboard.putNumber("T Setpoint", tSetPoint);
        SmartDashboard.putNumber("T encoder", tPos());
        SmartDashboard.putNumber("motor value", ArmOutput);
        SmartDashboard.putBoolean("Trap isOverride", OVERRIDE);
        

    }


    //TRAP AMP Spinner
    public void goTrapWheelState(tState state){

        switch(state) {
            case IN:
                spinSpeed = 1;
                tState = frc.robot.State.tState.IN;
            break;
            case OUT:
                spinSpeed = -1;
                tState = frc.robot.State.tState.OUT;
            break;
            case STOP:
                spinSpeed = 0;
                tState = frc.robot.State.tState.STOP;
            break;
            default:
            break;
        }

    }

    //ARM SET SETPOINT
    public void setTSetPoint(double setpoint){
        tSetPoint = setpoint;
    }

    //ARM GET SETPOINT
    public double getTSetPoint(){
        return tSetPoint;
    }


    //Arm
    public void goTrapArmState(eState state){

        switch(state) {

            case HOME:
                OVERRIDE = false;
                setTSetPoint(toHome);
                eState = frc.robot.State.eState.HOME;
            break;
            case TRAP_POS:
                OVERRIDE = false;
                setTSetPoint(toTrap);
                eState = frc.robot.State.eState.TRAP_POS;
            break;
            case AIM_POS:
                OVERRIDE = false;
                setTSetPoint(toAim);
                eState = frc.robot.State.eState.AIM_POS;
            break;
            case M_UP:
                eState = frc.robot.State.eState.M_UP;
                OVERRIDE = true;
            break;
            case M_DOWN:
                eState = frc.robot.State.eState.M_DOWN;
                OVERRIDE = true;
            break;
            case M_IDLE:
                eState = frc.robot.State.eState.M_IDLE;
                OVERRIDE = true;
            break;
            default:
            break;

        }

        if(!OVERRIDE)
        {
           armPid.setSetpoint(tSetPoint);
        }

    }


}

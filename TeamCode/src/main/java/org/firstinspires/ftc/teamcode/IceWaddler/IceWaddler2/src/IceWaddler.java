package org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src;

import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.IWConfig.*;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Hardware.IWLocalizer;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

import java.util.List;

public class IceWaddler {
    // Hardware
    IWDriveTrain driveTrain;
    IWLocalizer localizer;

    // Timers
    Scalar tickTime;
    ElapsedTime runTimer;
    Scalar runTime;

    // Situations
    Situation lastSituation;    //Situation during the last tick, used to interpolate accelerations and velocities, if needed
    Situation currentSituation; //Current situation, from odometry
    Situation targetSituation;  //Target situation to drive motors. Position is null
    // Note: All situation objects should be field centric

    // Velocity -> Acceleration PID Controllers
    PIDController vController;
    PIDController vAngController;

    // Position -> Velocity PID Controllers
    PIDController pController;
    PIDController pAngController;

    // Control Settings
    public enum CONTROLMODE {
        ACCELERATION,
        VELOCITY,
        PATH,
        STBY
    }
    public CONTROLMODE controlMode; //Controls which control layers to trigger during the loop
    public boolean fieldCentric;

    /// Creates a new "waddler", and defines the hardware to interface
    /// @param driveTrain An implementation of IWDriveTrain
    /// @param localizer An implementation of a IWLocalizer
    public IceWaddler(IWDriveTrain driveTrain, IWLocalizer localizer){
        this.driveTrain=driveTrain;
        this.localizer=localizer;
    }

    ///Self-explanatory, inits the waddler
    ///Sets initial values, including controlmode, pose, and fieldcentricity ig
    ///Also querries config for PID controllers, and inits timers
    public void init(CONTROLMODE initMode, Position initPos, boolean fieldCentric){
        controlMode=initMode;
        this.fieldCentric=fieldCentric;

        //Init timer
        runTimer=new ElapsedTime();
        runTime=new Scalar(runTimer.nanoseconds(), ns);

        //Init hardware
        driveTrain.init();
        localizer.init();
        localizer.reset(new Situation(
                Acceleration.zero,
                Velocity.zero,
                initPos
        ));
        localizer.update();
        lastSituation=localizer.getSituation();// To avoid not defined errors in later derivatives

        //Define PID controllers
        vController=fromCoeffs(vControllerCoeff);
        vAngController=fromCoeffs(vAngControllerCoeff);

        pController=fromCoeffs(pControllerCoeff);
        pAngController=fromCoeffs(pAngControllerCoeff);

        targetSituation=new Situation(null,null,null);
    }

    public void resetOdo(Position resetPos){
        localizer.reset(new Situation(Acceleration.zero,
                Velocity.zero,
                resetPos));
    }

    ///Updates tickTime variable
    private void updateTimer(){
        Scalar lastRunTime=runTime;
        runTime=new Scalar(runTimer.nanoseconds(), ns);
        tickTime=runTime.sub(lastRunTime);
    }

    ///Updates odometry, and computes derivatives if needed
    private void updateOdo(){
        localizer.update();
        lastSituation=currentSituation;
        currentSituation=localizer.getSituation();

        //Derivatives first, derivatives avoid cumulative errors
        //Velocity as the derivative of position
        if(currentSituation.getVelocity()==null&&currentSituation.getPosition()!=null){
            currentSituation.setVelocity(currentSituation.getPosition().sub(lastSituation.getPosition()).differentiate(tickTime));
        }
        //Acceleration as the derivative of velocity
        if(currentSituation.getAcceleration()==null&&currentSituation.getVelocity()!=null){
            currentSituation.setAcceleration(currentSituation.getVelocity().sub(lastSituation.getVelocity()).differentiate(tickTime));
        }

        //Integrals
        //Velocity as the integral of acceleration
        if(currentSituation.getVelocity()==null&&currentSituation.getAcceleration()!=null){
            currentSituation.setVelocity(lastSituation.getVelocity().add(currentSituation.getAcceleration().integrate(tickTime)));
        }
        //Position as the integral of velocity
        if(currentSituation.getPosition()==null&&currentSituation.getVelocity()!=null){
            currentSituation.setPosition(lastSituation.getPosition().add(currentSituation.getVelocity().integrate(tickTime)));
        }
    }

    public void setFieldCentric(boolean fieldCentric){
        this.fieldCentric = fieldCentric;
    }

    public void toggleFieldCentric(){
        fieldCentric = !fieldCentric;
    }

    public Situation getCurrentSituation(){
        return currentSituation;
    }

    // Power methods
    ///Sets all motors to 0 power, and places waddler on standby
    public void zeroPower(){
        controlMode=CONTROLMODE.STBY;
        driveTrain.writePowers(0,0,0,0);
    }

    ///Used in pathing to safely set zero power without changing controlMode
    private void pathZeroPower(){
        driveTrain.writePowers(0,0,0,0);
    }

    //Acceleration Methods
    ///Tell waddler to run at a target acceleration<br>
    /// It's not recommended to work with this method externally, as an acceleration of 0 will cause the robot to continue moving at a constant velocity
    public void runByAccel(Acceleration target){
        controlMode=CONTROLMODE.ACCELERATION;
        if(fieldCentric) {
            targetSituation.setAcceleration(target);
        }else{
            targetSituation.setAcceleration(target.rotateBy(currentSituation.getPosition().getHeading()));
        }
        limitAcceleration();
    }

    ///Writes power to drivetrain based on acceleration
    private void writeAccel(){
        Acceleration robotCentricAcc=targetSituation.getAcceleration().rotateBy(currentSituation.getPosition().getHeading().multiply(-1));
        Scalar strafe=robotCentricAcc.getX();
        Scalar forward=robotCentricAcc.getY();
        Scalar rot=robotCentricAcc.getAngAcc().multiply(wheelPivotRadius).div(new Scalar(1,rad)); //Find linear acceleration needed to reach required angular acceleration
        List<Double> motorVels=driveTrain.getVelocities();
        driveTrain.writePowers(
                calculatePower(forward.add(strafe).add(rot),motorVels.get(0)),
                calculatePower(forward.sub(strafe).add(rot),motorVels.get(1)),
                calculatePower(forward.sub(strafe).sub(rot),motorVels.get(2)),
                calculatePower(forward.add(strafe).sub(rot),motorVels.get(3))
        );
    }

    /// Tells waddler to maintain an acceleration of zero<br>
    /// It's not recommended to work with this method externally, as an acceleration of 0 will cause the robot to continue moving at a constant velocity
    public void zeroAccel(){
        runByAccel(Acceleration.zero);
    }


    ///Tell waddler to run at a target acceleration<br>
    /// recommended method for driver control
    public void runByVel(Velocity target){
        controlMode=CONTROLMODE.VELOCITY;
        if(fieldCentric){
            targetSituation.setVelocity(target);
        }else{
            targetSituation.setVelocity(target.rotateBy(currentSituation.getPosition().getHeading()));
        }
    }

    private void writeVel(){
        Velocity current=currentSituation.getVelocity();
        Velocity target=currentSituation.getVelocity();
        targetSituation.setAcceleration(new Acceleration(
                new Vector(
                        vController.calculate(current.getX().getValueSI(),target.getX().getValueSI()),
                        vController.calculate(current.getY().getValueSI(),target.getY().getValueSI()),
                        metersPerSecondSquared),
                new Scalar(vAngController.calculate(current.getAngVel().getValueSI(),target.getAngVel().getValueSI()),radiansPerSecondSquared)
        ));
        limitAcceleration();
        writeAccel();
    }

    /// Run with target velocity of zero.
    /// Sets controlMode to Velocity, and must be looped to work
    /// Do not use in pathing
    public void brake(){
        runByVel(new Velocity(new Vector(0,0,metersPerSecond), new Scalar(0, radiansPerSecond)));
    }

    public void loop(){
        updateTimer();
        updateOdo();

        switch(controlMode){
            case STBY:
                break;

            case ACCELERATION:
                writeAccel();
                break;

            case VELOCITY:
                writeVel();
                break;
        }
    }

    //Helper methods
    private PIDController fromCoeffs(PIDCoefficients Coeffs){
        return new PIDController(Coeffs.p, Coeffs.i, Coeffs.d);
    }

    private double calculatePower(Scalar acceleration, double currentVel){
        //Will implement based on fit later, currently write acceleration to motors directly
        return acceleration.div(maxAccel).getValueSI();
    }

    ///limits acceleration within the bounds specified in the config
    private void limitAcceleration(){
        Acceleration acceleration=targetSituation.getAcceleration();
        targetSituation.setAcceleration(new Acceleration(
                acceleration.getLinAcc().mag().lessThanOrEqual(maxAccel)?acceleration.getLinAcc():acceleration.getLinAcc().multiply(maxAccel.div(acceleration.getLinAcc().mag())),
                acceleration.getAngAcc().abs().lessThanOrEqual(maxAngAccel)?acceleration.getAngAcc():acceleration.getAngAcc().multiply(maxAngAccel.div(acceleration.getAngAcc().abs()))
        ));//Ternary operator is used instead of min to prevent divisions by zero
    }
}
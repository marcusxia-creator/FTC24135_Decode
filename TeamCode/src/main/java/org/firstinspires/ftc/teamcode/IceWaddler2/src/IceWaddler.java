package org.firstinspires.ftc.teamcode.IceWaddler2.src;

import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.IWDriveTrain;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.IWLocalizer;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Acceleration;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Position;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Situation;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.Velocity;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Vector;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

import java.util.List;
import java.util.function.Supplier;

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
    public void init(Position initPos, boolean fieldCentric){
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
        lastSituation=currentSituation;
        localizer.update();
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

    ///Runs updates on odo and ticktime. Needs to be run every loop
    public void update(){
        updateTimer();
        updateOdo();
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
    ///Sets all motors to 0 power
    public void zeroPower(){
        driveTrain.writePowers(0,0,0,0);
    }

    ///Directly write powers into motors, used for tuning
    class PowerDrive implements Action{
        Supplier<Double> FL_Power;
        Supplier<Double> BL_Power;
        Supplier<Double> FR_Power;
        Supplier<Double> BR_Power;

        public PowerDrive(Supplier<Double> FL_Power, Supplier<Double> BL_Power, Supplier<Double> FR_Power, Supplier<Double> BR_Power){
            this.FL_Power=FL_Power;
            this.FR_Power=FR_Power;
            this.BL_Power=BL_Power;
            this.BR_Power=BR_Power;
        }

        @Override
        public void init() {}

        @Override
        public void loop() {
            update();
            driveTrain.writePowers(
                    FL_Power.get(),
                    FR_Power.get(),
                    BL_Power.get(),
                    BR_Power.get()
            );
        }

        @Override
        public void shutdown() {
            zeroPower();
        }

        @Override
        public boolean finished() {
            return false;
        }
    }

    /// Writes power to drivetrain based on target acceleration in targetSituation
    private void writeAccel(){
        limitAcceleration();

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
        targetSituation.setAcceleration(Acceleration.zero);
        writeAccel();
    }

    /// Runs by target acceleration, not recommended for use anywhere
    class AccelDrive implements Action{
        Supplier<Acceleration> targetAcceleration;
        public AccelDrive(Supplier<Acceleration> targetAcceleration){
            this.targetAcceleration=targetAcceleration;
        }

        @Override
        public void init() {}

        @Override
        public void loop() {
            update();
            if(fieldCentric) {
                targetSituation.setAcceleration(targetAcceleration.get());
            }else{
                targetSituation.setAcceleration(targetAcceleration.get().rotateBy(currentSituation.getPosition().getHeading()));
            }
            writeAccel();
        }

        @Override
        public void shutdown() {
            zeroPower();
        }

        @Override
        public boolean finished() {
            return false;
        }
    }

    ///Calculates target acceleration based on a target velocity
    private void writeVel(){

        Velocity current=currentSituation.getVelocity();
        Velocity target=targetSituation.getVelocity();
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
    public void zeroVel(){
        targetSituation.setVelocity(Velocity.zero);
        writeVel();
    }

    /// Runs by target velocity, recommended for TeleOp driver control
    class VelDrive implements Action{
        Supplier<Velocity> targetVelSupply;
        public VelDrive(Supplier<Velocity> targetVelSupply){
            this.targetVelSupply=targetVelSupply;
        }

        @Override
        public void init() {}

        @Override
        public void loop() {
            update();
            if(fieldCentric) {
                targetSituation.setVelocity(targetVelSupply.get());
            }else{
                targetSituation.setVelocity(targetVelSupply.get().rotateBy(currentSituation.getPosition().getHeading()));
            }
            writeVel();
        }

        @Override
        public void shutdown() {
            zeroPower();
        }

        @Override
        public boolean finished() {
            return false;
        }
    }

    class Brake implements Action{
        public Brake(){
        }

        @Override
        public void init() {}

        @Override
        public void loop() {
            update();
            zeroVel();
        }

        @Override
        public void shutdown() {
            zeroPower();
        }

        @Override
        public boolean finished() {
            return false;
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
                acceleration.getLinAcc().mag().lessThanOrEqual(maxAccel)?acceleration.getLinAcc():acceleration.unitVector().multi(maxAccel),
                acceleration.getAngAcc().abs().lessThanOrEqual(maxAngAccel)?acceleration.getAngAcc():acceleration.getAngAcc().multiply(maxAngAccel.div(acceleration.getAngAcc().abs()))
        ));//Ternary operator is used instead of min to prevent divisions by zero
    }
}
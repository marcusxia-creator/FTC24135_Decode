package org.firstinspires.ftc.teamcode.IceWaddler2.src;

import static org.apache.commons.math3.util.FastMath.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.IWConfig.*;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Action;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Hardware.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

import org.firstinspires.ftc.teamcode.IceWaddler2.src.Pathing.*;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Supplier;

public class IceWaddler {
    // Hardware
    IWDriveTrain driveTrain;
    IWLocalizer localizer;

    // Timers
    Scalar tickTime;
    ElapsedTime tickTimer;

    // Situations
    Queue<Situation> lastSituations=new LinkedList<>();    //Situation during the last few ticks, used to interpolate accelerations and velocities, if needed
    Situation currentSituation; //Current situation, from odometry
    Situation lastTargetSituation; //Used for derivatives in controllers
    Situation targetSituation;  //Target situation to drive motors. Position is null
    PathingPoint targetPathingPoint; //Used to transfer information while transitioning between pathing segments
    // Note: All situation objects should be field centric

    public boolean fieldCentric;

    // Storage variables for action querying
    Movement currentAction;

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
        tickTimer=new ElapsedTime();

        //Init hardware
        driveTrain.init();

        localizer.init();
        localizer.reset(new Situation(
                Acceleration.zero,
                Velocity.zero,
                initPos
        ));
        localizer.update();
        currentSituation=localizer.getSituation();
        lastSituations.offer(currentSituation);// To avoid not defined errors in later derivatives

        targetSituation=new Situation(null,null,null);
        lastTargetSituation=currentSituation;
    }

    public void resetOdo(Position resetPos){
        localizer.reset(new Situation(Acceleration.zero,
                Velocity.zero,
                resetPos));
        currentSituation.setPosition(resetPos);
    }

    ///Updates tickTime variable
    private void updateTimer(){
        tickTime=new Scalar(tickTimer.nanoseconds(), ns);
        tickTimer.reset();
    }

    public Scalar getTickTime(){
        return tickTime;
    }

    ///Updates odometry, and computes derivatives if needed
    private void updateOdo(){
        Situation lastSituation;

        lastSituations.offer(currentSituation);
        if(lastSituations.size()>=derivativeTicks){
            lastSituation=lastSituations.poll();
        }
        else{
            lastSituation=lastSituations.peek();
        }
        localizer.update();
        currentSituation=localizer.getSituation();

        //Derivatives first, derivatives avoid cumulative errors
        //Velocity as the derivative of position
        if(currentSituation.getVelocity()==null&&currentSituation.getPosition()!=null){
            currentSituation.setVelocity(currentSituation.getPosition().sub(lastSituation.getPosition()).differentiate(tickTime.multiply(lastSituations.size()+1)));
        }
        //Acceleration as the derivative of velocity
        if(currentSituation.getAcceleration()==null&&currentSituation.getVelocity()!=null){
            currentSituation.setAcceleration(currentSituation.getVelocity().sub(lastSituation.getVelocity()).differentiate(tickTime.multiply(lastSituations.size()+1)));
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

    ///Runs updates on odo and ticktime. Needs to be run every loop, preferably before any other methods
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

    public Situation getTargetSituation(){
        return targetSituation;
    }

    public Movement getCurrentAction(){return currentAction;}

    // Power methods
    ///Sets all motors to 0 power
    public void zeroPower(){
        driveTrain.runPowers(0,0,0,0);
    }

    ///Directly write powers into motors, used for tuning
    public class PowerDrive implements Action{
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

        public PowerDrive(Supplier<Double> powerSupply){
            this.FL_Power=powerSupply;
            this.FR_Power=powerSupply;
            this.BL_Power=powerSupply;
            this.BR_Power=powerSupply;
        }

        @Override
        public void loop() {
            update();
            driveTrain.runPowers(
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
    }

    /// Writes power to drivetrain based on target acceleration in targetSituation
    private void writeAccel(){
        limitAcceleration();

        Acceleration robotCentricAcc=targetSituation.getAcceleration().rotateBy(currentSituation.getPosition().getHeading().multiply(-1));
        Scalar strafe=robotCentricAcc.getX();
        Scalar forward=robotCentricAcc.getY();
        Scalar rot=robotCentricAcc.getAngAcc().multiply(wheelPivotRadius).div(new Scalar(1,rad)); //Find linear acceleration needed to reach required angular acceleration
        driveTrain.runAccel(
                (forward).add(strafe).add(rot),
                (forward).sub(strafe).add(rot),
                (forward).sub(strafe).sub(rot),
                (forward).add(strafe).sub(rot)
        );
    }

    /// Tells waddler to maintain an acceleration of zero<br>
    /// It's not recommended to work with this method externally, as an acceleration of 0 will cause the robot to continue moving at a constant velocity
    public void zeroAccel(){
        targetSituation.setAcceleration(Acceleration.zero);
        writeAccel();
    }

    /// Runs by target acceleration, not recommended for use anywhere
    public class AccelDrive implements Action{
        Supplier<Acceleration> targetAcceleration;
        public AccelDrive(Supplier<Acceleration> targetAcceleration){
            this.targetAcceleration=targetAcceleration;
        }

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
    }

    ///Calculates target acceleration based on a target velocity
    private void writeVel(){

        Velocity current=currentSituation.getVelocity();
        Velocity lastTarget=lastTargetSituation.getVelocity();
        Velocity target=targetSituation.getVelocity();
        targetSituation.setAcceleration(target.sub(lastTarget).differentiate(tickTime).add(accelerationController.getCorrection(target.sub(current))));
        limitAcceleration();
        lastTargetSituation.setVelocity(current);
        writeAccel();
    }

    /// Run with target velocity of zero.
    /// Sets controlMode to Velocity, and must be looped to work
    public void zeroVel(){
        targetSituation.setVelocity(Velocity.zero);
        writeVel();
    }

    /// Runs by target velocity, recommended for TeleOp driver control
    public class VelDrive implements Action{
        Supplier<Velocity> targetVelSupply;
        public VelDrive(Supplier<Velocity> targetVelSupply){
            this.targetVelSupply=targetVelSupply;
        }

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
    }

    ///Holds Velocity at Zero
    public class Brake implements Action{
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

    /// Motion Action
    public class MotionAction implements Action{
        Movement movement;

        public MotionAction(Movement movement){
            this.movement=movement;
        }

        @Override
        public void init() {
            movement.init(targetPathingPoint);
            targetSituation.setPosition(targetPathingPoint.getPosition());
            targetPathingPoint=movement.getTargetPoint();
            currentAction=movement;
        }

        @Override
        public void loop(){
            update();
            movement.loop(currentSituation,tickTime);
            targetSituation.setVelocity(movement.getTargetVel());
            writeVel();
        }

        @Override
        public void shutdown(){
            zeroPower();
            currentAction=null;
            targetPathingPoint=movement.getTargetPoint();
        }

        @Override
        public boolean finished() {
            return movement.finished();
        }
    }

    public TelemetryPacket drawField(){
        double x=currentSituation.getPosition().getX().getValue(in);
        double y=currentSituation.getPosition().getY().getValue(in);
        double h=currentSituation.getPosition().getHeading().getValue(rad);
        TelemetryPacket packet= new TelemetryPacket(true);
        packet.fieldOverlay()
                .setAlpha(0.25)
                .setFill("white")
                .fillPolygon(   new double[]{x, x+9*sin(h), x+sqrt(162)*sin(h+PI/4),    x+sqrt(162)*sin(h+3*PI/4),  x+sqrt(162)*sin(h-3*PI/4),  x+sqrt(162)*sin(h-PI/4),    x+9*sin(h)},
                        new double[]{y, y+9*cos(h), y+sqrt(162)*cos(h+PI/4),    y+sqrt(162)*cos(h+3*PI/4),  y+sqrt(162)*cos(h-3*PI/4),  y+sqrt(162)*cos(h-PI/4),    y+9*cos(h)})
                .setAlpha(1)
                .setStroke("white")
                .setStrokeWidth(2)
                .strokePolygon( new double[]{x, x+9*sin(h), x+sqrt(162)*sin(h+PI/4),    x+sqrt(162)*sin(h+3*PI/4),  x+sqrt(162)*sin(h-3*PI/4),  x+sqrt(162)*sin(h-PI/4),    x+9*sin(h)},
                        new double[]{y, y+9*cos(h), y+sqrt(162)*cos(h+PI/4),    y+sqrt(162)*cos(h+3*PI/4),  y+sqrt(162)*cos(h-3*PI/4),  y+sqrt(162)*cos(h-PI/4),    y+9*cos(h)})
                .setStroke("red")
                .strokeLine(x,y,x+6*currentSituation.getVelocity().getX().getValueSI(),y+6*currentSituation.getVelocity().getY().getValueSI())
                .setStroke("blue")
                .strokeLine(x,y,x+20*currentSituation.getAcceleration().getX().getValueSI(),y+20*currentSituation.getAcceleration().getY().getValueSI())
                .setStroke("green")
                .setStrokeWidth(1);
        return packet;
    }


    //Helper methods
    private PIDController fromCoeffs(PIDCoefficients Coeffs){
        return new PIDController(Coeffs.p, Coeffs.i, Coeffs.d);
    }

    ///limits acceleration within the bounds specified in the config
    private void limitAcceleration(){
        Acceleration acceleration=targetSituation.getAcceleration();
        /*targetSituation.setAcceleration(new Acceleration(
                acceleration.getLinAcc().mag().lessThanOrEqual(maxAccel)?acceleration.getLinAcc():acceleration.unitVector().multi(maxAccel),
                acceleration.getAngAcc().abs().lessThanOrEqual(maxAngAccel)?acceleration.getAngAcc():acceleration.getAngAcc().multiply(maxAngAccel.div(acceleration.getAngAcc().abs()))
        ));*///Ternary operator is used instead of min to prevent divisions by zero
    }
}
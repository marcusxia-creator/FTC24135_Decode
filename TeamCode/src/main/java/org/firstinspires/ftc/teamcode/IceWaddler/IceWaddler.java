package org.firstinspires.ftc.teamcode.IceWaddler;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.drive.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

import java.util.List;

public class IceWaddler {
    final RobotHardware robot;

    //Hardware Definition
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    public GoBildaPinpointDriver odo;

    //global odo data
    public Pose2D currentPos;
    public Pose2D currentVel;
    public double currentRotVel;

    //Power Pose2D object
    public Pose2D targetPower;
    public double targetRotPower;

    //Velocity Pose2D object
    public Pose2D targetVel;
    public double targetRotVel;

    //Velocity to Power PID Controllers
    public PIDController vController;
    public PIDController vRotController;

    //Position to Power PID Controllers
    public PIDController pController;
    public PIDController pRotController;

    //Position Pose2D object
    public Pose2D startingPos;
    public Pose2D targetPos;
    public boolean decelerate;

    //Publicly referencable variables for actions
    public double distanceTraveled;
    public double distanceRemaining;

    //Debug public variables
    public double latCorrection;
    public double lonCorrection;
    public double rotCorrection;
    public double lineAngle;
    
    //Path Variables
    public Action currentAction;
    public int currentActionIndex;
    public List<Action> path;
    public double actionCompletion; //Also used by Position
    public boolean actionCompleted;
    public ElapsedTime delayTimer;

    public enum CONTROLMODE {
        POWER,
        VELOCITY,
        POSITION,
        PATH,
        STBY
    }

    public boolean fieldCentric=false;

    public CONTROLMODE controlMode;

    public IceWaddler(RobotHardware robot){
        this.robot=robot;
    }

    public void Init(CONTROLMODE InitMode, Pose2D initPose, boolean fieldCentric){
        controlMode = InitMode;
        this.fieldCentric = fieldCentric;

        frontLeftMotor  = robot.frontLeftMotor;
        backLeftMotor   = robot.backLeftMotor;
        frontRightMotor = robot.frontRightMotor;
        backRightMotor  = robot.backRightMotor;

        vController = fromCoeffs(IceWaddlerConfig.vController);
        vRotController = fromCoeffs(IceWaddlerConfig.vRotController);

        pController = fromCoeffs(IceWaddlerConfig.pController);
        pRotController = fromCoeffs(IceWaddlerConfig.pRotController);

        InitOdo(initPose);
        updateOdo();

        targetPos = initPose;
    }

    public void InitOdo(Pose2D initPose){
        odo = robot.odo;

        odo.setOffsets(IceWaddlerConfig.odoXOffset, IceWaddlerConfig.odoYOffset); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(IceWaddlerConfig.odoEncoderResolution);
        odo.setEncoderDirections(IceWaddlerConfig.xEncoderDirection, IceWaddlerConfig.yEncoderDirection);
        //Set to start counting at initPose parameter
        odo.resetPosAndIMU();
        odo.setPosition(initPose);

        updateOdo();
    }

    public void updateOdo(){
        odo.update();

        currentPos = odo.getPosition();
        currentVel = odo.getVelocity();
        currentRotVel = odo.getHeadingVelocity();
    }

    public void setFieldCentric(boolean v){
        fieldCentric = v;
    }

    public void toggleFieldCentric(){
        fieldCentric = !fieldCentric;
    }

    public void runByPower(Pose2D targetPower, double Rot){
        controlMode = CONTROLMODE.POWER;
        this.targetPower = targetPower;
        targetRotPower = Rot;
    }

    private void writePower(){

        Pose2D robotCentricPower = rotatePose(targetPower, AngleUnit.RADIANS, currentPos.getHeading(AngleUnit.RADIANS));

        double x = robotCentricPower.getX(DistanceUnit.METER);
        double y = robotCentricPower.getY(DistanceUnit.METER);
        double rot = targetRotPower;

        //Write to Mecanum drive

        frontLeftMotor.setPower(x+y+rot);
        backLeftMotor.setPower(x-y+rot); 
        frontRightMotor.setPower(x-y-rot);
        backRightMotor.setPower(x+y-rot);
    }

    public void zeroPower(){
        targetPower = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
        targetRotPower = 0;
        writePower();
    }

    public void runByVel(Pose2D targetVel, double Rot){
        controlMode = CONTROLMODE.VELOCITY;
        this.targetVel = targetVel;
        targetRotVel = Rot;
    }

    private void writeVel() {

        targetRotPower = -vRotController.calculate(currentRotVel, targetRotVel);

        targetPower = new Pose2D(
                DistanceUnit.METER,
                vController.calculate(currentVel.getX(DistanceUnit.METER), targetVel.getX(DistanceUnit.METER)),
                vController.calculate(-currentVel.getY(DistanceUnit.METER), targetVel.getY(DistanceUnit.METER)),
                AngleUnit.RADIANS,
                0);
        writePower();
    }

    public void brake(){
        targetRotVel = 0;
        targetVel = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
        writeVel();
    }

    public void runByPos(Pose2D startingPos, Pose2D targetPos, boolean decelerate){
        controlMode = CONTROLMODE.POSITION;
        this.startingPos=startingPos;
        this.targetPos=targetPos;
        this.decelerate=decelerate;
    }

    private void writePos(){
        //Line Constants
        double A = startingPos.getY(DistanceUnit.METER)-targetPos.getY(DistanceUnit.METER);
        double B = targetPos.getX(DistanceUnit.METER)-startingPos.getX(DistanceUnit.METER);
        double C = startingPos.getX(DistanceUnit.METER)*targetPos.getY(DistanceUnit.METER)-targetPos.getX(DistanceUnit.METER)*startingPos.getY(DistanceUnit.METER);
        //Target line can be graphed as Ax+By+C=0
        //Lateral PID correction
        double latDistance = (A*currentPos.getX(DistanceUnit.METER)+B*currentPos.getY(DistanceUnit.METER)+C)/
                Math.sqrt(Math.pow(A,2)+Math.pow(B,2)); //From Desmos graph https://www.desmos.com/calculator/uw6fymsdjv
        latCorrection = -pController.calculate(latDistance);

        //Action triggers
        distanceTraveled = Math.sqrt(Math.pow(distanceBetween(startingPos, currentPos, DistanceUnit.METER),2)-Math.pow(latDistance,2));
        distanceRemaining = Math.sqrt(Math.pow(distanceBetween(currentPos, targetPos, DistanceUnit.METER),2)-Math.pow(latDistance,2));
        double totalDistance = distanceBetween(startingPos, targetPos, DistanceUnit.METER);
        actionCompletion = distanceTraveled/totalDistance;

        //Deceleration if enabled
        if(decelerate) {
            lonCorrection = Range.clip(Math.sqrt(Math.pow(IceWaddlerConfig.minSpeed, 2) + 2 * IceWaddlerConfig.maxDecel * distanceRemaining),
                    IceWaddlerConfig.minSpeed, IceWaddlerConfig.maxSpeed); //From Desmos graph https://www.desmos.com/calculator/e7plnhpxva
        }
        else {
            lonCorrection = IceWaddlerConfig.maxSpeed;
        }
        //PID will handle acceleration

        //Rotation control, changes linearly over distance
        double modOffset = 0; //To minimize required movement, see Desmos graph https://www.desmos.com/calculator/zbjvqscngx
        if(startingPos.getHeading(AngleUnit.DEGREES)-targetPos.getHeading(AngleUnit.DEGREES)<-180){
            modOffset = 2*Math.PI;
        }
        else if(startingPos.getHeading(AngleUnit.DEGREES)-targetPos.getHeading(AngleUnit.DEGREES)>180){
            modOffset = -2*Math.PI;
        }

        double rotSetpoint = startingPos.getHeading(AngleUnit.RADIANS)+actionCompletion*(targetPos.getHeading(AngleUnit.RADIANS)-startingPos.getHeading(AngleUnit.RADIANS)+modOffset);

        rotCorrection = pRotController.calculate(((currentPos.getHeading(AngleUnit.RADIANS)-rotSetpoint+Math.PI)%(2*Math.PI))-Math.PI);

        Pose2D OrientedVel = new Pose2D(DistanceUnit.METER,lonCorrection , latCorrection, AngleUnit.RADIANS, 0);

        //Align movement to line
        lineAngle = -Math.atan2(targetPos.getY(DistanceUnit.METER)-startingPos.getY(DistanceUnit.METER),targetPos.getX(DistanceUnit.METER)-startingPos.getX(DistanceUnit.METER));

        targetVel = rotatePose(OrientedVel, AngleUnit.RADIANS, lineAngle);
        targetRotVel = rotCorrection;
        
        writeVel();
    }

    private void holdPos(){
        targetRotVel = pRotController.calculate(currentPos.getHeading(AngleUnit.RADIANS),targetPos.getHeading(AngleUnit.RADIANS));

        targetVel = new Pose2D(DistanceUnit.METER,
                Range.clip(-pController.calculate(currentPos.getX(DistanceUnit.METER),targetPos.getX(DistanceUnit.METER)),-IceWaddlerConfig.minSpeed,IceWaddlerConfig.minSpeed),
                Range.clip(-pController.calculate(currentPos.getY(DistanceUnit.METER),targetPos.getY(DistanceUnit.METER)),-IceWaddlerConfig.minSpeed,IceWaddlerConfig.minSpeed),
                AngleUnit.RADIANS, 0);

        writeVel();
    }
    
    public void runPath(List<Action> path){
        controlMode=CONTROLMODE.PATH;
        this.path=path;
        currentActionIndex=-1;
        actionCompletion=1;
        actionCompleted=true;
    }

    private void writePath(){
        //Switch to STBY if last action is completed
        if(actionCompleted && currentActionIndex==path.size()-1){
            controlMode=CONTROLMODE.STBY;
        }
        else {
            //Switch to next action if needed
            if (actionCompleted) {
                currentActionIndex++;
                currentAction = path.get(currentActionIndex);
                actionInit();
            }
            actionLoop();
        }
    }

    public void loop(){

        updateOdo();

        switch (controlMode){
            case POWER:
                //Apply robot centric with fresh odo data
                if(!fieldCentric){
                    targetPower=rotatePose(targetPower, AngleUnit.RADIANS, -currentPos.getHeading(AngleUnit.RADIANS));
                }
                writePower();
                break;

            case VELOCITY:
                //Apply robot centric with fresh odo data
                if(!fieldCentric){
                    targetVel=rotatePose(targetVel, AngleUnit.RADIANS, -currentPos.getHeading(AngleUnit.RADIANS));
                }
                writeVel();
                break;

            case POSITION:
                writePos();
                break;
                
            case PATH:
                writePath();
                break;

            case STBY:
                brake();
                break;
        }
    }

    //Helper Functions
    private Pose2D rotatePose(Pose2D pose, AngleUnit angleUnit, double angle){

        angle=AngleUnit.RADIANS.fromUnit(angleUnit,angle);

        return new Pose2D(
                DistanceUnit.METER,
                pose.getX(DistanceUnit.METER)*Math.cos(angle)-pose.getY(DistanceUnit.METER)*Math.sin(angle),
                pose.getX(DistanceUnit.METER)*Math.sin(angle)+pose.getY(DistanceUnit.METER)*Math.cos(angle),
                AngleUnit.RADIANS,
                pose.getHeading(AngleUnit.RADIANS)
        );
    }

    private double distanceBetween(Pose2D pose1, Pose2D pose2, DistanceUnit distanceUnit){
        return Math.sqrt(
                Math.pow(pose1.getX(distanceUnit)-pose2.getX(distanceUnit),2)+
                        Math.pow(pose1.getY(distanceUnit)-pose2.getY(distanceUnit),2)
        );
    }

    private PIDController fromCoeffs(PIDCoefficients Coeffs){
        return new PIDController(Coeffs.p, Coeffs.i, Coeffs.d);
    }


    //Action Managers
    private void actionInit(){
        actionCompletion = 0;
        actionCompleted = false;

        switch(currentAction.actionType){
            case RUN:
                startingPos = currentAction.startingPos;
                targetPos = currentAction.targetPos;
                decelerate = currentAction.decelerate;
                break;

            case HOLD:
                switch(currentAction.terminationtype) {
                    case NONE:
                        break;

                    case TIME:
                        delayTimer = new ElapsedTime();

                    case POS:
                        break;
                }
                break;
        }
    }

    private void actionLoop() {

        switch(currentAction.actionType){
            case RUN:
                targetVel = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
                writePos();
                if (distanceRemaining <= IceWaddlerConfig.tolerance || actionCompletion >= 1 || distanceBetween(startingPos,currentPos,DistanceUnit.METER)>distanceBetween(startingPos,targetPos,DistanceUnit.METER)) {
                    actionCompleted = true;
                    if (decelerate) {
                        zeroPower();
                    }
                }
                break;

            case HOLD:

                switch(currentAction.holdtype){
                    case POWER:
                        zeroPower();
                        break;

                    case VEL:
                        brake();
                        break;

                    case POS:
                        holdPos();
                        break;
                }

                switch(currentAction.terminationtype) {
                    case NONE:
                        break;

                    case TIME:
                        actionCompletion = delayTimer.seconds() / currentAction.delayLength;
                        if (actionCompletion >= 1) {
                            actionCompleted = true;
                        }
                        break;

                    case POS:
                        if (Math.abs(currentPos.getX(DistanceUnit.CM)-targetPos.getX(DistanceUnit.CM))<=currentAction.pTolorance &&
                                Math.abs(currentPos.getHeading(AngleUnit.DEGREES)-targetPos.getHeading(AngleUnit.DEGREES))<=currentAction.hTolorance){
                            actionCompleted = true;
                        }
                }

                break;
        }
    }

    public static class Action {
        //Common variables
        public ACTIONTYPE actionType;

        public enum ACTIONTYPE{
            RUN,
            HOLD
        }

        //Hold Variables
        public HOLDTYPE holdtype;

        public enum HOLDTYPE{
            POWER,
            VEL,
            POS
        }

        public TERMINATIONTYPE terminationtype;

        public enum TERMINATIONTYPE{
            NONE,
            TIME,
            POS
        }

        //Delay Termination Variables
        public double delayLength;

        //Position Termination Variables
        public double pTolorance;
        public double hTolorance;

        //Run variables
        public Pose2D startingPos;
        public Pose2D targetPos;
        public boolean decelerate;

        //Run Action
        public Action(Pose2D startingPos, Pose2D targetPos, boolean decelerate){
            this.actionType = ACTIONTYPE.RUN;
            this.startingPos = startingPos;
            this.targetPos = targetPos;
            this.decelerate = decelerate;
        }

        //No Termination
        public Action(HOLDTYPE holdtype) {
            this.actionType = ACTIONTYPE.HOLD;
            this.holdtype = holdtype;
            this.terminationtype= TERMINATIONTYPE.NONE;
        }

        //Time Termination
        public Action(HOLDTYPE holdtype, double delayLength) {
            this.actionType = ACTIONTYPE.HOLD;
            this.holdtype = holdtype;
            this.terminationtype = TERMINATIONTYPE.TIME;
            this.delayLength = delayLength;
        }

        //Position Termination; All units are CM and DEG
        public Action(double pTolorance, double hTolorance) {
            this.actionType = ACTIONTYPE.HOLD;
            this.holdtype = HOLDTYPE.POS;
            this.terminationtype = TERMINATIONTYPE.POS;
            this.pTolorance = pTolorance;
            this.hTolorance = hTolorance;
        }
    }
}
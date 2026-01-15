package org.firstinspires.ftc.teamcode.IceWaddler;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.METER;
import static org.firstinspires.ftc.teamcode.IceWaddler.IceWaddlerConfig.*;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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

    public IWLocalizer odo;

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
    public PIDController pLatController;
    public PIDController pRotController;

    //Position Pose2D object
    public Pose2D startingPos;
    public Pose2D targetPos;
    public boolean decelerate;
    public double tolorance=stopTolerance;

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

        vController = fromCoeffs(vControllerCoeff);
        vRotController = fromCoeffs(vRotControllerCoeff);

        pController = fromCoeffs(pControllerCoeff);

        pLatController = fromCoeffs(pLatControllerCoeff);
        pRotController = fromCoeffs(pRotControllerCoeff);

        odo=robot.IWodo;
        this.odo.init(initPose);
        updateOdo();

        targetPos = initPose;
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

        Pose2D robotCentricPower = rotatePose(targetPower, RADIANS, currentPos.getHeading(RADIANS));

        double x = robotCentricPower.getX(METER);
        double y = robotCentricPower.getY(METER);
        double rot = targetRotPower;
        double fac = Math.min(1/(Math.abs(rot)+Math.abs(x)+Math.abs(y)),1.5);

        //Write to Mecanum drive

        frontLeftMotor.setPower((x+y+rot)*fac);
        backLeftMotor.setPower((x-y+rot)*fac);
        frontRightMotor.setPower((x-y-rot)*fac);
        backRightMotor.setPower((x+y-rot)*fac);
    }

    public void zeroPower(){
        targetPower = new Pose2D(METER, 0, 0, AngleUnit.DEGREES, 0);
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
                METER,
                vController.calculate(currentVel.getX(METER), targetVel.getX(METER)),
                vController.calculate(-currentVel.getY(METER), targetVel.getY(METER)),
                RADIANS,
                0);
        writePower();
    }

    public void brake(){
        targetRotVel = 0;
        targetVel = new Pose2D(METER, 0, 0, AngleUnit.DEGREES, 0);
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
        double A = startingPos.getY(METER)-targetPos.getY(METER);
        double B = targetPos.getX(METER)-startingPos.getX(METER);
        double C = startingPos.getX(METER)*targetPos.getY(METER)-targetPos.getX(METER)*startingPos.getY(METER);
        //Target line can be graphed as Ax+By+C=0
        //Lateral PID correction
        double latDistance = (A*currentPos.getX(METER)+B*currentPos.getY(METER)+C)/
                Math.sqrt(Math.pow(A,2)+Math.pow(B,2)); //From Desmos graph https://www.desmos.com/calculator/uw6fymsdjv
        latCorrection = Range.clip(-pLatController.calculate(latDistance), -Math.PI/2, Math.PI/2);

        //Action triggers
        distanceTraveled = Math.sqrt(Math.pow(distanceBetween(startingPos, currentPos, METER),2)-Math.pow(latDistance,2));
        distanceRemaining = Math.sqrt(Math.pow(distanceBetween(currentPos, targetPos, METER),2)-Math.pow(latDistance,2));
        double totalDistance = distanceBetween(startingPos, targetPos, METER);
        actionCompletion = distanceTraveled/totalDistance;

        //Deceleration if enabled
        if(decelerate) {
            lonCorrection = Range.clip(Math.sqrt(Math.pow(minSpeed, 2) + 2 * maxDecel * distanceRemaining),
                    minSpeed, maxSpeed); //From Desmos graph https://www.desmos.com/calculator/e7plnhpxva
        }
        else {
            lonCorrection = maxSpeed;
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

        double rotSetpoint = floorMod(startingPos.getHeading(RADIANS)+actionCompletion*(targetPos.getHeading(RADIANS)-startingPos.getHeading(RADIANS)+modOffset),2*Math.PI);

        rotCorrection = -pRotController.calculate(floorMod((currentPos.getHeading(RADIANS)-rotSetpoint+Math.PI),(2*Math.PI))-Math.PI);

        Pose2D OrientedVel = new Pose2D(METER,lonCorrection, 0, RADIANS, 0);

        //Align movement to line
        lineAngle = -Math.atan2(targetPos.getY(METER)-startingPos.getY(METER),targetPos.getX(METER)-startingPos.getX(METER));

        targetVel = rotatePose(OrientedVel, RADIANS, lineAngle + latCorrection);
        targetRotVel = rotCorrection;

        writeVel();
    }

    private void holdPos(){
        targetRotVel = -pRotController.calculate(floorMod((currentPos.getHeading(RADIANS)-targetPos.getHeading(RADIANS)+21*Math.PI),(2*Math.PI))-Math.PI);;

        targetVel = new Pose2D(METER,
                Range.clip(-pController.calculate(currentPos.getX(METER),targetPos.getX(METER)),-minSpeed, minSpeed),
                Range.clip(-pController.calculate(currentPos.getY(METER),targetPos.getY(METER)),-minSpeed, minSpeed),
                RADIANS, 0);
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
                    targetPower=rotatePose(targetPower, RADIANS, -currentPos.getHeading(RADIANS));
                }
                writePower();
                break;

            case VELOCITY:
                //Apply robot centric with fresh odo data
                if(!fieldCentric){
                    targetVel=rotatePose(targetVel, RADIANS, -currentPos.getHeading(RADIANS));
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

        angle= RADIANS.fromUnit(angleUnit,angle);

        return new Pose2D(
                METER,
                pose.getX(METER)*Math.cos(angle)-pose.getY(METER)*Math.sin(angle),
                pose.getX(METER)*Math.sin(angle)+pose.getY(METER)*Math.cos(angle),
                RADIANS,
                pose.getHeading(RADIANS)
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

    private double floorMod(double x, double y){
        return x-(Math.floor(x/y)*y);
    }

    //Action Managers
    private void actionInit(){
        actionCompletion = 0;
        actionCompleted = false;

        switch(currentAction.actionType){
            case RUN:
                startingPos = targetPos;
                targetPos = currentAction.targetPos;
                tolorance = currentAction.threshold;
                decelerate = currentAction.decelerate;
                maxDecel = currentAction.speed;
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
                targetVel = new Pose2D(METER, 0, 0, AngleUnit.DEGREES, 0);
                writePos();
                if (distanceRemaining <= tolorance || actionCompletion >= 1 || distanceBetween(startingPos,currentPos, METER)>distanceBetween(startingPos,targetPos, METER)) {
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
                        if (Math.abs(currentPos.getX(METER)-targetPos.getX(METER))<=currentAction.pTolorance &&
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

        public String[] IDs;

        //Delay Termination Variables
        public double delayLength;

        //Position Termination Variables
        public double pTolorance;
        public double hTolorance;

        //Run variables
        public Pose2D targetPos;
        public double speed;
        public boolean decelerate;
        public double threshold; //For transitions

        //Run Action
        public Action(Pose2D targetPos, double speed, boolean decelerate, double threshold, String[] IDs){
            this.actionType = ACTIONTYPE.RUN;
            this.speed = speed;
            this.targetPos = targetPos;
            this.decelerate = decelerate;
            this.threshold = threshold;
            this.IDs = IDs;
        }

        //No Termination
        public Action(HOLDTYPE holdtype, String[] IDs) {
            this.actionType = ACTIONTYPE.HOLD;
            this.holdtype = holdtype;
            this.terminationtype= TERMINATIONTYPE.NONE;
            this.IDs = IDs;
        }

        //Time Termination
        public Action(HOLDTYPE holdtype, double delayLength, String[] IDs) {
            this.actionType = ACTIONTYPE.HOLD;
            this.holdtype = holdtype;
            this.terminationtype = TERMINATIONTYPE.TIME;
            this.delayLength = delayLength;
            this.IDs = IDs;
        }

        //Position Termination; All units are CM and DEG
        public Action(double pTolorance, double hTolorance, String[] IDs) {
            this.actionType = ACTIONTYPE.HOLD;
            this.holdtype = HOLDTYPE.POS;
            this.terminationtype = TERMINATIONTYPE.POS;
            this.pTolorance = pTolorance;
            this.hTolorance = hTolorance;
            this.IDs = IDs;
        }
    }

    public static class IWLocalizer{
        public enum ODOType{
            goBildaPinpoint,
            OTOS
        }
        ODOType odoType;
        GoBildaPinpointDriver goBildaPinpointDriver;
        SparkFunOTOS otos;

        public IWLocalizer(GoBildaPinpointDriver goBildaPinpointDriver){
            this.goBildaPinpointDriver = goBildaPinpointDriver;
            odoType=ODOType.goBildaPinpoint;
        }

        public IWLocalizer(SparkFunOTOS otos){
            this.otos = otos;
            odoType=ODOType.OTOS;
        }

        public void init(Pose2D initPose){
            switch(odoType){
                case goBildaPinpoint:
                    goBildaPinpointDriver.setOffsets(odoXOffset, odoYOffset);
                    goBildaPinpointDriver.setEncoderResolution(odoEncoderResolution);
                    goBildaPinpointDriver.setEncoderDirections(xEncoderDirection, yEncoderDirection);
                    //Set to start counting at initPose parameter
                    goBildaPinpointDriver.resetPosAndIMU();
                    goBildaPinpointDriver.setPosition(initPose);
                    break;

                case OTOS:
                    otos.setLinearUnit(METER);
                    otos.setAngularUnit(RADIANS);

                    otos.setLinearScalar(1.0);
                    otos.setAngularScalar(1.0);

                    otos.setOffset(new SparkFunOTOS.Pose2D(OTOSOffset.getX(METER),OTOSOffset.getY(METER),OTOSOffset.getHeading(RADIANS)));

                    otos.calibrateImu();
                    otos.resetTracking();
                    otos.setPosition(new SparkFunOTOS.Pose2D(initPose.getX(METER),initPose.getY(METER),initPose.getHeading(RADIANS)));

                    otos.begin();
                    break;
                }
                this.update();
            }


        public void update() {
            switch(odoType){
                case goBildaPinpoint:
                    goBildaPinpointDriver.update();
                    break;
                case OTOS:
                    break;
            }
        }

        public Pose2D getPosition(){
            Pose2D output=null;
            switch(odoType) {
                case goBildaPinpoint:
                    output=goBildaPinpointDriver.getPosition();
                break;
                case OTOS:
                    SparkFunOTOS.Pose2D input = otos.getPosition();
                    output=new Pose2D(METER, input.x, input.y, RADIANS, input.h);
                break;
            }
            return output;
        }

        public Pose2D getVelocity(){
            Pose2D output=null;
            switch(odoType) {
                case goBildaPinpoint:
                    output=goBildaPinpointDriver.getVelocity();
                    break;
                case OTOS:
                    SparkFunOTOS.Pose2D input = otos.getVelocity();
                    output=new Pose2D(METER, input.x, input.y, RADIANS, input.h);
                    break;
            }
            return output;
        }

        public double getHeadingVelocity(){
            double output=0;
            switch(odoType) {
                case goBildaPinpoint:
                    output=goBildaPinpointDriver.getHeadingVelocity();
                    break;
                case OTOS:
                    output=otos.getVelocity().h;
                    break;
            }
            return output;
        }
    }
}
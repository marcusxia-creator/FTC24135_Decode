package org.firstinspires.ftc.teamcode.TeleOps;

public class Spindexer {
    public enum SLOT{
        Empty,
        Green,
        Purple
    }

    public RobotHardware robot;
    public SLOT[] slots;
    public int currentSlot;
    //For jams
    public int prevSlot;

    Spindexer(SLOT slot1,SLOT slot2,SLOT slot3, int currentSlot){
        slots = new SLOT[]{slot1, slot2, slot3};
        this.currentSlot = currentSlot;
    }

    public void writeToCurrent(SLOT a){
        slots[currentSlot]=a;
    }

    public void writeToCurrent(ColourCriterion[] criteria){
        //not yet implimented
    }

    public int count(SLOT a){
        int counter = 0;
        for(SLOT slot:slots){
            if(slot==a){
                counter++;
            }
        }
        return counter;
    }

    public Boolean checkFor(SLOT a){
        return count(a)>0;
    }

    public void runToSlot(int n){
        prevSlot = currentSlot;
        currentSlot = (n-1)%3+1;
        if(n==1){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot1);
        }
        if(n==2){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot2);
        }
        if(n==3){
            robot.spindexerServo.setPosition(RobotActionConfig.spindexerSlot3);
        }
    }

    public Boolean runToSlot(SLOT a){
        if(checkFor(a)){
            int n;
            int distance = 4;

            //look for closest slot
            for(int i=0; i<=2; i++){
                if(slots[i]==a && Math.abs(i-currentSlot)<=distance){
                    distance=Math.abs(i-currentSlot);
                    n=i;
                }
            }

            runToSlot(n);
            return true;
        }
        else{
            return false;
        }
    }

    public void unJam(){
        runToSlot(prevSlot);
    }
}
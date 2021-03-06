package frc.robot;

public class Autonomous {

    Lemonlight ParkersLemon = new Lemonlight();
    AutoState autoState = AutoState.doNothing;
    boolean count = true;
    double startAiming;

    double aimStartTime = 0;
    double shootStartTime = 0;


    /**
     * 
     * @return In degress, how much you are changing the steering by
     *         to see the reflective when it doesnt see it
     */
    public double seeking() {
        // double offsetX = ParkersLemon.getHorizontalOffset();
        double valid = Lemonlight.validTarget();
        double steeringAdjust = 0;
        if (valid == 0.0) {
            steeringAdjust = 30;

        } else {
            steeringAdjust = 0;

        }
        return steeringAdjust;
    }

    /**
     * 
     * @return In degress the amount you are adjusting the
     *         robot to line it up with the reflective tape
     */
    

    /**
     * 
     * @return how many inches you move forward
     */
    public double getInRange() {
        double range = 3.28;
        double distance = ParkersLemon.distanceGrab() / (12 * 3.28);
        return distance - range;
    }

    /**
     * 
     * @return getting out of zone during 15 second autonomous period
     */
    public double getOut() {
        double get = 3.045;
        return get;
    }

    public void autoPeriodic(DriveTrain m_drive, Shooter shoot, Climb climb) {

        climb.Teleop(false, false, false);
        switch (autoState) {
            case doNothing:
                shootStartTime = 0;
                aimStartTime = 0;
                shoot.backCollector.set(0);
                shoot.frontCollector.set(0);
                System.out.println("Do Nothing");
                m_drive.drive(0, 0);
                shoot.shooterTeleop(false, false, false, false, false);
                if (count) {
                    autoState = AutoState.moveBack;
                    count = false;
                }
                break;

            case moveBack:
                System.out.println("Move Back");
                m_drive.goDistance(-1.68);
                if (m_drive.driveFinishTime > System.currentTimeMillis()) {
                    m_drive.goDistance(-1.68);
                } 
                else {
                    autoState = AutoState.aim;
                }
                break;

            case aim:
                if (aimStartTime == 0){
                    aimStartTime = System.currentTimeMillis();
                    shoot.Aiming();
                }
                else if (aimStartTime + 3000 > System.currentTimeMillis()){
                    shoot.Aiming();
                }
                else{
                   autoState = AutoState.shoot; 
                }
                break;

            case shoot:
                System.out.println("Shoot");
                shoot.shooterTeleop(true, false, false, false, false);
                if (shootStartTime == 0){
                    shootStartTime = System.currentTimeMillis();
                    shoot.shooterTeleop(true, false, false, false, false);
                }
                else if (System.currentTimeMillis() <= shootStartTime + 8000) {
                    shoot.shooterTeleop(true, false, false, false, false);
                } 
                else {
                    autoState = AutoState.doNothing;
                    shoot.backCollector.set(0);
                    shoot.frontCollector.set(0);
                }
                break;
        }
    }

}

enum AutoState {
    doNothing,
    moveBack,
    aim,
    shoot
}

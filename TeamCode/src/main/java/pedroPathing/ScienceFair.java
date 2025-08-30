package pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "ScienceFair", group = "Adon")
public class ScienceFair extends OpMode {
    private Follower follower;
    private boolean inbounds = true;

    private double x1=-43; //inches from origin
    private double y1=-3; //inches from origin
    private double x2=3; //inches from origin
    private double y2=43; //inches from origin
    private boolean inboundsupwards = true;
    private boolean inboundsdownwards = true;
    private boolean inboundsleftwards = true;
    private boolean inboundsrightwards = true;
    private final Pose startPose = new Pose(0,0,0);
    private PathChain driveToGoalRight, driveToGoalLeft, driveToGoalUp, driveToGoalDown;


    private int pathState = 0;

    private void setPathState(int newState) {
        pathState = newState;
    }

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    public void buildPaths() {
        Pose outofboundsrightwards = new Pose(follower.getPose().getX(), -x2+1, follower.getPose().getHeading());
        driveToGoalRight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(outofboundsrightwards)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), outofboundsrightwards.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(50)
                .build();

        Pose outofboundsleftwards = new Pose(follower.getPose().getX(), -x1-1, follower.getPose().getHeading());
        driveToGoalLeft = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(outofboundsleftwards)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), outofboundsleftwards.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(50)
                .build();

        Pose outofboundsupwards = new Pose(y2-1, follower.getPose().getY(), follower.getPose().getHeading());
        driveToGoalUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(outofboundsupwards)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), outofboundsupwards.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(50)
                .build();

        Pose outofboundsdownwards = new Pose(y1+1, follower.getPose().getY(), follower.getPose().getHeading());
        driveToGoalDown = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(outofboundsdownwards)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), outofboundsdownwards.getHeading())
                .setZeroPowerAccelerationMultiplier(1)
                .setPathEndTimeoutConstraint(50)
                .build();

    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        buildPaths();
        follower.setMaxPower(1);
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*0.4, -gamepad1.left_stick_x*0.4, -gamepad1.right_stick_x*0.4, true);

        inboundsdownwards = follower.getPose().getX() >= y1;
        inboundsupwards = follower.getPose().getX() <= y2;
        inboundsrightwards = follower.getPose().getY() >= -x2;
        inboundsleftwards = follower.getPose().getY() <= -x1;


        switch (pathState) {
            case 0:
                // Normal driving
                follower.setTeleOpMovementVectors(-gamepad1.left_stick_y*0.4, -gamepad1.left_stick_x*0.4, -gamepad1.right_stick_x*0.4, true);

                // Boundary detection -> trigger corrections
                if (!inboundsrightwards) {
                    buildPaths();
                    setPathState(1);
                } else if (!inboundsupwards) {
                    buildPaths();
                    setPathState(2);
                } else if (!inboundsleftwards) {
                    buildPaths();
                    setPathState(3);
                } else if (!inboundsdownwards) {
                    buildPaths();
                    setPathState(4);
                }
                break;

            case 1:
                follower.followPath(driveToGoalRight, 0.2, false);
                setPathState(5);
                break;

            case 2:
                follower.followPath(driveToGoalUp, 0.2, false);
                setPathState(5);
                break;

            case 3:
                follower.followPath(driveToGoalLeft, 0.2, false);
                setPathState(5);
                break;

            case 4:
                follower.followPath(driveToGoalDown, 0.2, false);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.startTeleopDrive();
                    setPathState(0);
                }
                break;
        }



        follower.update();
        telemetry.addData("inboundsleftwards", inboundsleftwards);
        telemetry.addData("inboundsrightwards", inboundsrightwards);
        telemetry.addData("inboundsdownwards", inboundsdownwards);
        telemetry.addData("inboundsupwards", inboundsupwards);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
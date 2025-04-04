package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Poses {
    /*
     * All poses are for the Red Alliance.
     * They are named from the driver perspective then robot perspective.
     * ReefA
     * This means on the reef at the A location
     * See 
     * https://www.chiefdelphi.com/t/frc-4481-team-rembrandts-2025-build-thread-open-alliance/472303/74
     * for labeling of the field.
     */

    public static Pose2d kReefA = new Pose2d();

    public static Pose2d kReefB = new Pose2d();

    public static Pose2d kReefC = new Pose2d();

    public static Pose2d kReefD = new Pose2d();

    public static Pose2d kReefE = new Pose2d();

    public static Pose2d kReefF = new Pose2d();

    public static Pose2d kReefG = new Pose2d();

    public static Pose2d kReefH = new Pose2d();

    public static Pose2d kReefI = new Pose2d();

    public static Pose2d kReefJ = new Pose2d();

    public static Pose2d kReefK = new Pose2d(
        13.04,
        2.55,
        new Rotation2d(Units.degreesToRadians(112.46))
    );

    public static Pose2d kReefL = new Pose2d(
        13.75,
        2.95,
        new Rotation2d(Units.degreesToRadians(115.54))
    );

    public static Pose2d kLeftFeeder = new Pose2d(
        16.43,
        0.98,
        new Rotation2d(Units.degreesToRadians(37.16))
    );

    public static Pose2d kRightFeeder = new Pose2d();


    /**
     * 
     * @param id The id should only be for the left alliance side.
     * @return Poses in [Left, Right] for possible positions for the given id. 
     * If only one position is possible it will return the same pose twice.
     * If could not find a position it will return [new Pose2d(), new Pose2d()]
     */
    public static Pose2d[] aprilTagIdToPoseOptions(int id) {
        switch (id) {
            case 13:
                return new Pose2d[] {kLeftFeeder, kLeftFeeder};

            case 12:
                return new Pose2d[] {kRightFeeder, kRightFeeder};

            case 18:
                return new Pose2d[] {kReefA, kReefB};
            
            case 17:
                return new Pose2d[] {kReefC, kReefD};

            case 22:
                return new Pose2d[] {kReefE, kReefF};

            case 21:
                return new Pose2d[] {kReefG, kReefH};

            case 20:
                return new Pose2d[] {kReefI, kReefJ};

            case 19:
                return new Pose2d[] {kReefK, kReefL};            
        }

        return new Pose2d[] {new Pose2d(), new Pose2d()};
    }


    /**
     * 
     * @param id Red side april tag id
     * @return the blue side april tag id.
     * If could not convert to blue side id. It will return the given id because it must already be blue side
     */
    public static int redSideAprilTagIdToBlueSideId(int id) {
        switch (id) {
            case 1:
                return 13;
            
            case 2:
                return 12;

            case 7:
                return 18;

            case 8:
                return 17;
            
            case 9:
                return 22;

            case 10:
                return 21;

            case 11:
                return 20;
            
            case 6:
                return 19;
            
            case 3:
                return 16;
            
            case 4:
                return 15;
            
            case 5:
                return 14;
        }

        return id;
    }
}

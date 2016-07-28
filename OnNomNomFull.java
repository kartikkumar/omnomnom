import lejos.nxt.Motor;
import lejos.util.Stopwatch;
import lejos.robotics.navigation.DifferentialPilot;
import java.util.Random;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.nxt.LightSensor;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.SensorPort;
import lejos.util.Delay;
import lejos.robotics.navigation.Navigator;
import lejos.nxt.Sound;
import lejos.robotics.navigation.Waypoint;

public class OnNomNom
{
    DifferentialPilot pilot;

    public void go(  int maximumTime,
                     int returnHomeTime,
                     int returnHomeObstacleTime,
                     int delay,
                     boolean returnHomeBlind,
                     int turnInterval,
                     float initialHeading,
                     float ultrasonicSensorOffsetX,
                     float ultrasonicSensorOffsetY,
                     int travelSpeed,
                     double minimumDistance,
                     double xMinimum,
                     double xMaximum,
                     double yMinimum,
                     double yMaximum,
                     double boundsTolerance,
                     int perimeterLightLimit )
    {
        // Create random number generator.
        Random random = new Random( );

        // Create sensors.
        LightSensor lightSensor = new LightSensor( SensorPort.S4 );
        UltrasonicSensor sonicSensor = new UltrasonicSensor( SensorPort.S2 );

        // Set up odometer and set the starting position to starting point (X=0, Y=0; H=0) [cm].
        OdometryPoseProvider poseProvider = new OdometryPoseProvider( pilot );
        Pose pose = new Pose( ultrasonicSensorOffsetX, ultrasonicSensorOffsetY, initialHeading );
        poseProvider.setPose( pose );

        // Create a navigator.
        Navigator navigator = new Navigator( pilot, poseProvider );

        // Set the constant travel speed for the robot [cm/s].
        pilot.setTravelSpeed( travelSpeed );

        // Create stopwatch to time the run.
        Stopwatch stopwatch = new Stopwatch( );

        // Create stopwatch for turning.
        Stopwatch turnTimer = new Stopwatch( );

        // Move forward in a straight line.
        pilot.forward( );

        // Main loop.
        while( true )
        {
            // Read current pose (X, Y; H).
            pose = poseProvider.getPose( );

            // CASE 1: MAXIMUM TIME REACHED.
            // Check if the maximum drive time has been reached and stop if exceeded.
            if ( stopwatch.elapsed( ) > maximumTime )
            {
               pilot.stop( );
               break;
            }

            // CASE 2: OBSTACLE DETECTED.
            // Measure distance to any obstacles and if the distance is less than the minimum, turn.
            if ( sonicSensor.getDistance( ) < minimumDistance )
            {
                pilot.rotate( ( random.nextBoolean( ) ? 1 : -1 ) * 90 );
                pilot.forward( );
            }

            // CASE 3: TURN RANDOMLY.
            // Turn randomly at right angles to the left or right after turn interval.
            if ( turnTimer.elapsed( ) > turnInterval )
            {
                pilot.rotate( ( random.nextBoolean( ) ? 1 : -1 ) * 90 );
                pilot.forward( );
                turnTimer.reset( );
            }

            // CASE 4: INNER BOUNDARY DETECTED.
            // Check if inner boundary has been reached based on absolute pose and turn around.
            if ( pose.getX( ) > ( xMaximum - boundsTolerance )
                 || pose.getX( ) < ( xMinimum + boundsTolerance )
                 || pose.getY( ) > ( yMaximum - boundsTolerance )
                 || pose.getY( ) < ( yMinimum + boundsTolerance ) )
            {
                pilot.rotate( 180 );
                Delay.msDelay( delay );
                pilot.forward( );
                Delay.msDelay( delay );
            }

            // CASE 5: PERIMETER DETECTED (HARD LIMIT).
            // Check if the tape has been reached and turn around.
            // This should only be reached if Case 4 is not working well.
            if ( lightSensor.getLightValue( ) < perimeterLightLimit )
            {
                pilot.rotate( 180 );
                Delay.msDelay( delay );
                pilot.forward( );
                Delay.msDelay( delay );
            }

            // CASE 6: RETURN HOME.
            // Check if it's time to return home, signal and break from loop.
            if ( stopwatch.elapsed( ) > returnHomeTime )
            {
                pilot.stop( );
                System.out.println("[Heading home!]");
                Delay.msDelay( delay );
                Sound.beep( );
                Sound.beep( );
                Sound.beep( );
                Sound.beep( );
                break;
            }
        }

        // Compute the current pose of the rover.
        pose = poseProvider.getPose( );

        // Return home.
        if ( returnHomeBlind )
        {
            navigator.goTo( new Waypoint( 0, 0 ) );
            navigator.waitForStop( );

            System.out.println("[I'm home!]");

            Delay.msDelay( delay );

            Sound.beepSequence( );
            Sound.beepSequence( );
            Sound.beepSequence( );
        }
        else
        {
            // Create stopwatch for obstacle avoidance when heading home.
            Stopwatch homeTimer = new Stopwatch( );

            pilot.stop( );

            // Loop until you reach home.
            while ( Math.abs( pose.getX( ) ) > 0 || Math.abs( pose.getY( ) ) > 0 )
            {
                // Store current heading.
                double currentHeading = pose.getHeading( );

                // Compute heading to go home.
                double homeHeading = Math.atan2( pose.getY( ), pose.getX( ) );

                Delay.msDelay( delay );

                // Rotate from current heading to heading to go home.
                pilot.rotate( currentHeading - homeHeading );

                // Go forward.
                pilot.forward( );

                // Measure distance to any obstacles and if the distance is less than the minimum,
                // turn.
                if ( sonicSensor.getDistance( ) < minimumDistance )
                {
                    pilot.rotate( ( random.nextBoolean( ) ? 1 : -1 ) * 90 );
                    Delay.msDelay( delay );
                    homeTimer.reset( );
                    pilot.forward( );

                    // Stop going forward after set time.
                    if ( homeTimer.elapsed( ) > returnHomeObstacleTime )
                    {
                        pilot.stop( );
                    }
                }
            }

        }
    }

    public static void main( String[ ] args )
    {
        /////////////////////////////////////////////

        // Set up input deck.

        // Set maximum run time [ms].
        int maximumTime = 120 * 1000;

        // Set return-to-base epoch [ms].
        int returnHomeTime = 60 * 1000;

        // Set return-to-base obstacle avoidance time [ms].
        int returnHomeObstacleTime = 2000;

        // Set delay [ms].
        int delay = 500;

        // Set if the rover should return home blind (ignoring obstacles).
        boolean returnHomeBlind = true;

        // Set turn interval [ms].
        int turnInterval = 20000;

        // Set wheel diameter [cm].
        double wheelDiameter = 8.0;

        // Set vehicle track [cm].
        double vehicleTrack = 15.0;

        // Set initial heading [deg].
        float initialHeading = 0.0f;

        // Set offset of ultrasonic sensor from center of robot [cm].
        float ultrasonicSensorOffsetX = 13.0f;
        float ultrasonicSensorOffsetY = 4.0f;

        // Constant travel speed [cm/s].
        int travelSpeed = 10;

        // Obstacle minimum distance [cm].
        double minimumDistance = 20.0;

        // Area bounds and bounds tolerance [cm].
        double xMinimum = -90.0;
        double xMaximum = 90.0;
        double yMinimum = -90.0;
        double yMaximum = 90.0;
        double boundsTolerance = 10.0;

        // Set scoop rotation speed [deg/s].
        int scoopRotationSpeed = 180;

        // Set perimeter light value (light sensor reading for tape).
        int perimeterLightLimit = 30;

        /////////////////////////////////////////////

        /////////////////////////////////////////////

        // Execute run.

        // Set max rotational speed for scoop and set in motion [deg/s].
        Motor.B.setSpeed( scoopRotationSpeed );

        // Start scoop.
        Motor.B.forward( );

        // Create on-board pilot and run motor.
        OnNomNom omNomNom = new OnNomNom( );
        omNomNom.pilot = new DifferentialPilot( wheelDiameter, vehicleTrack, Motor.A, Motor.C );
        omNomNom.go( maximumTime,
                     returnHomeTime,
                     returnHomeObstacleTime,
                     delay,
                     returnHomeBlind,
                     turnInterval,
                     initialHeading,
                     ultrasonicSensorOffsetX,
                     ultrasonicSensorOffsetY,
                     travelSpeed,
                     minimumDistance,
                     xMinimum,
                     xMaximum,
                     yMinimum,
                     yMaximum,
                     boundsTolerance,
                     perimeterLightLimit );

        /////////////////////////////////////////////
    }
}

package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.masscalc.RigidBody;
import net.sf.openrocket.simulation.AccelerationData;
import net.sf.openrocket.simulation.RK4SimulationStatus;
import net.sf.openrocket.simulation.RK4SimulationStepper;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.simulation.listeners.SimulationListenerHelper;
import net.sf.openrocket.startup.Application;
import net.sf.openrocket.startup.Preferences;
import net.sf.openrocket.util.Coordinate;

import net.sf.openrocket.simulation.extension.impl.RLModel.Action;
import net.sf.openrocket.util.MathUtil;
import net.sf.openrocket.util.Quaternion;
import net.sf.openrocket.util.Rotation2D;

import java.util.ArrayList;
import java.util.Random;

public class RocketLanderListener extends AbstractSimulationListener {
    private RLEpisodeManager episodeManager = RLEpisodeManager.getInstance();
    private RLModel model = RLModel.getInstance();
    private ArrayList<StateActionTuple> episodeStateActions;
    //HashMap<String, ArrayList<Double>> episodeData;
    private RocketLander rocketLander;
    private Random random;
    private Boolean forcingFailure = false;
    private static double variation = 5;
    private static double timeStep = 0.05;  // RK4SimulationStepper.MIN_TIME_STEP --> 0.001

    Action action;

    // thrust vectoring
    private FlightConditions RLVectoringFlightConditions = null;
    private AerodynamicForces RLVectoringAerodynamicForces = null;
    private RigidBody RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private double RLVectoringThrust;


    RocketLanderListener(RocketLander rocketLander) {
        this.rocketLander = rocketLander;
        random = new Random();
    }

    private double calculateNumberWithIntegerVariation(double startNumber, double variation) {
        return startNumber - variation / 2 + variation * random.nextDouble();
    }

    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        //episodeData = episodeManager.initializeEmptyEpisode();
        episodeManager.initializeEpisodeManager();
        model.initializeModel();
        episodeStateActions = episodeManager.initializeEmptyActionStateTuples();
        episodeManager.setupParameters(status);

        status.getMotors().iterator().next().getConfig().setIgnitionDelay(5);

        // set the rocket position at the launch altitude as defined by the extension
        //status.setRocketPosition(new Coordinate(0, 0, calculateNumberWithIntegerVariation(100, variation)));
        status.setRocketPosition(new Coordinate(0, 0, calculateNumberWithIntegerVariation(rocketLander.getLaunchAltitude(), variation)));
        // set the rocket velocity at the rocket velocity as defined by the extension
        //status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(-40, variation))));
        status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(rocketLander.getLaunchVelocity(), variation))));
        // set the simulation timeStep
        status.getSimulationConditions().setTimeStep(timeStep);
    }

    /*
    @Override
    public double preSimpleThrustCalculation(SimulationStatus status) throws SimulationException {
        // note we would want to also fix the fuel.  This ignores the fuel level of the rocket.

        // status.getActiveMotors().iterator().next().getThrust(status.getSimulationTime());
        //status.getRocketVelocity();
        //return 0.0;

        double MAX_THRUST = 150;

        action = model.run_policy(status, episodeStateActions);
        // return Double.NaN;
        //if (status.getSimulationTime() < 0.1) {
        //    return MAX_THRUST;
        //} else {
            return MAX_THRUST * action.thrust;
        //}
    }
    */





    @Override
    public FlightConditions postFlightConditions(SimulationStatus status, FlightConditions flightConditions) throws SimulationException {
        this.RLVectoringFlightConditions = flightConditions;
        return null;
    }

    @Override
    public AerodynamicForces postAerodynamicCalculation(SimulationStatus status, AerodynamicForces forces) throws SimulationException {
        this.RLVectoringAerodynamicForces = forces;
        return null;
    }

    @Override
    public double postSimpleThrustCalculation(SimulationStatus status, double thrust) throws SimulationException {
        RLVectoringThrust = thrust;
        return Double.NaN;
    }

    @Override
    public RigidBody postMassCalculation(SimulationStatus status, RigidBody rigidBody) throws SimulationException {
        RLVectoringStructureMassData = RLVectoringStructureMassData.add(rigidBody);
        return null;
    }

    // TODO: should be PRE -- BUT then the thrust method will not be called.
    @Override
    public AccelerationData postAccelerationCalculation(SimulationStatus status, AccelerationData acceleration) throws SimulationException {
        if (RLVectoringFlightConditions == null) return null;

        action = model.run_policy(status, episodeStateActions);

        // TODO: SUPER DUPER BROKEN HERE!!!

        // RLModel.State state = episodeStateActions.get(episodeStateActions.size() - 1).state;
        double X = status.getRocketOrientationQuaternion().getX();
        double Y = status.getRocketOrientationQuaternion().getY();
        double Z = status.getRocketOrientationQuaternion().getZ();
        double W = status.getRocketOrientationQuaternion().getW();
        double xDir =  2 * (X * Z - W * Y);
        double yDir = 2 * (Y * Z + W * X);
        double zDir  = 1 - 2 * (X * X + Y * Y);
        xDir = Math.atan2(2 * Y * W + 2 * X * Z, 1 - 2 * Y * Y - 2 * Z * Z);
        yDir = Math.atan2(2 * X * W + 2 * Y * Z, 1 - 2 * X * X - 2 * Z * Z);
        zDir = Math.asin(2 * X * Y + 2 * Z * W);


        double rocketTheta = RLVectoringFlightConditions.getTheta();
        double theta = rocketTheta - Math.atan2(yDir, xDir);

        double move_gimbal_to_x = Math.cos(xDir) / 20; //Math.cos(theta) / 10;
        double move_gimbal_to_y = Math.sin(yDir) / 20; // Math.sin(theta) / 10;
//        move_gimbal_to_x=Math.PI/8;

        //Coordinate optimalGimbalCoordinate = new Coordinate(move_gimbal_to_x, move_gimbal_to_y, 0);
        // optimalGimbalCoordinate = new Rotation2D(-RLVectoringFlightConditions.getTheta()).rotateZ(optimalGimbalCoordinate);

        //optimalGimbalCoordinate = status.getRocketOrientationQuaternion().invRotate(optimalGimbalCoordinate);



        action = new Action(0.6, move_gimbal_to_x, move_gimbal_to_y);

        RLVectoringThrust *= action.thrust;

        // return calculateAcceleration(status, action.getGimble_x(), action.getGimble_y());
        return calculateAcceleration(status, move_gimbal_to_x, move_gimbal_to_y);
    }





    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {
        // status.setRocketOrientationQuaternion(new Quaternion(0, 0, 0, 1));
        return true;
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        Coordinate terminalCoordinate = new Coordinate(0,0,1);
        Coordinate terminalVelocity = new Coordinate(0,0,-100);

        if ((status.getRocketPosition().z > 200.0) || (status.getSimulationTime() > 15.0)) {
            status.setRocketVelocity(terminalVelocity);
            throw new SimulationException("Simulation Was NOT UNDER CONTROL.");
        }

        // ignore adding the data for now
        // episodeManager.addData(status, episodeData);
    }
    @Override
    public void endSimulation(SimulationStatus status, SimulationException exception) {
        // episodeManager.addEpisode(episodeData);
        model.updateStateActionValueFuncton(episodeStateActions);
        //System.out.println("Numbers of iterations: " + episodeStateActions.size() + " " + episodeStateActions.get(episodeStateActions.size()-1).state.velocity);
    }

















    private AccelerationData calculateAcceleration(SimulationStatus status, Double gimble_x, Double gimble_y) throws SimulationException {
        // pre-define the variables for the Acceleration Data
        Coordinate linearAcceleration;
        Coordinate angularAcceleration;

        // Calculate the forces from the aerodynamic coefficients

        double dynP = (0.5 * RLVectoringFlightConditions.getAtmosphericConditions().getDensity() *
                MathUtil.pow2(RLVectoringFlightConditions.getVelocity()));
        double refArea = RLVectoringFlightConditions.getRefArea();
        double refLength = RLVectoringFlightConditions.getRefLength();


        // Linear forces in rocket coordinates
        double dragForce = RLVectoringAerodynamicForces.getCaxial() * dynP * refArea;
        double fN = RLVectoringAerodynamicForces.getCN() * dynP * refArea;
        double fSide = RLVectoringAerodynamicForces.getCside() * dynP * refArea;

        // gimble direction calculations
        double gimbleComponentX = - Math.sin(gimble_x);
        double gimbleComponentY = - Math.sin(gimble_y);
        double gimbleComponentZ = - Math.sqrt(1.0 - Math.pow(gimbleComponentX, 2) + Math.pow(gimbleComponentY, 2));

        assert RLVectoringThrust >= 0;

        // thrust vectoring force
        double forceX = RLVectoringThrust * gimbleComponentX;
        double forceY = RLVectoringThrust * gimbleComponentY;
        double forceZ = - RLVectoringThrust * gimbleComponentZ;

        // final directed force calculations
        double finalForceX = forceX - fN;
        double finalForceY = forceY - fSide;
        double finalForceZ = forceZ - dragForce;


        linearAcceleration = new Coordinate(finalForceX / RLVectoringStructureMassData.getMass(),
                finalForceY / RLVectoringStructureMassData.getMass(),
                finalForceZ / RLVectoringStructureMassData.getMass()
        );

        linearAcceleration = new Rotation2D(RLVectoringFlightConditions.getTheta()).rotateZ(linearAcceleration);

        // Convert into rocket world coordinates
        linearAcceleration = status.getRocketOrientationQuaternion().rotate(linearAcceleration);

        // add effect of gravity
        double gravity = status.getSimulationConditions().getGravityModel().getGravity(status.getRocketWorldPosition());
        linearAcceleration = linearAcceleration.sub(0, 0, gravity);

        // add effect of Coriolis acceleration
        Coordinate coriolisAcceleration = status.getSimulationConditions().getGeodeticComputation()
                .getCoriolisAcceleration(status.getRocketWorldPosition(), status.getRocketVelocity());
        linearAcceleration = linearAcceleration.add(coriolisAcceleration);

        // If still on the launch rod, project acceleration onto launch rod direction and
        // set angular acceleration to zero.
        if (!status.isLaunchRodCleared()) {
            double launchRodAngle = status.getSimulationConditions().getLaunchRodAngle();
            double launchRodDirection = status.getSimulationConditions().getLaunchRodDirection();
            Coordinate launchDirectionCoordinate = new Coordinate(
                    Math.sin(launchRodAngle) * Math.cos(Math.PI / 2.0 - launchRodDirection),
                    Math.sin(launchRodAngle) * Math.sin(Math.PI / 2.0 - launchRodDirection),
                    Math.cos(launchRodAngle)
            );

            linearAcceleration = launchDirectionCoordinate.multiply(
                    linearAcceleration.dot(launchDirectionCoordinate));
            angularAcceleration = Coordinate.NUL;

        } else {

            // Shift moments to CG
            double Cm = RLVectoringAerodynamicForces.getCm() - RLVectoringAerodynamicForces.getCN() * RLVectoringStructureMassData.getCM().x / refLength;
            double Cyaw = RLVectoringAerodynamicForces.getCyaw() - RLVectoringAerodynamicForces.getCside() * RLVectoringStructureMassData.getCM().x / refLength;

            double momentArm = status.getConfiguration().getLength() - RLVectoringStructureMassData.cm.x;
            double gimbleMomentX = momentArm * forceX;
            double gimbleMomentY = momentArm * forceY;

            // Compute moments
//            double momX = -Cyaw * dynP * refArea * refLength + gimbleMomentX;
            double r = status.getFlightConfiguration().getReferenceLength()/2;
            double wx = RLVectoringFlightConditions.getYawRate();
            double wy = RLVectoringFlightConditions.getPitchRate();
//            double wz = RLVectoringFlightConditions.getYawRate();
            double h = status.getConfiguration().getLength();
            double rho = RLVectoringFlightConditions.getAtmosphericConditions().getDensity();
            double Tx = - Math.signum(wx) * Math.PI * Math.pow(wx,2) * Math.pow(r,4) * h * rho * RLVectoringAerodynamicForces.getCyaw();
            double Ty = - Math.signum(wy) * Math.PI * Math.pow(wy,2) * Math.pow(r,4) * h * rho * RLVectoringAerodynamicForces.getCm();
//            double Tz = - Math.signum(wz)*Math.PI*Math.pow(wy,2)*Math.pow(r,4)*h*rho*RLVectoringAerodynamicForces.getCyaw();
            double momX = -Cyaw * dynP * refArea * refLength + gimbleMomentX + Tx;
            double momY = Cm * dynP * refArea * refLength + gimbleMomentY + Ty;
            double momZ = RLVectoringAerodynamicForces.getCroll() * dynP * refArea * refLength;

            // Compute acceleration in rocket coordinates
            angularAcceleration = new Coordinate(momX / RLVectoringStructureMassData.getLongitudinalInertia(),
                    momY / RLVectoringStructureMassData.getLongitudinalInertia(),
                    momZ / RLVectoringStructureMassData.getRotationalInertia());

            double rollAcceleration = angularAcceleration.z;
            // TODO: LOW: This should be hypot, but does it matter?
            double lateralPitchAcceleration = MathUtil.max(Math.abs(angularAcceleration.x),
                    Math.abs(angularAcceleration.y));

            angularAcceleration = new Rotation2D(RLVectoringFlightConditions.getTheta()).rotateZ(angularAcceleration);

            // Convert to world coordinates
            angularAcceleration = status.getRocketOrientationQuaternion().rotate(angularAcceleration);

            int a = 5;
        }

        RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
        return new AccelerationData(null, null, linearAcceleration, angularAcceleration, status.getRocketOrientationQuaternion());
    }



}

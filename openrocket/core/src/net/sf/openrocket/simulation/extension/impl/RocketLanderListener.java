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

import net.sf.openrocket.simulation.extension.impl.RLModel.*;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;
import net.sf.openrocket.util.MathUtil;
import net.sf.openrocket.util.Quaternion;
import net.sf.openrocket.util.Rotation2D;

import java.util.ArrayList;
import java.util.Random;

public class RocketLanderListener extends AbstractSimulationListener {
    private RLEpisodeManager episodeManager = RLEpisodeManager.getInstance();
    private RLModel model = RLModel.getInstance();
    private RLMethod method = model.getCurrentMethod();
    private ArrayList<StateActionTuple> episodeStateActions;
    //HashMap<String, ArrayList<Double>> episodeData;
    private RocketLander rocketLander;
    private Random random;
    private State state;
    private Action action;
    private Boolean forcingFailure = false;
    private static double variation = 5;
    private static double timeStep = 0.05;  // RK4SimulationStepper.MIN_TIME_STEP --> 0.001

    // thrust vectoring
    private FlightConditions RLVectoringFlightConditions = null;
    private AerodynamicForces RLVectoringAerodynamicForces = null;
    private RigidBody RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private RigidBody OLD_RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private double RLVectoringThrust;


    RocketLanderListener(RocketLander rocketLander) {
        this.rocketLander = rocketLander;
        random = new Random();
    }

    private double calculateNumberWithIntegerVariation(double startNumber, double variation) {
        return startNumber - variation / 2 + variation * random.nextDouble();
    }

    private void setupStateActionAndStore(SimulationStatus status) {
        state = new State(status);
        if (episodeStateActions.size() != 0) {
            StateActionTuple lastStateAction = episodeStateActions.get(episodeStateActions.size() - 1);
            //state.setGimbleYWithoutRounding(lastStateAction.action.gimbleY);
            //state.setGimbleZWithoutRounding(lastStateAction.action.gimbleZ);
            state.setThrust(lastStateAction.action.thrust);
        } else {
            state.thrust = 100.0;
        }
        action = model.generateAction(state);

        episodeStateActions.add(new StateActionTuple(state, action));
    }

    @Override
    public void startSimulation(SimulationStatus status) {
        episodeManager.initializeEpisodeManager();
        model.initializeModel();
        episodeStateActions = episodeManager.initializeEmptyActionStateTuples();
        episodeManager.setupParameters(status);
        status.getSimulationConditions().setTimeStep(timeStep);

        // set the rocket position at the launch altitude as defined by the extension
        //status.setRocketPosition(new Coordinate(0, 0, calculateNumberWithIntegerVariation(100, variation)));
        status.setRocketPosition(new Coordinate(0, 0, calculateNumberWithIntegerVariation(rocketLander.getLaunchAltitude(), variation)));

        // set the rocket velocity at the rocket velocity as defined by the extension
        //status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(-40, variation))));
        status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(rocketLander.getLaunchVelocity(), variation))));

        // initialize the state and the action
        setupStateActionAndStore(status);
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
    public boolean preStep(SimulationStatus status) {
        status.setRocketOrientationQuaternion(new Quaternion(0, 0, 0, 1));
        return true;
    }



    @Override
    public FlightConditions postFlightConditions(SimulationStatus status, FlightConditions flightConditions) {
        if (flightConditions != null) {
            this.RLVectoringFlightConditions = flightConditions;
        }
        return flightConditions;
    }

    @Override
    public AerodynamicForces postAerodynamicCalculation(SimulationStatus status, AerodynamicForces forces) {
        this.RLVectoringAerodynamicForces = forces;
        return null;
    }

    @Override
    public double postSimpleThrustCalculation(SimulationStatus status, double thrust) {
        RLVectoringThrust = thrust;
        return Double.NaN;
    }

    @Override
    public RigidBody postMassCalculation(SimulationStatus status, RigidBody rigidBody) {
        RLVectoringStructureMassData = RLVectoringStructureMassData.add(rigidBody);
        return null;
    }

    // TODO: should be PRE -- BUT then the thrust method will not be called.
    @Override
    public AccelerationData preAccelerationCalculation(SimulationStatus status) {
        if (RLVectoringFlightConditions == null) return null;

        //action = new Action(0.6, move_gimbal_to_y, move_gimbal_to_z);
        RLVectoringThrust *= action.thrust / 100.0;
        return calculateAcceleration(status, action.gimbleY, action.gimbleZ);
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        Coordinate terminalVelocity = new Coordinate(0,0,-1000);

        if ((status.getRocketPosition().z > 200.0) || (status.getSimulationTime() > 15.0)) {
            status.setRocketVelocity(terminalVelocity);
            throw new SimulationException("Simulation Was NOT UNDER CONTROL.");
        }

        setupStateActionAndStore(status);

        model.updateStepStateActionValueFunction(episodeStateActions);
    }

    @Override
    public void endSimulation(SimulationStatus status, SimulationException exception) {
        model.updateTerminalStateActionValueFunction(episodeStateActions);
    }








    private AccelerationData calculateAcceleration(SimulationStatus status, Double gimbleY, Double gimbleZ) {
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

        // custom action here if wanted  TODO: NEED TO MOVE THE FOLLOWING LINES
        double x = episodeStateActions.get(episodeStateActions.size() - 1).state.angleX;
        double z = episodeStateActions.get(episodeStateActions.size() - 1).state.angleZ;
        double theta = (x - Math.PI) % (2 * Math.PI);
        double move_gimbal_to_x = Math.sin(z) * Math.cos(theta) / 10;
        double move_gimbal_to_y = Math.sin(z) * Math.sin(theta) / 10;


        // gimble direction calculations
        double gimbleComponentX = move_gimbal_to_x;
        double gimbleComponentY = move_gimbal_to_y;
        double gimbleComponentZ = - Math.sqrt(1.0 - Math.pow(gimbleComponentX, 2) + Math.pow(gimbleComponentY, 2));

        assert RLVectoringThrust >= 0;

        // thrust vectoring force
        double forceX = - RLVectoringThrust * gimbleComponentX;
        double forceY = - RLVectoringThrust * gimbleComponentY;
        double forceZ = - RLVectoringThrust * gimbleComponentZ;  // note double negative

        // final directed force calculations
        double finalForceX = forceX - fN;
        double finalForceY = forceY - fSide;
        double finalForceZ = forceZ - dragForce;

        if (RLVectoringStructureMassData.getMass() == 0) {
            RLVectoringStructureMassData = OLD_RLVectoringStructureMassData;
        }
        if (RLVectoringStructureMassData.getMass() == 0) {
            new AccelerationData(null, null, new Coordinate(0, 0, 0), new Coordinate(0, 0, 0), status.getRocketOrientationQuaternion());
        }

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
            double gimbleMomentX = momentArm * forceY;
            double gimbleMomentY = - momentArm * forceX;

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
        }

        OLD_RLVectoringStructureMassData = RLVectoringStructureMassData;
        RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
        return new AccelerationData(null, null, linearAcceleration, angularAcceleration, status.getRocketOrientationQuaternion());
    }
}

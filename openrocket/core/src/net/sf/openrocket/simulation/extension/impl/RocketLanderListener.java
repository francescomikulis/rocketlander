package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.masscalc.RigidBody;
import net.sf.openrocket.simulation.AccelerationData;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.util.Coordinate;

import net.sf.openrocket.simulation.extension.impl.RLModel.*;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import net.sf.openrocket.util.MathUtil;
import net.sf.openrocket.util.Quaternion;

import java.util.ArrayList;
import java.util.Random;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

public class RocketLanderListener extends AbstractSimulationListener {
    private static final double MIN_VELOCITY = -10;
    private static final double MAX_ALTITUDE = 30;
    private static final double MAX_POSITION = 10;

    private RLEpisodeManager episodeManager = RLEpisodeManager.getInstance();
    private RLModel model = RLModel.getInstance();
    private ArrayList<StateActionTuple> episodeStateActionsPrimary;
    private ArrayList<StateActionTuple> episodeStateActionsGimbalX;
    private ArrayList<StateActionTuple> episodeStateActionsGimbalY;
    //HashMap<String, ArrayList<Double>> episodeData;
    private RocketLander rocketLander;
    private Random random;
    private CoupledStates state;
    private CoupledActions action;
    TerminationBooleanTuple terminationBooleanTuple;
    private int[] lastStepUpdateSizes = new int[]{0, 0, 0};
    private static double variation = 2;
    private static double timeStep = 0.01;  // RK4SimulationStepper.MIN_TIME_STEP --> 0.001

    // thrust vectoring
    private FlightConditions RLVectoringFlightConditions = null;
    private AerodynamicForces RLVectoringAerodynamicForces = null;
    private RigidBody RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private RigidBody OLD_RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private double RLVectoringThrust;

    private boolean hasCompletedTerminalUpdate = false;

    /** Used by the Visualize3DListener extension */
    public CoupledActions getLastAction() {
        return model.getLastAction(episodeStateActionsPrimary, episodeStateActionsGimbalX, episodeStateActionsGimbalY);
    }

    RocketLanderListener(RocketLander rocketLander) {
        this.rocketLander = rocketLander;
        random = new Random();
    }

    private double calculateNumberWithIntegerVariation(double startNumber, double variation) {
        return startNumber - (variation) + 2 * variation * random.nextDouble();
    }

    private boolean setupStateActionAndStore(SimulationStatus status) {
        CoupledStates oldState = state;
        state = model.generateNewCoupledStates(status);

        model.updateStateBeforeNextAction(state, episodeStateActionsPrimary, episodeStateActionsGimbalX, episodeStateActionsGimbalY);

        CoupledActions newAction = model.generateAction(state, episodeStateActionsPrimary, episodeStateActionsGimbalX, episodeStateActionsGimbalY);
        if (newAction != null) {
            if ((action == null) || !action.equals(newAction)) {
                // System.out.println("NEW action thrust = " + newAction.getDouble("thrust"));
                //System.out.println("NEW action name = " + newAction.definition.get("meta").get("name"));
                //System.out.println("NEW action hashCode = " + newAction.hashCode());
                if (action != null) {
                    //System.out.println("OLD action thrust = " + action.getDouble("thrust"));
                    //System.out.println("OLD action name = " + action.definition.get("meta").get("name"));
                    //System.out.println("OLD action hashCode = " + action.hashCode());
                }
            }
            action = newAction;
        }
        return true;
    }

    @Override
    public void startSimulation(SimulationStatus status) {
        episodeManager.initializeEpisodeManager();
        model.initializeModel();
        episodeStateActionsPrimary = episodeManager.initializeEmptyActionStateTuples();
        episodeStateActionsGimbalX = episodeManager.initializeEmptyActionStateTuples();
        episodeStateActionsGimbalY = episodeManager.initializeEmptyActionStateTuples();
        episodeManager.setupParameters(status);
        status.getSimulationConditions().setTimeStep(timeStep);

        double posX = calculateNumberWithIntegerVariation(0, MAX_POSITION);
        posX = 0;
        double posY = calculateNumberWithIntegerVariation(0, MAX_POSITION);
        posY = 0;
        double posZ = calculateNumberWithIntegerVariation(MAX_ALTITUDE - variation, variation);

        // set the rocket position at the launch altitude as defined by the extension
        status.setRocketPosition(new Coordinate(posX, posY, posZ));
        //status.setRocketPosition(new Coordinate(0, 0, calculateNumberWithIntegerVariation(MAX_ALTITUDE - variation, variation)));
        //status.setRocketPosition(new Coordinate(0, 0, calculateNumberWithIntegerVariation(rocketLander.getLaunchAltitude(), variation)));

        // set the rocket velocity at the rocket velocity as defined by the extension
        status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(MIN_VELOCITY + variation, variation))));
        //status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(rocketLander.getLaunchVelocity(), variation))));

        double dx = calculateNumberWithIntegerVariation(0, variation * 3);
        double dy = calculateNumberWithIntegerVariation(0, variation * 3);
        double dz = 90;
        status.setRocketOrientationQuaternion(new Quaternion(0, dx, dy, dz).normalizeIfNecessary());
        // NOTE: IMPORTANT - DISABLED RANDOM ANGLE STARTS HERE!
        // status.setRocketOrientationQuaternion(new Quaternion(0, 0, 0, 1));

        status.setLaunchRodCleared(true);

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

    public void setRollToZero(SimulationStatus status) {
        Coordinate rotVel = status.getRocketRotationVelocity();
        rotVel = rotVel.setZ(0.0);
        status.setRocketRotationVelocity(rotVel);
    }

    public void stabilizeRocketBasedOnSimType(SimulationStatus status) {
        setRollToZero(status);  // prevent rocket from spinning
        if (model.simulationType == SimulationType._1D) {
            status.setRocketOrientationQuaternion(new Quaternion(0, 0, 0, 1)); // set rocket to vertical
        } else if(model.simulationType == SimulationType._2D) {
            /*
            Quaternion currentQuaternion = status.getRocketOrientationQuaternion();
            Coordinate rocketDirection = convertRocketStatusQuaternionToDirection(status);
            Quaternion newQuaternion = new Quaternion(0, rocketDirection.x / 2, 0, rocketDirection.z).normalizeIfNecessary();
            status.setRocketOrientationQuaternion(newQuaternion);
             */
            // force the stabilizer to be trained here!
            status.setRocketPosition(new Coordinate(0, 0, status.getRocketPosition().z));
        } else if(model.simulationType == SimulationType._3D) {
            // already set the roll to zero
        }
    }

    @Override
    public boolean preStep(SimulationStatus status) {
        stabilizeRocketBasedOnSimType(status);
        return true;
    }



    @Override
    public FlightConditions postFlightConditions(SimulationStatus status, FlightConditions flightConditions) {
        if (flightConditions != null) {
            this.RLVectoringFlightConditions = flightConditions;
            this.RLVectoringFlightConditions.setTheta(0.0);
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

        //action = new Action(60.0, move_gimbal_to_y, move_gimbal_to_z);
        RLVectoringThrust *= (action.getDouble("thrust"));
        double gimbalX = action.getDouble("gimbalX");
        double gimbalY = action.getDouble("gimbalY");
        if (model.simulationType == SimulationType._1D) {
            gimbalX = 0.0;
            gimbalY = 0.0;
        }
        return calculateAcceleration(status, gimbalX, gimbalY);
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        stabilizeRocketBasedOnSimType(status);

        boolean addedStateActionTuple = setupStateActionAndStore(status);

        /*
        if (terminationBooleanTuple.simulationFailed() || (status.getSimulationTime() > MAX_TIME)) {
            throw new SimulationException("Simulation Was NOT UNDER CONTROL.");
        }
         */

        model.updateStepStateActionValueFunction(episodeStateActionsPrimary, episodeStateActionsGimbalX, episodeStateActionsGimbalY, lastStepUpdateSizes);
        lastStepUpdateSizes = new int[]{episodeStateActionsPrimary.size(), episodeStateActionsGimbalX.size(), episodeStateActionsGimbalY.size()};
    }

    @Override
    public void endSimulation(SimulationStatus status, SimulationException exception) {
        // this method is called at least twice if a SimulationException occurs - this is why the boolean was created

        /*
        System.out.print("Thrusts: ");
        for (int i = 0; i < episodeStateActionsPrimary.size(); i ++) {
            StateActionTuple stateActionTuple = episodeStateActionsPrimary.get(i);
            System.out.print("(" + i + ")" +  stateActionTuple.action + " ");
        }
        System.out.println("");
         */

        terminationBooleanTuple = model.getValueFunctionTable().getTerminationValidity(model.generateNewCoupledStates(status));
        if (!hasCompletedTerminalUpdate) {
            model.updateTerminalStateActionValueFunction(episodeStateActionsPrimary, episodeStateActionsGimbalX, episodeStateActionsGimbalY, terminationBooleanTuple);
            hasCompletedTerminalUpdate = true;
        }
    }








    private AccelerationData calculateAcceleration(SimulationStatus status, Double gimbalX, Double gimbalY) {
        // pre-define the variables for the Acceleration Data
        Coordinate linearAcceleration;
        Coordinate angularAcceleration;

        // Calculate the forces from the aerodynamic coefficients
        double dynP = (0.5 * RLVectoringFlightConditions.getAtmosphericConditions().getDensity() *
                MathUtil.pow2(RLVectoringFlightConditions.getVelocity()));
        double refArea = RLVectoringFlightConditions.getRefArea();
        double refLength = RLVectoringFlightConditions.getRefLength();


        // Linear forces in rocket coordinates
        double dragForce = Math.signum(status.getRocketVelocity().z) * RLVectoringAerodynamicForces.getCaxial() * dynP * refArea;
        double fN = RLVectoringAerodynamicForces.getCN() * dynP * refArea;
        double fSide = RLVectoringAerodynamicForces.getCside() * dynP * refArea;  // Cside may be ALWAYS 0

        // gimbal direction calculations
        /*
        double gimbalComponentX = Math.sin(gimbalZ) * Math.cos(gimbalY);
        double gimbalComponentY = Math.sin(gimbalZ) * Math.sin(gimbalY);
        double gimbalComponentZ = Math.cos(gimbalZ);
         */
        double gimbalComponentX = Math.sin(gimbalX);
        double gimbalComponentY = Math.sin(gimbalY);
        double gimbalComponentZ = Math.sqrt(1.0 - gimbalComponentX * gimbalComponentX - gimbalComponentY * gimbalComponentY);

        assert RLVectoringThrust >= 0;

        // thrust vectoring force
        double forceX = -RLVectoringThrust * gimbalComponentX;
        double forceY = -RLVectoringThrust * gimbalComponentY;
        double forceZ = RLVectoringThrust * gimbalComponentZ;  // note double negative

        // System.out.println(gimbalComponentX + " " + gimbalComponentY + " " + gimbalComponentZ);

        // final directed force calculations
        double finalForceX = forceX;// - fN;  // TODO: FIX THIS URGENTLY
        double finalForceY = forceY;// - fSide;  // TODO: FIX THIS URGENTLY
        double finalForceZ = forceZ - dragForce;

        if (RLVectoringStructureMassData.getMass() == 0) {
            RLVectoringStructureMassData = OLD_RLVectoringStructureMassData;
            // catch statement if the above assignment fails
            if (RLVectoringStructureMassData.getMass() == 0) {
                System.out.println("SERIOUS MASS DEFECT HERE.  IF THIS EVER PRINTS THERE ARE SERIOUS ISSUES.");
                return new AccelerationData(null, null, new Coordinate(0, 0, 0), new Coordinate(0, 0, 0), status.getRocketOrientationQuaternion());
            }
        }

        linearAcceleration = new Coordinate(finalForceX / RLVectoringStructureMassData.getMass(),
                finalForceY / RLVectoringStructureMassData.getMass(),
                finalForceZ / RLVectoringStructureMassData.getMass()
        );
        if (linearAcceleration.isNaN()) {
            System.out.println("NAN PARAMETER PRESENT");
        }

        // TODO: Note this is disabled because we disabled roll.  No rotation is required.
        // linearAcceleration = new Rotation2D(RLVectoringFlightConditions.getTheta()).rotateZ(linearAcceleration);

        // Convert into rocket world coordinates
        linearAcceleration = status.getRocketOrientationQuaternion().rotate(linearAcceleration);

        if (linearAcceleration.isNaN()) {
            System.out.println("NAN PARAMETER PRESENT");
        }

        // add effect of gravity
        double gravity = status.getSimulationConditions().getGravityModel().getGravity(status.getRocketWorldPosition());
        linearAcceleration = linearAcceleration.sub(0, 0, gravity);

        if (linearAcceleration.isNaN()) {
            System.out.println("NAN PARAMETER PRESENT");
        }

        // add effect of Coriolis acceleration
        Coordinate coriolisAcceleration = status.getSimulationConditions().getGeodeticComputation()
                .getCoriolisAcceleration(status.getRocketWorldPosition(), status.getRocketVelocity());
        linearAcceleration = linearAcceleration.add(coriolisAcceleration);

        if (linearAcceleration.isNaN()) {
            System.out.println("NAN PARAMETER PRESENT");
        }

        // Shift moments to CG
        double Cm = RLVectoringAerodynamicForces.getCm() - RLVectoringAerodynamicForces.getCN() * RLVectoringStructureMassData.getCM().x / refLength;
        double Cyaw = RLVectoringAerodynamicForces.getCyaw() - RLVectoringAerodynamicForces.getCside() * RLVectoringStructureMassData.getCM().x / refLength;

        double momentArm = status.getConfiguration().getLength() - RLVectoringStructureMassData.cm.x;
        double gimbalMomentX = -momentArm * forceY;
        double gimbalMomentY = momentArm * forceX;

        // Compute moments
//            double momX = -Cyaw * dynP * refArea * refLength + gimbalMomentX;
        double r = status.getFlightConfiguration().getReferenceLength()/2;
        double wx = RLVectoringFlightConditions.getYawRate();
        double wy = RLVectoringFlightConditions.getPitchRate();
//            double wz = RLVectoringFlightConditions.getYawRate();
        double h = status.getConfiguration().getLength();
        double rho = RLVectoringFlightConditions.getAtmosphericConditions().getDensity();
        double Tx = - Math.signum(wx) * Math.PI * Math.pow(wx,2) * Math.pow(r,4) * h * rho * RLVectoringAerodynamicForces.getCyaw();
        double Ty = - Math.signum(wy) * Math.PI * Math.pow(wy,2) * Math.pow(r,4) * h * rho * RLVectoringAerodynamicForces.getCm();
//            double Tz = - Math.signum(wz)*Math.PI*Math.pow(wy,2)*Math.pow(r,4)*h*rho*RLVectoringAerodynamicForces.getCyaw();
        double momX = gimbalMomentX + Tx - 0 * (Math.signum(status.getRocketVelocity().z) * (Cyaw * dynP * refArea * refLength));  // TODO: REMOVE THE ZERO
        double momY = gimbalMomentY + Ty - 0 * (Math.signum(status.getRocketVelocity().z) * (Cm * dynP * refArea * refLength));    // TODO: REMOVE THE ZERO
        double momZ = RLVectoringAerodynamicForces.getCroll() * dynP * refArea * refLength;

        // Compute acceleration in rocket coordinates
        angularAcceleration = new Coordinate(momX / RLVectoringStructureMassData.getLongitudinalInertia(),
                momY / RLVectoringStructureMassData.getLongitudinalInertia(),
                momZ / RLVectoringStructureMassData.getRotationalInertia());

        // System.out.println(angularAcceleration.x + " " + angularAcceleration.y + " " + angularAcceleration.z);

        double rollAcceleration = angularAcceleration.z;
        // TODO: LOW: This should be hypot, but does it matter?
        double lateralPitchAcceleration = MathUtil.max(Math.abs(angularAcceleration.x),
                Math.abs(angularAcceleration.y));

        // Convert to world coordinates
        angularAcceleration = status.getRocketOrientationQuaternion().rotate(angularAcceleration);


        OLD_RLVectoringStructureMassData = RLVectoringStructureMassData;
        RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
        return new AccelerationData(null, null, linearAcceleration, angularAcceleration, status.getRocketOrientationQuaternion());
    }
}

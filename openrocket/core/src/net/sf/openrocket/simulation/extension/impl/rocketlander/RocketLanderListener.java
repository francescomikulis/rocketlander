package net.sf.openrocket.simulation.extension.impl.rocketlander;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.masscalc.RigidBody;
import net.sf.openrocket.simulation.AccelerationData;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.impl.visualize3d.AbstractSimulationListenerSupportsVisualize3DListener;
import net.sf.openrocket.simulation.extension.impl.visualize3d.Visualize3DListener;
import net.sf.openrocket.simulation.listeners.SimulationListener;
import net.sf.openrocket.util.Coordinate;

import net.sf.openrocket.simulation.extension.impl.rocketlander.RLModelSingleton.*;
import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple.*;

import net.sf.openrocket.util.MathUtil;
import net.sf.openrocket.util.Quaternion;

import java.util.*;

import static net.sf.openrocket.simulation.extension.impl.rocketlander.RLModelSingleton.SimulationInitVariation.*;

public class RocketLanderListener extends AbstractSimulationListenerSupportsVisualize3DListener {
    private static final double MIN_VELOCITY = -10;
    private static final double MAX_ALTITUDE = 30;
    private static final double MAX_POSITION = 6;
    private static final double MAX_LAT_VELOCITY = 6;

    private RLModelSingleton model = RLModelSingleton.getInstance();
    private LinkedHashMap<String, ArrayList<StateActionTuple>> episodeStateActions = new LinkedHashMap<>();
    //HashMap<String, ArrayList<Double>> episodeData;
    private RocketLander rocketLander;
    private CoupledStates state;
    private CoupledActions action;
    TerminationBooleans terminationBooleans;
    private LinkedHashMap<String, Integer> lastStepUpdateSizes = new LinkedHashMap<>();
    private static double variation = 2;
    private static double timeStep = 0.02;  // RK4SimulationStepper.MIN_TIME_STEP --> 0.001

    // thrust vectoring
    private FlightConditions RLVectoringFlightConditions = null;
    private AerodynamicForces RLVectoringAerodynamicForces = null;
    private RigidBody RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private RigidBody OLD_RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    // private Double RLVectoringInitialThrust = 0.0;
    private double RLVectoringThrust;

    private boolean hasCompletedTerminalUpdate = false;

    /** Used by the Visualize3DListener extension */
    public double getMaxMotorPower() { return 200.0; }
    public double getLastThrust() { return action.getDouble("thrust"); }
    public double getLastGimbalX() { return action.getDouble("gimbalX"); }
    public double getLastGimbalY()  { return action.getDouble("gimbalY"); }
    public double getLastLateralThrustX()  { return action.getDouble("lateralThrustX"); }
    public double getLastLateralThrustY()  { return action.getDouble("lateralThrustY"); }

    public CoupledActions getLastAction() {
        return action;
    }

    RocketLanderListener(RocketLander rocketLander) {
        this.rocketLander = rocketLander;
    }

    private static double calculateNumberWithIntegerVariation(double startNumber, double variation) {
        return startNumber - (variation) + 2 * variation * new Random().nextDouble();
    }

    private boolean setupStateActionAndStore(SimulationStatus status) {
        CoupledStates oldState = state;

        Object[] stateActionReturnObject = model.generateStateAndActionAndStoreHistory(status, episodeStateActions);
        CoupledStates newState = (CoupledStates) stateActionReturnObject[0];
        CoupledActions newAction = (CoupledActions) stateActionReturnObject[1];

        state = newState;
        action = newAction;

        if (state != null)
            model.dataStoreState.memoizeSmartGuesses(state);

        return true;
    }

    private static Coordinate calculatePositionCoordinate(double maxX, double maxY, double maxZ) {
        double posX = calculateNumberWithIntegerVariation(0, maxX);
        double posY = calculateNumberWithIntegerVariation(0, maxY);
        RLModelSingleton model = RLModelSingleton.getInstance();
        if(model.simulationType == SimulationType._1D) {
            posX = 0; posY = 0;
        }
        if(model.simulationType == SimulationType._2D) {
            if (model.symmetryAxis2D.equals("X")) {
                posY = 0;
            } else if (model.symmetryAxis2D.equals("Y")) {
                posX = 0;
            }
        }
        // dx and dy position only in advanced
        if((model.initVariation == fixed) || (model.initVariation == posVel) || (model.initVariation == posVelAngle)) {
            posX = 0; posY = 0;
        }
        double posZ = calculateNumberWithIntegerVariation(maxZ - variation, variation);
        if((model.initVariation == fixed) || (model.initVariation == loc)) {
            posZ = maxZ;
        }
        return new Coordinate(posX, posY, posZ);
    }

    private static Coordinate calculateVelocityCoordinate(double maxX, double maxY, double maxZ) {
        double velZ = calculateNumberWithIntegerVariation(maxZ - variation, variation);
        RLModelSingleton model = RLModelSingleton.getInstance();
        if((model.initVariation == fixed) || (model.initVariation == loc)) {
            velZ = maxZ;
        }
        double velX = calculateNumberWithIntegerVariation(0, maxX);
        double velY = calculateNumberWithIntegerVariation(0, maxY);
        if(model.simulationType == SimulationType._2D) {
            if (model.symmetryAxis2D.equals("X")) {
                velY = 0;
            } else if (model.symmetryAxis2D.equals("Y")) {
                velX = 0;
            }
        }
        if((model.initVariation != all)) {
            velX = 0; velY = 0;
        }
        return new Coordinate(velX, velY, velZ);
    }

    private static Quaternion calculateInitialOrientation() {
        double dx = calculateNumberWithIntegerVariation(0, variation * 8);  // 3
        double dy = calculateNumberWithIntegerVariation(0, variation * 8);  // 3
        double dz = 90;
        RLModelSingleton model = RLModelSingleton.getInstance();
        if(model.simulationType == SimulationType._1D) {
            dx = 0; dy = 0;
        }
        if(model.simulationType == SimulationType._2D) {
            if (model.symmetryAxis2D.equals("X")) {
                dy = 0;
            } else if (model.symmetryAxis2D.equals("Y")) {
                dx = 0;
            }
        }
        if((model.initVariation == fixed) || (model.initVariation == posVel) || (model.initVariation == loc) || (model.initVariation == posVelLoc)) {
            dx = 0; dy = 0;
        }
        return new Quaternion(0, dx, dy, dz).normalizeIfNecessary();
    }

    private static Coordinate calculateInitialRotationVelocity() {
        double dx = calculateNumberWithIntegerVariation(0, variation * 13) * Math.PI / 180;
        double dy = calculateNumberWithIntegerVariation(0, variation * 13) * Math.PI / 180;
        RLModelSingleton model = RLModelSingleton.getInstance();
        if(model.simulationType == SimulationType._1D) {
            dx = 0; dy = 0;
        }
        if(model.simulationType == SimulationType._2D) {
            if (model.symmetryAxis2D.equals("X")) {
                dx = 0;
            } else if (model.symmetryAxis2D.equals("Y")) {
                dy = 0;
            }
        }
        if((model.initVariation != all)) {
            dx = 0; dy = 0;
        }
        return new Coordinate(dx, dy, 0);
    }

    @Override
    public void startSimulation(SimulationStatus status) {
        model.setupSimulationTypeBasedOnMDPDefinitions(status);
        episodeStateActions = model.initializeEpisodeStateActions();
        status.getSimulationConditions().setTimeStep(timeStep);

        customInitialConditions(status);

        status.setLaunchRodCleared(true);
        // initialize the state and the action
        setupStateActionAndStore(status);
    }

    public static void customInitialConditions(SimulationStatus status) {
        // set the rocket position at the launch altitude as defined by the extension
        status.setRocketPosition(calculatePositionCoordinate(MAX_POSITION, MAX_POSITION, MAX_ALTITUDE));
        // set the rocket velocity at the rocket velocity as defined by the extension
        Coordinate rocketVelocity = calculateVelocityCoordinate(MAX_LAT_VELOCITY, MAX_LAT_VELOCITY, MIN_VELOCITY + variation);
        status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(rocketVelocity));
        status.setRocketOrientationQuaternion(calculateInitialOrientation());
        // status.setRocketOrientationQuaternion(new Quaternion(0, 0, 0, 1));
        // set the rocket rotational velocity
        status.setRocketRotationVelocity(calculateInitialRotationVelocity());
    }

    public void setRollToZero(SimulationStatus status) {
        Coordinate rotVel = status.getRocketRotationVelocity();
        rotVel = rotVel.setZ(0.0);
        status.setRocketRotationVelocity(rotVel);
    }

    public void stabilizeRocketBasedOnSimType(SimulationStatus status) {
        setRollToZero(status);  // prevent rocket from spinning
        if (model.simulationType == SimulationType._1D) {
            status.setRocketOrientationQuaternion(new Quaternion(0, 0, 0, 1)); // set rocket to vertical
            status.setRocketRotationVelocity(new Coordinate(0, 0, 0));
        } else if(model.simulationType == SimulationType._2D) {
            Coordinate pos = status.getRocketPosition();
            Coordinate rotVel = status.getRocketRotationVelocity();
            Coordinate vel = status.getRocketVelocity();
            if (model.symmetryAxis2D.equals("X")) {
                pos = pos.setY(0.0);
                vel = vel.setY(0.0);
                rotVel = rotVel.setX(0.0);
            } else if (model.symmetryAxis2D.equals("Y")) {
                pos = pos.setX(0.0);
                vel = vel.setX(0.0);
                rotVel = rotVel.setY(0.0);
            } else {
                System.out.println("INVALID SYMMETRY AXIS!!!");
            }
            status.setRocketPosition(pos);
            status.setRocketVelocity(vel);
            status.setRocketRotationVelocity(rotVel);

            // force the stabilizer to be trained here!
            // status.setRocketPosition(new Coordinate(0, 0, status.getRocketPosition().z));
        } else if(model.simulationType == SimulationType._3D) {
            // already set the roll to zero
        }
    }

    @Override
    public boolean preStep(SimulationStatus status) {
        stabilizeRocketBasedOnSimType(status);

        // RocketLanderListener integration for gimbal
        List<SimulationListener> listeners = status.getSimulationConditions().getSimulationListenerList();
        for (SimulationListener listener: listeners) {
            if (listener.getClass().toString().contains("Visualize3DListener")) {
                ((Visualize3DListener)listener).setListener(this);
            }
        }
        return true;
    }



    @Override
    public FlightConditions postFlightConditions(SimulationStatus status, FlightConditions flightConditions) {
        if (flightConditions != null) {
            applyCustomFlightConditions(flightConditions);
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
        // currently not used - thrust goes up over the first second
        // RLVectoringInitialThrust = Math.max(thrust, RLVectoringInitialThrust);
        RLVectoringThrust = getMaxMotorPower();
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
        if (action == null) {
            assert state == null;
            return null;
        }
        double thrust = action.getDouble("thrust");
        RLVectoringThrust *= thrust;
        if (model.simulationType == SimulationType._1D) {
            action.setDouble("gimbalX", 0.0);
            action.setDouble("gimbalY", 0.0);
            action.setDouble("lateralThrustX", 0.0);
            action.setDouble("lateralThrustY", 0.0);
        } else if (model.simulationType == SimulationType._2D) {
            if (model.symmetryAxis2D.equals("X")) {
                action.setDouble("gimbalY", 0.0);
                action.setDouble("lateralThrustY", 0.0);
            } else if (model.symmetryAxis2D.equals("Y")) {
                action.setDouble("gimbalX", 0.0);
                action.setDouble("lateralThrustX", 0.0);
            }
        }
        double gimbalX = action.getDouble("gimbalX");
        double gimbalY = action.getDouble("gimbalY");
        double lateralThrustX = action.getDouble("lateralThrustX");
        double lateralThrustY = action.getDouble("lateralThrustY");
        return calculateAcceleration(status, gimbalX, gimbalY, lateralThrustX, lateralThrustY);
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        stabilizeRocketBasedOnSimType(status);
        setupStateActionAndStore(status);
        storeUpdatedFlightConditions();

        model.updateStepStateActionValueFunction(episodeStateActions, lastStepUpdateSizes);
        for (String method: episodeStateActions.keySet()) {
            lastStepUpdateSizes.put(method, episodeStateActions.get(method).size());
        }

        // terminationBooleans.simulationFailed()
        if (status.getSimulationTime() > 7.0) {
            throw new SimulationException("Simulation Was NOT UNDER CONTROL.");
        }

        // conditional end simulation on zero positionZ
        String stateNames = state.toStringNames();
        if (model.dataStoreState.get(stateNames).get("positionZ") != null) {
            int index = (int) model.dataStoreState.get(stateNames).get("positionZ")[0];
            String realField = (String) model.dataStoreState.get(stateNames).get("positionZ")[1];
            if (state.get(index).get(realField) == 0) {
                throw new SimulationException("Simulation completed.  Reached zero positionZ in the state definition.");
            }
        }
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
        if (action != null) {
            terminationBooleans = MDPDefinition.getTerminationValidity(model.generateCoupledStatesBasedOnLastActions(status, action));
            if (!hasCompletedTerminalUpdate) {
                model.updateTerminalStateActionValueFunction(episodeStateActions, terminationBooleans);
                model.printStatusInformationOnSingleSimTermination(status);
                hasCompletedTerminalUpdate = true;
            }
        }
    }

    /** Terminal output **/

    public static void printStatusInformation(SimulationStatus status) {
        System.out.println("Position: " +
                "  x: " + status.getRocketPosition().x +
                "  y: " + status.getRocketPosition().y +
                "  z: " + status.getRocketPosition().z
        );
        System.out.println("Velocity: " +
                "  x: " + status.getRocketVelocity().x +
                "  y: " + status.getRocketVelocity().y +
                "  z: " + status.getRocketVelocity().z
        );
        System.out.println("Angle: " +
                "  x: " + status.getRocketOrientationQuaternion().rotateZ().x +
                "  y: " + status.getRocketOrientationQuaternion().rotateZ().y +
                "  z: " + status.getRocketOrientationQuaternion().rotateZ().z
        );
        System.out.println("Angle Velocity: " +
                "  x: " + status.getRocketRotationVelocity().x +
                "  y: " + status.getRocketRotationVelocity().y +
                "  z: " + status.getRocketRotationVelocity().z
        );
    }





    private AccelerationData calculateAcceleration(SimulationStatus status, Double gimbalX, Double gimbalY, Double lateralThrustX, Double lateralThrustY) {
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
        double forceX = RLVectoringThrust * (gimbalComponentX + lateralThrustX);
        double forceY = RLVectoringThrust * (gimbalComponentY + lateralThrustY);
        double forceZ = RLVectoringThrust * gimbalComponentZ;

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

        assert RLVectoringThrust >= 0;

        // thrust vectoring force
        double rotationalForceX = RLVectoringThrust * gimbalComponentX;
        double rotationalForceY = RLVectoringThrust * gimbalComponentY;

        double momentArm = status.getConfiguration().getLength() - RLVectoringStructureMassData.cm.x;
        double gimbalMomentX = momentArm * -rotationalForceY;
        double gimbalMomentY = -momentArm * -rotationalForceX;

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
                0);
        // AngularAccZ is 0 because no roll.  If not, this should be: momZ / RLVectoringStructureMassData.getRotationalInertia()

        // disable rotation on lateral-only force
        if ((gimbalComponentX == 0) && (gimbalComponentY == 0))
            angularAcceleration = new Coordinate(0, 0, 0);

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

    private void applyCustomFlightConditions(FlightConditions toConditions) {
        if (this.RLVectoringFlightConditions == null) return;
        toConditions.setRLPosition(this.RLVectoringFlightConditions.getRLPosition());
        toConditions.setRLVelocity(this.RLVectoringFlightConditions.getRLVelocity());
        toConditions.setRLAngle(this.RLVectoringFlightConditions.getRLAngle());
        toConditions.setRLAngleVelocity(this.RLVectoringFlightConditions.getRLAngleVelocity());
        toConditions.setRLThrust(this.RLVectoringFlightConditions.getRLThrust());
        toConditions.setRLGimbal(this.RLVectoringFlightConditions.getRLGimbal());
    }

    private void storeUpdatedFlightConditions() {
        if (state == null) return;
        this.RLVectoringFlightConditions.setRLPosition(model.dataStoreState.getSmartGuessCoordinate(state, "position"));
        this.RLVectoringFlightConditions.setRLVelocity(model.dataStoreState.getSmartGuessCoordinate(state, "velocity"));
        this.RLVectoringFlightConditions.setRLAngle(model.dataStoreState.getSmartGuessCoordinate(state, "angle"));
        this.RLVectoringFlightConditions.setRLAngleVelocity(model.dataStoreState.getSmartGuessCoordinate(state, "angleVelocity"));

        // multiplication is for visualization purposes
        double thrustX = action.getDouble("lateralThrustX") * 100;
        double thrustY = action.getDouble("lateralThrustY") * 100;
        double thrustZ = action.getDouble("thrust") * 100;
        this.RLVectoringFlightConditions.setRLThrust(new Coordinate(thrustX, thrustY, thrustZ));

        double gimbalX = action.getDouble("gimbalX");
        double gimbalY = action.getDouble("gimbalY");
        double gimbalZ = Math.sqrt(1.0 - gimbalX * gimbalX - gimbalY * gimbalY);
        this.RLVectoringFlightConditions.setRLGimbal(new Coordinate(gimbalX, gimbalY, gimbalZ));
    }
}

package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.masscalc.RigidBody;
import net.sf.openrocket.simulation.*;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.impl.RLModel.SimulationType;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.CoupledActions;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.CoupledStates;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.State;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.simulation.listeners.SimulationListener;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.GeodeticComputationStrategy;
import net.sf.openrocket.util.MathUtil;
import net.sf.openrocket.util.Quaternion;

import java.nio.ByteBuffer;
import java.util.*;

import static net.sf.openrocket.simulation.extension.impl.RocketLanderListener.customInitialConditions;
import static net.sf.openrocket.simulation.extension.impl.RocketLanderListener.printStatusInformation;
import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.convertRocketStatusQuaternionToDirection;

public class RRTListener extends AbstractSimulationListenerSupportsVisualize3DListener {
    private RRTExtension rrtExtension;
    private Random random;
    private static double timeStep = 0.1;  // RK4SimulationStepper.MIN_TIME_STEP --> 0.001
    // thrust vectoring
    private FlightConditions RLVectoringFlightConditions = null;
    private AerodynamicForces RLVectoringAerodynamicForces = null;
    private RigidBody RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private RigidBody OLD_RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private double RLVectoringThrust;
    // RRT
    private RRT rrt = null;
    private RRT.Action action = null;
    // nodes = SimulationStatus status
    ArrayList<SimulationStatus> ss;
    ArrayList<RRT.Action> aa;
    int currentIndex = 0;
    boolean hasCompletedTerminalUpdate = false;

    boolean isUsingLateralVelocityObjective = true;

    /** Used by the Visualize3DListener extension */
    public double getMaxMotorPower() { return 200.0; }
    public double getLastThrust() { return aa.get(currentIndex).thrust; }
    public double getLastGimbalX() { return aa.get(currentIndex).gimbleX; }
    public double getLastGimbalY()  { return aa.get(currentIndex).gimbleY; }
    public double getLastLateralThrustX()  { return aa.get(currentIndex).lateralThrustX; }
    public double getLastLateralThrustY()  { return aa.get(currentIndex).lateralThrustY; }

    RRTListener(RRTExtension rrtExtension) {
        this.rrtExtension = rrtExtension;
        random = new Random();
    }

    @Override
    public void startSimulation(SimulationStatus status) {
        hasCompletedTerminalUpdate = false;
        // initialize episode
        status.getSimulationConditions().setTimeStep(timeStep);

        customInitialConditions(status);

        status.setLaunchRodCleared(true);
        RRTNode root = new RRTNode(status, null);
        rrt = new RRT(root);

        disableVisualization(status);
    }

    private void disableVisualization(SimulationStatus status) {
        Visualize3DListener visualize3DListener = null;
        List<SimulationListener> listeners = status.getSimulationConditions().getSimulationListenerList();
        for (SimulationListener listener: listeners) {
            if (listener.getClass().toString().contains("Visualize3DListener")) {
                visualize3DListener = (Visualize3DListener) listener;
            }
        }
        if (visualize3DListener == null)  return;
        visualize3DListener.setVisualizeDuringPostStep(false);
    }


    public void setRollToZero(SimulationStatus status) {
        Coordinate rotVel = status.getRocketRotationVelocity();
        rotVel = rotVel.setZ(0.0);
        status.setRocketRotationVelocity(rotVel);
    }

    @Override
    public boolean preStep(SimulationStatus status) {
        disableVisualization(status);
        if (status.getRocketPosition().z != 0)
            action = rrt.setStatus(status);
        setRollToZero(status);
        return true;
    }


    @Override
    public FlightConditions postFlightConditions(SimulationStatus status, FlightConditions flightConditions) {
        if (flightConditions != null) {
            //  applyCustomFlightConditions(flightConditions);
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
        if (RLVectoringFlightConditions == null || action == null) return null;
        RLVectoringThrust *= action.thrust;
        double gimbalX = action.gimbleX;
        double gimbalY = action.gimbleY;
        // return calculateAcceleration(status, gimbalX, gimbalY);
        return NEWcalculateAcceleration(status, gimbalX, gimbalY, action.lateralThrustX, action.lateralThrustY);
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        setRollToZero(status);
        if (action == null) {
            System.out.println("YAY!!");
            status.setRocketPosition(new Coordinate(0, 0, 0));
            throw new SimulationException();
        }
        if (status.getRocketPosition().z < 0) {
            Coordinate c = status.getRocketPosition();
            c = c.setZ(0.001);
            status.setRocketPosition(c);
            //status.setRocketPosition(new Coordinate());
        }
    }

    @Override
    public void endSimulation(SimulationStatus status, SimulationException exception) {
        if (hasCompletedTerminalUpdate) return;

        printStatusInformation(rrt.current.status);
        System.out.println("Number of nodes expanded: " + rrt.numNodesExpanded);

        hasCompletedTerminalUpdate = true;
        RRTNode n;
        if (action == null) {
            n = rrt.current;
        } else {
            n = rrt.minNode;
        }
        ss = new ArrayList<>();
        aa = new ArrayList<>();
        while (n.parent != null) {
            aa.add(n.action);
            n = n.parent;
            ss.add(n.status);
        }
        Visualize3DListener visualize3DListener = null;
        List<SimulationListener> listeners = status.getSimulationConditions().getSimulationListenerList();
        for (SimulationListener listener : listeners) {
            if (listener.getClass().toString().contains("Visualize3DListener")) {
                visualize3DListener = (Visualize3DListener) listener;
            }
        }
        if (visualize3DListener == null) return;

        visualize3DListener.setListener(this);
        visualize3DListener.setVisualizeDuringPostStep(true);
        for (currentIndex = ss.size() - 1; currentIndex >= 0; currentIndex--) {
            visualize3DListener.postStep(ss.get(currentIndex));
        }
        visualize3DListener.closeClient();
    }


    private byte[] serialize_single_timeStep(SimulationStatus status, RRT.Action action) {
        byte[] bytes = new byte[40];
        int offset = 0;
        offset = arrayAdd(bytes, status.getRocketPosition().x, offset);
        offset = arrayAdd(bytes, status.getRocketPosition().y, offset);
        offset = arrayAdd(bytes, status.getRocketPosition().z, offset);
        offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getW(), offset);
        offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getX(), offset);
        offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getY(), offset);
        offset = arrayAdd(bytes, status.getRocketOrientationQuaternion().getZ(), offset);
        // thrust may not work
        double actualMotorThrust = 0.0;
        try {
            actualMotorThrust = status.getActiveMotors().iterator().next().getThrust(status.getSimulationTime());
        } catch (Exception e) {
        }
        offset = arrayAdd(bytes, actualMotorThrust * action.thrust, offset);
        // gimbal angles not yet present in the simulationStatus
        offset = arrayAdd(bytes, action.gimbleX, offset);
        offset = arrayAdd(bytes, action.gimbleY, offset);
        return bytes;
    }

    private static byte[] getFloatBytes(double value) {
        byte[] floatByte = new byte[4];
        ByteBuffer.wrap(floatByte).putFloat((float) value);
        return floatByte;
    }

    private static int arrayAdd(byte[] bytes, double value, int offset) {
        byte[] floatBytes = getFloatBytes(value);
        int length = 4;
        System.arraycopy(floatBytes, 0, bytes, offset, length);
        return offset + length;
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
        //  System.out.println(RLVectoringThrust);
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
        double r = status.getFlightConfiguration().getReferenceLength() / 2;
        double wx = RLVectoringFlightConditions.getYawRate();
        double wy = RLVectoringFlightConditions.getPitchRate();
//            double wz = RLVectoringFlightConditions.getYawRate();
        double h = status.getConfiguration().getLength();
        double rho = RLVectoringFlightConditions.getAtmosphericConditions().getDensity();
        double Tx = -Math.signum(wx) * Math.PI * Math.pow(wx, 2) * Math.pow(r, 4) * h * rho * RLVectoringAerodynamicForces.getCyaw();
        double Ty = -Math.signum(wy) * Math.PI * Math.pow(wy, 2) * Math.pow(r, 4) * h * rho * RLVectoringAerodynamicForces.getCm();
//            double Tz = - Math.signum(wz)*Math.PI*Math.pow(wy,2)*Math.pow(r,4)*h*rho*RLVectoringAerodynamicForces.getCyaw();
        double momX = gimbalMomentX + Tx - 0 * (Math.signum(status.getRocketVelocity().z) * (Cyaw * dynP * refArea * refLength));  // TODO: REMOVE THE ZERO
        double momY = gimbalMomentY + Ty - 0 * (Math.signum(status.getRocketVelocity().z) * (Cm * dynP * refArea * refLength));    // TODO: REMOVE THE ZERO
        double momZ = RLVectoringAerodynamicForces.getCroll() * dynP * refArea * refLength;

        // Compute acceleration in rocket coordinates
        angularAcceleration = new Coordinate(momX / RLVectoringStructureMassData.getLongitudinalInertia(),
                momY / RLVectoringStructureMassData.getLongitudinalInertia(),
                0);
        // AngularAccZ is 0 because no roll.  If not, this should be: momZ / RLVectoringStructureMassData.getRotationalInertia()

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



    /** new method that allows for lateral thrust **/
    private AccelerationData NEWcalculateAcceleration(SimulationStatus status, Double gimbalX, Double gimbalY, Double lateralThrustX, Double lateralThrustY) {
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
}


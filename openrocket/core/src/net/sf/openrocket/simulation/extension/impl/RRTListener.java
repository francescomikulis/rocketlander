package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.aerodynamics.AerodynamicForces;
import net.sf.openrocket.aerodynamics.FlightConditions;
import net.sf.openrocket.masscalc.RigidBody;
import net.sf.openrocket.simulation.AccelerationData;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.extension.impl.RLModel.SimulationType;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.CoupledActions;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.CoupledStates;
import net.sf.openrocket.simulation.extension.impl.StateActionTuple.State;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.MathUtil;
import net.sf.openrocket.util.Quaternion;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Random;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.convertRocketStatusQuaternionToDirection;

public class RRTListener extends AbstractSimulationListener {
    private static final double MIN_VELOCITY = -10;
    private static final double MAX_ALTITUDE = 30;
    private static final double MAX_POSITION = 10;
    private RRTExtension rrtExtension;
    private Random random;
    private static double variation = 2;
    private static double timeStep = 0.01;  // RK4SimulationStepper.MIN_TIME_STEP --> 0.001
    // thrust vectoring
    private FlightConditions RLVectoringFlightConditions = null;
    private AerodynamicForces RLVectoringAerodynamicForces = null;
    private RigidBody RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private RigidBody OLD_RLVectoringStructureMassData = new RigidBody(new Coordinate(0, 0, 0), 0, 0, 0);
    private double RLVectoringThrust;


    RRTListener(RRTExtension rrtExtension) {
        this.rrtExtension = rrtExtension;
        random = new Random();
    }

    private double calculateNumberWithIntegerVariation(double startNumber, double variation) {
        return startNumber - (variation) + 2 * variation * random.nextDouble();
    }

    @Override
    public void startSimulation(SimulationStatus status) {
        // initialize episode
        status.getSimulationConditions().setTimeStep(timeStep);
        double posX = calculateNumberWithIntegerVariation(0, MAX_POSITION);
        double posY = calculateNumberWithIntegerVariation(0, MAX_POSITION);
        double posZ = calculateNumberWithIntegerVariation(MAX_ALTITUDE - variation, variation);
        // set the rocket position at the launch altitude as defined by the extension
        status.setRocketPosition(new Coordinate(posX, posY, posZ));
        // set the rocket velocity at the rocket velocity as defined by the extension
        status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(MIN_VELOCITY + variation, variation))));
        //status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(rocketLander.getLaunchVelocity(), variation))));
        double dx = calculateNumberWithIntegerVariation(0, variation * 2);  // 3
        double dy = calculateNumberWithIntegerVariation(0, variation * 2);  // 3
        double dz = 90;
        status.setRocketOrientationQuaternion(new Quaternion(0, dx, dy, dz).normalizeIfNecessary());
        dx = calculateNumberWithIntegerVariation(0, variation * 2) * Math.PI / 180;
        dy = calculateNumberWithIntegerVariation(0, variation * 2) * Math.PI / 180;
        status.setRocketRotationVelocity(new Coordinate(dx, dy, 0));
        status.setLaunchRodCleared(true);
    }


    public void setRollToZero(SimulationStatus status) {
        Coordinate rotVel = status.getRocketRotationVelocity();
        rotVel = rotVel.setZ(0.0);
        status.setRocketRotationVelocity(rotVel);
    }

    @Override
    public boolean preStep(SimulationStatus status) {
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
        RLVectoringThrust *= 1;
        double gimbalX = 0;
        double gimbalY = 0;
        return calculateAcceleration(status, gimbalX, gimbalY);
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        setRollToZero(status);
    }

    @Override
    public void endSimulation(SimulationStatus status, SimulationException exception) {

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


}

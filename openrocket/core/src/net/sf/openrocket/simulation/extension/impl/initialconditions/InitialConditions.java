package net.sf.openrocket.simulation.extension.impl.initialconditions;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import net.sf.openrocket.simulation.FlightDataBranch;
import net.sf.openrocket.simulation.FlightDataType;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.rocketlander.RLModelSingleton;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;

import java.io.Serializable;
import java.util.Random;

public class InitialConditions implements Serializable {
    private static Gson gson = new GsonBuilder().setPrettyPrinting().create();

    public int numDimensions = 3;
    public String symmetryAxis2D = "X";
    public double[] positionX = new double[]{0.0, 0.0};
    public double[] positionY = new double[]{0.0, 0.0};
    public double[] positionZ = new double[]{0.0, 0.0};
    public double[] velocityX = new double[]{0.0, 0.0};
    public double[] velocityY = new double[]{0.0, 0.0};
    public double[] velocityZ = new double[]{0.0, 0.0};
    public double[] angleX = new double[]{0.0, 0.0};
    public double[] angleY = new double[]{0.0, 0.0};
    public double[] angleVelocityX = new double[]{0.0, 0.0};
    public double[] angleVelocityY = new double[]{0.0, 0.0};

    public static InitialConditions buildFromJsonString(String jsonString) {
        InitialConditions initialConditions = gson.fromJson(jsonString, InitialConditions.class);
        initialConditions.postConstructor();
        return initialConditions;
    }

    public static String toJsonString(InitialConditions initialConditions) {
        return gson.toJson(initialConditions, InitialConditions.class);
    }

    public void postConstructor() {
        convertInitialConditionAnglesToRadians();
    }

    private void convertInitialConditionAnglesToRadians() {
        _convertInitialConditionAnglesToRadians(angleX);
        _convertInitialConditionAnglesToRadians(angleY);
        _convertInitialConditionAnglesToRadians(angleVelocityX);
        _convertInitialConditionAnglesToRadians(angleVelocityY);
    }

    private static void _convertInitialConditionAnglesToRadians(double[] minMaxField) {
        if (Math.abs(minMaxField[0]) > 1.0) {
            minMaxField[0] *= Math.PI / 180;
        }
        if (Math.abs(minMaxField[1]) > 1.0) {
            minMaxField[1] *= Math.PI / 180;
        }
    }

    /**
     * Apply the initial condition parameters to a status.
     * @param status
     */

    public void applyInitialConditionsToStatus(SimulationStatus status) {
        // set the rocket position at the launch altitude as defined by the extension
        status.setRocketPosition(calculatePositionCoordinate());
        // set the rocket velocity at the rocket velocity as defined by the extension
        Coordinate rocketVelocity = calculateVelocityCoordinate();
        status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(rocketVelocity));
        status.setRocketOrientationQuaternion(calculateInitialOrientation());
        // status.setRocketOrientationQuaternion(new Quaternion(0, 0, 0, 1));
        // set the rocket rotational velocity
        status.setRocketRotationVelocity(calculateInitialRotationVelocity());
    }

    private static double calculateNumberInRange(double[] minMax) {
        return minMax[0] + new Random().nextDouble() * (minMax[1] - minMax[0]);
    }

    private Coordinate calculatePositionCoordinate() {
        double posX = calculateNumberInRange(positionX);
        double posY = calculateNumberInRange(positionY);
        if(numDimensions == 1) { posX = 0; posY = 0; }
        if(numDimensions == 2) {
            if (symmetryAxis2D.equals("X")) posY = 0;
            else if (symmetryAxis2D.equals("Y")) posX = 0;
        }
        double posZ = calculateNumberInRange(positionZ);
        return new Coordinate(posX, posY, posZ);
    }

    private Coordinate calculateVelocityCoordinate() {
        double velX = calculateNumberInRange(velocityX);
        double velY = calculateNumberInRange(velocityY);
        if(numDimensions == 1) { velX = 0; velY = 0; }
        if(numDimensions == 2) {
            if (symmetryAxis2D.equals("X")) velY = 0;
            else if (symmetryAxis2D.equals("Y")) velX = 0;
        }
        double velZ = calculateNumberInRange(velocityZ);
        return new Coordinate(velX, velY, velZ);
    }

    private Quaternion calculateInitialOrientation() {
        double alphaX = calculateNumberInRange(angleX);
        double alphaY = calculateNumberInRange(angleY);
        if(numDimensions == 1) { alphaX = 0; alphaY = 0; }
        if(numDimensions == 2) {
            if (symmetryAxis2D.equals("X")) alphaY = 0;
            else if (symmetryAxis2D.equals("Y")) alphaX = 0;
        }
        double alphaZ = Math.PI / 2;  // normalized at 90 deg
        return new Quaternion(0, alphaX, alphaY, alphaZ).normalizeIfNecessary();
    }

    private Coordinate calculateInitialRotationVelocity() {
        double alphaVelX = calculateNumberInRange(angleVelocityX);
        double alphaVelY = calculateNumberInRange(angleVelocityY);
        if(numDimensions == 1) { alphaVelX = 0; alphaVelY = 0; }
        if(numDimensions == 2) {
            if (symmetryAxis2D.equals("X")) alphaVelX = 0;
            else if (symmetryAxis2D.equals("Y")) alphaVelY = 0;
        }
        double alphaVelZ = 0;
        return new Coordinate(alphaVelX, alphaVelY, alphaVelZ);
    }


    public void setRollToZero(SimulationStatus status) {
        Coordinate rotVel = status.getRocketRotationVelocity();
        rotVel = rotVel.setZ(0.0);
        status.setRocketRotationVelocity(rotVel);
    }

    public void stabilizeRocketBasedOnSimType(SimulationStatus status) {
        setRollToZero(status);  // prevent rocket from spinning
        if (numDimensions == 1) {
            status.setRocketOrientationQuaternion(new Quaternion(0, 0, 0, 1)); // set rocket to vertical
            status.setRocketRotationVelocity(new Coordinate(0, 0, 0));
        } else if(numDimensions == 2) {
            Coordinate pos = status.getRocketPosition();
            Coordinate rotVel = status.getRocketRotationVelocity();
            Coordinate vel = status.getRocketVelocity();
            if (symmetryAxis2D.equals("X")) {
                pos = pos.setY(0.0);
                vel = vel.setY(0.0);
                rotVel = rotVel.setX(0.0);
            } else if (symmetryAxis2D.equals("Y")) {
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
        } else if(numDimensions == 3) {
            // already set the roll to zero
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

    /** Distable SimulationStatus data for plotting - useful when running thousands of simulations! **/
    public static void clearExtraStatusFlightData(SimulationStatus status) {
        status.setFlightData(new FlightDataBranch("GARBAGE", FlightDataType.TYPE_TIME));
    }
}

package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.AccelerationData;
import net.sf.openrocket.simulation.RK4SimulationStepper;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.startup.Application;
import net.sf.openrocket.startup.Preferences;
import net.sf.openrocket.util.Coordinate;

import net.sf.openrocket.simulation.extension.impl.RLModel.*;
import net.sf.openrocket.util.Quaternion;

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

        // set the rocket position at the launch altitude as defined by the extension
        //status.setRocketPosition(new Coordinate(0, 0, calculateNumberWithIntegerVariation(rocketLander.getLaunchAltitude(), variation)));
        status.setRocketPosition(new Coordinate(0, 0, calculateNumberWithIntegerVariation(100, variation)));
        // set the rocket velocity at the rocket velocity as defined by the extension
        //status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(rocketLander.getLaunchVelocity(), variation))));
        status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, calculateNumberWithIntegerVariation(-40, variation))));
        // set the simulation timeStep
        status.getSimulationConditions().setTimeStep(timeStep);
    }

    @Override
    public double preSimpleThrustCalculation(SimulationStatus status) throws SimulationException {
        // note we would want to also fix the fuel.  This ignores the fuel level of the rocket.

        // status.getActiveMotors().iterator().next().getThrust(status.getSimulationTime());
        //status.getRocketVelocity();
        //return 0.0;

        double MAX_THRUST = 150;

        RLModel.Action action = model.run_policy(status, episodeStateActions);
        // return Double.NaN;
        //if (status.getSimulationTime() < 0.1) {
        //    return MAX_THRUST;
        //} else {
            return MAX_THRUST * action.thrust;
        //}
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
}

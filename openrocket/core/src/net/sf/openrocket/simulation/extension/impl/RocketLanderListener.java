package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.exception.SimulationException;
import net.sf.openrocket.simulation.listeners.AbstractSimulationListener;
import net.sf.openrocket.util.Coordinate;

import net.sf.openrocket.simulation.extension.impl.RLModel.*;

import java.util.ArrayList;

public class RocketLanderListener extends AbstractSimulationListener {
    private static RLEpisodeManager episodeManager = RLEpisodeManager.getInstance();
    private static RLModel model = RLModel.getInstance();
    private ArrayList<StateActionTuple> episodeStateActions;
    //HashMap<String, ArrayList<Double>> episodeData;
    private RocketLander rocketLander;

    RocketLanderListener(RocketLander rocketLander) {
        this.rocketLander = rocketLander;
    }

    @Override
    public void startSimulation(SimulationStatus status) throws SimulationException {
        //episodeData = episodeManager.initializeEmptyEpisode();
        episodeStateActions = episodeManager.initializeEmptyActionStateTuples();

        episodeManager.setupParameters(status);

        // set the rocket position at the launch altitude as defined by the extension
        status.setRocketPosition(new Coordinate(0, 0, rocketLander.getLaunchAltitude()));
        // set the rocket velocity at the rocket velocity as defined by the extension
        status.setRocketVelocity(status.getRocketOrientationQuaternion().rotate(new Coordinate(0, 0, rocketLander.getLaunchVelocity())));

        System.out.println("CALLED START SIMULATION");
    }

    @Override
    public double preSimpleThrustCalculation(SimulationStatus status) throws SimulationException {
        // note we would want to also fix the fuel.  This ignores the fuel level of the rocket.

        // status.getActiveMotors().iterator().next().getThrust(status.getSimulationTime());
        //status.getRocketVelocity();
        //return 0.0;

        RLModel.Action action = model.run_policy(status, episodeStateActions);
        if (action.thrust == 0.0) {
            // turn off
            return 0;
        } else {
            // leave on
            return Double.NaN;
        }
    }

    @Override
    public boolean preStep(SimulationStatus status) throws SimulationException {
        return true;
    }

    @Override
    public void postStep(SimulationStatus status) throws SimulationException {
        //status.getFlightData();

        // ignore adding the data for now
        // episodeManager.addData(status, episodeData);
    }
    @Override
    public void endSimulation(SimulationStatus status, SimulationException exception) {
        // episodeManager.addEpisode(episodeData);
        model.updateStateActionValueFuncton(episodeStateActions);
        episodeManager.storeActionValueFunction();
        System.out.println("Numbers of iterations: " + episodeStateActions.size() + " " + episodeStateActions.get(episodeStateActions.size()-1).state.velocity);
    }
}

package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Quaternion;

import java.io.Serializable;
import java.util.*;
import java.util.concurrent.Semaphore;
import java.util.function.BiFunction;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;
    private HashMap<StateActionTuple, Double> valueFunctionTable = null;
    private static boolean ONLY_GREEDY = true;

    private static double MIN_MOTOR_INCREMENT_PER_TIMESTEP = 0.05;
    private static double MAX_MOTOR_INCREMENT_PER_TIMESTEP = 0.25;

    private static double MIN_ANGLE_INCREMENT_PER_TIMESTEP = Math.PI / 36;
    private static double MAX_ANGLE_INCREMENT_PER_TIMESTEP = Math.PI / 18;
    private static double MAX_ANGLE = MAX_ANGLE_INCREMENT_PER_TIMESTEP * 4;

    private static double THRUST_TOGGLE_PENALTY = 0.1;
    private static double THRUST_ON_PENALTY = -5;

    private Semaphore mutex = new Semaphore(1);

    private RLMethod currentMethod = RLMethod.MONTE;

    enum RLMethod {
        MONTE, TD0, SARSA
    }

    private static class InstanceHolder {
        private static final RLModel instance = new RLModel();
    }

    public static RLModel getInstance() {
        return InstanceHolder.instance;
    }

    private RLModel(){}

    public void initializeModel() {
        episodeManager = RLEpisodeManager.getInstance();
        episodeManager.safeActionValueFunctionInitialization();
    }

    private ArrayList<Double> generatePossibleActionValues(double value, double[] actionIncrements, double MIN_VAL, double MAX_VAL) {
        ArrayList<Double> possibleActionValues = new ArrayList<>();

        int number_of_loops = (int) (2 * actionIncrements[1] / actionIncrements[0]) + 1;
        double smallest_increment = actionIncrements[0];
        double starting_low_value =  value - actionIncrements[1];
        for (int i = 0; i < number_of_loops; i++) {
            double possibleValue = starting_low_value + i * smallest_increment;
            if ((possibleValue >= MIN_VAL) && (possibleValue <= MAX_VAL))
                possibleActionValues.add(possibleValue);
        }

        return possibleActionValues;
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/
    public ArrayList<Action> generatePossibleActions(State state, double prevThurst) {
        ArrayList<Action> possibleActions = new ArrayList<>();

        double [] thrustChanges = new double[] {
                MIN_MOTOR_INCREMENT_PER_TIMESTEP, MAX_MOTOR_INCREMENT_PER_TIMESTEP
        };
        ArrayList<Double> possibleThrustValues = generatePossibleActionValues(prevThurst, thrustChanges, 0.0, 1.0);

        double [] gimbleChanges = new double[] {
                MIN_ANGLE_INCREMENT_PER_TIMESTEP, MAX_ANGLE_INCREMENT_PER_TIMESTEP
        };
        ArrayList<Double> possibleGimbleYValues = generatePossibleActionValues(state.getGimbleInRadians(state.gimbleY), gimbleChanges, - MAX_ANGLE, MAX_ANGLE);
        ArrayList<Double> possibleGimbleZValues = generatePossibleActionValues(state.getGimbleInRadians(state.gimbleZ), gimbleChanges, - MAX_ANGLE, MAX_ANGLE);
        for (Double possibleThrust: possibleThrustValues) {
            for (Double possibleGimbleY: possibleGimbleYValues) {
                for (Double possibleGimbleZ: possibleGimbleZValues) {
                    possibleActions.add(new Action(possibleThrust, possibleGimbleY, possibleGimbleZ));
                }
            }
        }

        return possibleActions;
    }


    private Double valueFunction(StateActionTuple stateActionTuple) {
        Action action = stateActionTuple.action;
        if (action.thrust > 0) {
            if (stateActionTuple.state.velocity > 0) {
                double penalty = THRUST_ON_PENALTY * action.thrust * 10;
                if (!valueFunctionTable.containsKey(stateActionTuple))
                    return penalty; // penalty
                else
                    return valueFunctionTable.get(stateActionTuple) - Math.abs(penalty);
            }
        }

        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0;
        return valueFunctionTable.get(stateActionTuple);
    }

    private Double valueFunction(State state, Action action) {
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        return valueFunction(stateActionTuple);
    }


    private Action run_policy(State state, double prevThrust) {
        Action action = policy(state, prevThrust, this::valueFunction);
        return action;
    }

    public Action generateAction(SimulationStatus status, ArrayList<StateActionTuple> episodeStateAction) {
        State state = new State(status);
        double prevThrust = 0.0;
        if (episodeStateAction.size() > 0) {
            StateActionTuple lastStateAction = episodeStateAction.get(episodeStateAction.size() - 1);
            state.setGimbleYWithoutRounding(lastStateAction.action.gimbleY);
            state.setGimbleZWithoutRounding(lastStateAction.action.gimbleZ);
            prevThrust = lastStateAction.action.thrust;
        }
        Action action = run_policy(state, prevThrust);
        episodeStateAction.add(new StateActionTuple(state, action));
        return action;
    }

    private Action policy(State state, double prevThurst, BiFunction<State, Action, Double> func) {
        ArrayList<Action> possibleActions = generatePossibleActions(state, prevThurst);

        double val = Double.NEGATIVE_INFINITY;
        ArrayList<Action> bestActions = new ArrayList<>();

        boolean greedy = ONLY_GREEDY;
        if (randomGenerator.nextDouble() <= 0.05) {
            greedy = false;  // false
        }

        for (Action action: possibleActions) {
            double v = func.apply(state, action);
            if (greedy) {
                if (v > val) {
                    // value is best compared to all previous encounters.  Reset bestAction ArrayList.
                    val = v;
                    bestActions = new ArrayList<>();
                    bestActions.add(action);
                } else if (v == val) {
                    // value is equal to other best value.  Add to bestAction ArrayList.
                    bestActions.add(action);
                }
            } else {
                bestActions.add(action);
            }
        }
        // ties broken completely at random
        return bestActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        try {
            mutex.acquire();
            actuallyUpdateStateActionValueFuncton(stateActionTuples);
        } catch (InterruptedException e) {
            // exception handling code
        } finally {
            mutex.release();
        }
    }

    private double terminalReward(State lastState) {
        double lowSpeedLandingBonus = 0.0;
        if (Math.abs(lastState.velocity) <= 2.0) {
            lowSpeedLandingBonus = 50;
            if (Math.abs(lastState.velocity) <= 1.0)
                lowSpeedLandingBonus = 100;
        }

        double altitudeMultiplier = lastState.altitude;
        if (altitudeMultiplier < 1) {
            altitudeMultiplier = 1;
        } else {
            lowSpeedLandingBonus = 0;
        }

        double landingVelocity = Math.abs(lastState.velocity);
        // this edge case will tell the system it was doing well because the multiplication wasn't penalizing enough
        if ((landingVelocity < 100) && (altitudeMultiplier != 1)) {
            landingVelocity = 100;
        }

        return -(landingVelocity * altitudeMultiplier) - altitudeMultiplier + lowSpeedLandingBonus;  // landing velocity
    }

    private double reward(StateActionTuple stateActionTuple) {
        double reward = 0.0;

        reward = - Math.abs(stateActionTuple.state.velocity) / 10;
        return reward;
    }

    private void actuallyUpdateStateActionValueFuncton(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);

        double G = terminalReward(lastStateActionTuple.state);
        double discount = 0.999;
        double alpha = 0.3;

        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            double originalValue = valueFunction(stateActionTuple);
            valueFunctionTable.put(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (discount * G) - reward(stateActionTuple);
        }
    }

    public void updateTD0ValueFunction(ArrayList<StateActionTuple> episodeStateActions) {
        try {
            mutex.acquire();
            actuallyUpdateTD0ValueFunction(episodeStateActions);
        } catch (InterruptedException e) {
            // exception handling code
        } finally {
            mutex.release();
        }
    }

    private void actuallyUpdateTD0ValueFunction(ArrayList<StateActionTuple> episodeStateActions) {
        StateActionTuple old = episodeStateActions.get(episodeStateActions.size() - 2);
        StateActionTuple current = episodeStateActions.get(episodeStateActions.size() - 1);
        double alpha = .05, gamma = .95;

        valueFunctionTable.put(old, valueFunctionTable.get(old) +
                alpha * (reward(current) + gamma * valueFunctionTable.get(current) - valueFunctionTable.get(old)));
    }

    public void resetValueFunctionTable() {
        valueFunctionTable = new HashMap<>();
    }

    public HashMap<StateActionTuple, Double> getValueFunctionTable() {
        return valueFunctionTable;
    }

    public void setValueFunctionTable(HashMap<StateActionTuple, Double> valueFunctionTable) {
        this.valueFunctionTable = valueFunctionTable;
    }

    public RLMethod getCurrentMethod() {
        return currentMethod;
    }

    public void setCurrentMethod(RLMethod method) {
        currentMethod = method;
    }

    @FunctionalInterface
    interface TriFunction<A,B,C,R> {

        R apply(A a, B b, C c);

        default <V> TriFunction<A, B, C, V> andThen(
                Function<? super R, ? extends V> after) {
            Objects.requireNonNull(after);
            return (A a, B b, C c) -> after.apply(apply(a, b, c));
        }
    }
}





package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.util.Quaternion;

import java.io.Serializable;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Semaphore;
import java.util.function.BiFunction;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;
    private ConcurrentHashMap<StateActionTuple, Double> valueFunctionTable = null;
    private static boolean ONLY_GREEDY = true;

    private static double MIN_MOTOR_INCREMENT_PER_TIMESTEP = 25;
    private static double MAX_MOTOR_INCREMENT_PER_TIMESTEP = 100;

    private static double MIN_ANGLE_INCREMENT_PER_TIMESTEP = Math.PI / 36;
    private static double MAX_ANGLE_INCREMENT_PER_TIMESTEP = Math.PI / 18;
    private static double MAX_ANGLE = MAX_ANGLE_INCREMENT_PER_TIMESTEP * 4;

    private static double THRUST_TOGGLE_PENALTY = 0.1;
    private static double THRUST_ON_PENALTY = -5;

    private Semaphore mutex = new Semaphore(1);

    private RLMethod currentMethod = RLMethod.TD0;

    enum RLMethod {
        MONTE, TD0, SEMI_SARSA
    }

    private static volatile RLModel instance;

    private RLModel(){}

    public static RLModel getInstance() {
        if (instance == null) { // first time lock
            synchronized (RLModel.class) {
                if (instance == null) {  // second time lock
                    instance = new RLModel();
                }
            }
        }
        return instance;
    }


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
    public ArrayList<Action> generatePossibleActions(State state) {
        double currentThrust = state.thrust;
        ArrayList<Action> possibleActions = new ArrayList<>();

        double [] thrustChanges = new double[] {
                MIN_MOTOR_INCREMENT_PER_TIMESTEP, MAX_MOTOR_INCREMENT_PER_TIMESTEP
        };
        ArrayList<Double> possibleThrustValues = generatePossibleActionValues(currentThrust, thrustChanges, 0.0, 100);

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

    private Double actuallySemiSarsaUpdateStateActionValueFunction(ArrayList<StateActionTuple> episodeStateActions) {
        double[] weight = new double[]{1.0,1.0};
        double discount = 0.999;
        double alpha = 0.3;

        int lastTimeStep = episodeStateActions.size() - 1;
        StateActionTuple currentStateAction = episodeStateActions.get(lastTimeStep);
        double rew = reward(currentStateAction.state);

        double[] evaluation = new double[]{0.0,0.0};
        evaluation[0] = discount * (weight[0] * currentStateAction.action.thrust);
        evaluation[1] = discount * (weight[1] * currentStateAction.state.velocity);


        StateActionTuple lastStateActionTuple = episodeStateActions.get(lastTimeStep - 1);

        double[] previous = new double[]{0.0,0.0};
        previous[0] = weight[0] * lastStateActionTuple.action.thrust;
        previous[1] = weight[1] * lastStateActionTuple.state.velocity;

        double[] gradient = new double[]{1.0,1.0};
        gradient[0] = lastStateActionTuple.action.thrust;
        gradient[1] = lastStateActionTuple.state.velocity;

        weight[0] = weight[0] + alpha * (rew + evaluation[0] - previous[0]) * gradient[0];
        weight[1] = weight[1] + alpha * (rew + evaluation[1] - previous[1]) * gradient[1];


        //return valueFunction(episodeStateActions, stateActionTuple);
        return 0.0;
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


    private Action run_policy(State state) {
        Action action = policy(state, this::valueFunction);
        return action;
    }

    public Action generateAction(State state) {
        return run_policy(state);
    }

    private Action policy(State state, BiFunction<State, Action, Double> func) {
        ArrayList<Action> possibleActions = generatePossibleActions(state);

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

    public void updateStepStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        try {
            mutex.acquire();

            if(currentMethod == RLMethod.SEMI_SARSA)
                actuallySemiSarsaUpdateStateActionValueFunction(stateActionTuples);
            else if(currentMethod == RLMethod.TD0)
                actuallyTD0UpdateStateActionValueFunction(stateActionTuples, this::reward);

            // actuallyMonteCarloUpdateStateActionValueFunction(stateActionTuples);


        } catch (InterruptedException e) {
            // exception handling code
        } finally {
            mutex.release();
        }
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
       //try {
       //     mutex.acquire();

            if(currentMethod == RLMethod.MONTE)
                actuallyMonteCarloUpdateStateActionValueFunction(stateActionTuples);
            else if(currentMethod == RLMethod.TD0)
                actuallyTD0UpdateStateActionValueFunction(stateActionTuples, this::terminalReward);

        //} catch (InterruptedException e) {
            // exception handling code
        //} finally {
        //    mutex.release();
        //}
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

    private double reward(State state) {
        double reward = 0.0;

        reward = - Math.abs(state.velocity) / 10;
        return reward;
    }

    private void actuallyMonteCarloUpdateStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);

        double G = terminalReward(lastStateActionTuple.state);
        double discount = 0.999;
        double alpha = 0.3;

        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            double originalValue = valueFunction(stateActionTuple);
            valueFunctionTable.put(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (discount * G) - reward(stateActionTuple.state);
        }
    }

    private void actuallyTD0UpdateStateActionValueFunction(ArrayList<StateActionTuple> episodeStateActions, Function<State, Double> rewardFunction) {
        if(episodeStateActions.size() <= 2) { return; }

        StateActionTuple old = episodeStateActions.get(episodeStateActions.size() - 2);
        StateActionTuple current = episodeStateActions.get(episodeStateActions.size() - 1);
        double alpha = .3, gamma = .999;

        if (!valueFunctionTable.containsKey(old)) valueFunctionTable.put(old, 0.0);
        if (!valueFunctionTable.containsKey(current)) valueFunctionTable.put(current, 0.0);

        valueFunctionTable.put(old, valueFunctionTable.get(old) +
                alpha * (rewardFunction.apply(current.state) + gamma * valueFunctionTable.get(current) - valueFunctionTable.get(old)));
    }

    public void resetValueFunctionTable() {
        valueFunctionTable = new ConcurrentHashMap<>();
    }

    public ConcurrentHashMap<StateActionTuple, Double> getValueFunctionTable() {
        return valueFunctionTable;
    }

    public void setValueFunctionTable(ConcurrentHashMap<StateActionTuple, Double> valueFunctionTable) {
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





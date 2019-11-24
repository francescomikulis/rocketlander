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

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;
    private OptimizedMap valueFunctionTable = null;
    private static boolean ONLY_GREEDY = true;

    private static float THRUST_ON_PENALTY = -5;

    private RLMethod currentMethod = RLMethod.MONTE;
    public SimulationType simulationType = SimulationType._3D;

    enum RLMethod {
        MONTE, TD0, SEMI_SARSA
    }
    enum SimulationType {
        _1D, _2D, _3D
    }

    float discount = 0.99f;
    float alpha = 0.3f;

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

    private ArrayList<Float> generatePossibleActionValues(float value, float smallest_increment, float max_increment, float MIN_VAL, float MAX_VAL) {
        ArrayList<Float> possibleActionValues = new ArrayList<>();

        int number_of_loops = (int) (2.0 * max_increment / smallest_increment) + 1;
        float starting_low_value =  value - max_increment;
        for (int i = 0; i < number_of_loops; i++) {
            float possibleValue = starting_low_value + i * smallest_increment;
            if ((possibleValue >= MIN_VAL) && (possibleValue <= MAX_VAL)) {
                possibleActionValues.add(possibleValue);
            }
        }

        return possibleActionValues;
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/
    public ArrayList<Action> generatePossibleActions(State state) {
        float currentThrust = (float)state.getThrustDouble();
        ArrayList<Action> possibleActions = new ArrayList<>();

        ArrayList<Float> possibleThrustValues = generatePossibleActionValues(
                currentThrust, MIN_THRUST_INCREMENT_PER_TIMESTEP, MAX_THRUST_INCREMENT_PER_TIMESTEP, MIN_THRUST, MAX_THRUST);
        ArrayList<Float> possibleGimbleYValues = generatePossibleActionValues(
                (float)state.getGimbleYDouble(), MIN_GIMBLE_Y_INCREMENT_PER_TIMESTEP, MAX_GIMBLE_Y_INCREMENT_PER_TIMESTEP,
                MIN_GIMBLE_Y, MAX_GIMBLE_Y
        );
        ArrayList<Float> possibleGimbleZValues = generatePossibleActionValues(
                (float)state.getGimbleZDouble(), MIN_GIMBLE_Z_INCREMENT_PER_TIMESTEP, MAX_GIMBLE_Z_INCREMENT_PER_TIMESTEP,
                MIN_GIMBLE_Z, MAX_GIMBLE_Z
        );
        if (simulationType == SimulationType._1D) {
            possibleGimbleYValues = new ArrayList<>();
            possibleGimbleYValues.add(0.0f);
            possibleGimbleZValues = new ArrayList<>();
            possibleGimbleZValues.add(0.0f);
        } else if (simulationType == SimulationType._2D) {
            possibleGimbleYValues = new ArrayList<>();
            possibleGimbleYValues.add(0.0f);
            possibleGimbleYValues.add((float)Math.PI);
        }
        for (float possibleThrust: possibleThrustValues) {
            for (float possibleGimbleY: possibleGimbleYValues) {
                for (float possibleGimbleZ: possibleGimbleZValues) {
                    possibleActions.add(new Action(possibleThrust, possibleGimbleY, possibleGimbleZ));
                }
            }
        }

        return possibleActions;
    }

    private Float SemiSarsaUpdateStateActionValueFunction(ArrayList<StateActionTuple> episodeStateActions) {
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
        return 0.0f;
    }

    // traditional value function

    private float valueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        float value = valueFunctionTable.get(stateActionTuple);
        if (value != 0.0f) {
            // System.out.println("Already visited this state!");
            value = value;
        }
        return value;

    }

    private float valueFunction(State state, Action action) {
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        return valueFunction(stateActionTuple);
    }

    // lander value function

    private float landerValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        return valueFunctionTable.getLander(stateActionTuple);
    }

    private float landerValueFunction(State state, Action action) {
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        return landerValueFunction(stateActionTuple);
    }

    // stabilizer value function

    private float stabilizerValueFunction(StateActionTuple stateActionTuple) {
        if (!valueFunctionTable.containsKey(stateActionTuple))
            return 0.0f;
        float value = valueFunctionTable.getStabilizer(stateActionTuple);
        if (value != 0.0f) {
            //System.out.println("Already visited this state!");
            value = value;
        }
        return value;
    }

    private float stabilizerValueFunction(State state, Action action) {
        StateActionTuple stateActionTuple = new StateActionTuple(state, action);
        return stabilizerValueFunction(stateActionTuple);
    }

    // policy management

    private Action run_policy(State state) {
        ArrayList<Action> possibleActions = generatePossibleActions(state);
        Action action = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {

            action = policy(state, possibleActions, this::valueFunction);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            Action bestLanderAction = policy(state, possibleActions, this::landerValueFunction);
            Action bestStabilizerAction = policy(state, possibleActions, this::stabilizerValueFunction);
            action = OptimizedMap.combineCoupledActions(bestLanderAction, bestStabilizerAction);
        }
        return action;
    }

    public Action generateAction(State state) {
        return run_policy(state);
    }

    private Action policy(State state, ArrayList<Action> possibleActions, BiFunction<State, Action, Float> func) {
        ArrayList<Action> bestActions = new ArrayList<>();
        bestActions.add(new Action(state.getThrustDouble(), state.getGimbleYDouble(), state.getGimbleZDouble()));

        float val = Float.NEGATIVE_INFINITY;
        boolean greedy = true;
        double randomDouble = randomGenerator.nextDouble();
        if (randomDouble <= 0.05) {
            greedy = false;
        }

        for (Action action: possibleActions) {
            try {
                float v = func.apply(state, action);
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
            } catch (Exception e) {
                System.out.println("Failed to compute the value in the hashmap");
            }
        }
        // ties broken completely at random
        return bestActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStepStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
            if(currentMethod == RLMethod.SEMI_SARSA)
                SemiSarsaUpdateStateActionValueFunction(stateActionTuples);
            else if(currentMethod == RLMethod.TD0)
                TD0UpdateStateActionValueFunction(stateActionTuples, this::reward);
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        updateTerminalStateActionValueFunction(stateActionTuples, new TerminationBooleanTuple(true, true));
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples, TerminationBooleanTuple terminationBooleanTuple) {
        if(currentMethod == RLMethod.MONTE) {
            if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
                monteCarloUpdateStateActionValueFunction(stateActionTuples);
            } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
                // don't learn how to land if not hitting ground WHEN NOT IN 3D
                if ((simulationType == SimulationType._1D) || (terminationBooleanTuple.angleSuccess)) {
                    monteCarloUpdateLandingStateActionValueFunction(stateActionTuples);
                }
                monteCarloUpdateStabilizingStateActionValueFunction(stateActionTuples);
            }
        }
        else if(currentMethod == RLMethod.TD0)
            TD0UpdateStateActionValueFunction(stateActionTuples, this::terminalReward);
    }

    private float terminalReward(State lastState) {
        float angleFromZ = (float)(Math.abs(lastState.getAngleZDouble()) * (180.0f / Math.PI));
        float landingVelocity = (float)Math.abs(lastState.getVelocityDouble());
        float altitude = (float)lastState.getAltitudeDouble();

        return -(angleFromZ + landingVelocity) * (altitude + 1.0f);
    }

    private float reward(State state) {
        return -(float)(Math.abs(state.getAngleZDouble()) * (180.0f / Math.PI));
    }

    private void monteCarloUpdateStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);

        float G = terminalReward(lastStateActionTuple.state);
        int numExplorationSteps = 0;

        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            try {
                float originalValue = valueFunction(stateActionTuple);
                if (originalValue == 0.0f) numExplorationSteps++;
                valueFunctionTable.put(stateActionTuple, originalValue + alpha * (G - originalValue));
                G = (discount * G) - reward(stateActionTuple.state);
            } catch (Exception e) {
                System.out.println("EXCEPTION HERE WHEN EVALUATING VALUE FUNCTION");
            }
        }
        System.out.println("Combined - Exploration ratio " + (numExplorationSteps * 100.0f)/lastTimeStep + "% out of " + lastTimeStep + " states");
    }

    private float terminalLandingReward(State lastState) {
        float landingVelocity = (float)Math.abs(lastState.getVelocityDouble());
        float altitude = (float)lastState.getAltitudeDouble();

        return - (landingVelocity * (altitude + 1.0f));
    }

    private float rewardLander(State state) {
        return 0.0f;
    }

    private void monteCarloUpdateLandingStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);
        float G = terminalLandingReward(lastStateActionTuple.state);
        int numExplorationSteps = 0;
        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            float originalValue = landerValueFunction(stateActionTuple);
            if (originalValue == 0.0f) numExplorationSteps++;
            valueFunctionTable.putLander(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (discount * G) + rewardLander(stateActionTuple.state);
        }
        System.out.println("Landing - Exploration ratio " + (numExplorationSteps * 100.0f)/lastTimeStep + "% out of " + lastTimeStep + " states");
    }

    private float terminalStabilizingReward(State lastState) {
        return 200.0f * -(float)(Math.abs(lastState.getAngleZDouble()) * (180.0f / Math.PI));
    }

    private float rewardStabilizer(State state) {
        return (float) (1.0 / (Math.abs(state.getAngleZDouble()) * (180.0f / Math.PI) + 1.0));  // max 1
    }

    private void monteCarloUpdateStabilizingStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        int lastTimeStep = stateActionTuples.size() - 1;
        StateActionTuple lastStateActionTuple = stateActionTuples.get(lastTimeStep);
        float G = terminalStabilizingReward(lastStateActionTuple.state);
        int numExplorationSteps = 0;
        for (int timeStep = lastTimeStep; timeStep >= 0; timeStep--) {
            StateActionTuple stateActionTuple = stateActionTuples.get(timeStep);
            float originalValue = stabilizerValueFunction(stateActionTuple);
            if (originalValue == 0.0f) numExplorationSteps++;
            valueFunctionTable.putStabilizer(stateActionTuple, originalValue + alpha * (G - originalValue));
            G = (discount * G) + rewardStabilizer(stateActionTuple.state);
        }
        System.out.println("Stabilizing - Exploration ratio " + (numExplorationSteps * 100.0f)/lastTimeStep + "% out of " + lastTimeStep + " states");
    }

    private void TD0UpdateStateActionValueFunction(ArrayList<StateActionTuple> episodeStateActions, Function<State, Float> rewardFunction) {
        if(episodeStateActions.size() <= 2) { return; }

        StateActionTuple old = episodeStateActions.get(episodeStateActions.size() - 2);
        StateActionTuple current = episodeStateActions.get(episodeStateActions.size() - 1);
        float alpha = 0.3f;
        float gamma = 0.999f;

        if (!valueFunctionTable.containsKey(old)) valueFunctionTable.put(old, 0.0f);
        if (!valueFunctionTable.containsKey(current)) valueFunctionTable.put(current, 0.0f);

        valueFunctionTable.put(old, valueFunctionTable.get(old) +
                alpha * (rewardFunction.apply(current.state) + gamma * valueFunctionTable.get(current) - valueFunctionTable.get(old)));
    }

    public void resetValueFunctionTable() {
        valueFunctionTable = new OptimizedMap();
    }

    public OptimizedMap getValueFunctionTable() {
        return valueFunctionTable;
    }

    public void setValueFunctionTable(OptimizedMap valueFunctionTable) {
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





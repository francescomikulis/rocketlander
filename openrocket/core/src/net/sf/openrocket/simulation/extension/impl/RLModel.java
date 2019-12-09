package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.methods.*;

import java.util.*;
import java.util.function.BiFunction;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import static net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;
    private static boolean ONLY_GREEDY = true;

    private static float THRUST_ON_PENALTY = -5;

    // MonteCarlo or TD0
    private ModelBaseImplementation primaryMethod = new MonteCarlo();
    private ModelBaseImplementation secondaryMethod = new TD0();
    public SimulationType simulationType = SimulationType._3D;

    enum SimulationType {
        _1D, _2D, _3D
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

    private HashSet<Double> generatePossibleActionValues(double value, double smallest_increment, double max_increment, double MIN_VAL, double MAX_VAL) {
        HashSet<Double> possibleActionValues = new HashSet<>();

        int number_of_loops = (int) (2.0 * max_increment / smallest_increment) + 1;
        double starting_low_value =  value - max_increment;
        for (int i = 0; i < number_of_loops; i++) {
            double possibleValue = starting_low_value + i * smallest_increment;
            if ((possibleValue >= MIN_VAL) && (possibleValue <= MAX_VAL)) {
                possibleActionValues.add(possibleValue);
            }
        }

        return possibleActionValues;
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/
    public HashSet<Action> generatePossibleActions(State state) {
        return generatePossibleActions(state, new HashSet<>(), new HashSet<>(), new HashSet<>());
    }

    public HashSet<Action> generatePossibleActions(State state, HashSet<Double> overrideThrustValues) {
        return generatePossibleActions(state, overrideThrustValues, new HashSet<>(), new HashSet<>());
    }

    public HashSet<Action> generatePossibleActions(State state, HashSet<Double> possibleGimbleYValues, HashSet<Double> possibleGimbleZValues) {
        return generatePossibleActions(state, new HashSet<>(), possibleGimbleYValues, possibleGimbleZValues);
    }

    public HashSet<Action> generatePossibleActions(State state, HashSet<Double> overrideThrustValues, HashSet<Double> overrideGimbleYValues, HashSet<Double> overrideGimbleZValues) {
        double currentThrust = state.getDouble("thrust");
        HashSet<Action> possibleActions = new HashSet<>();

        if (simulationType == SimulationType._1D) {
            overrideGimbleYValues = new HashSet<>(Arrays.asList(0.0));
            overrideGimbleZValues = new HashSet<>(Arrays.asList(0.0));
        } else if (simulationType == SimulationType._2D) {
            overrideGimbleYValues = new HashSet<>(Arrays.asList(0.0, Math.PI + 0.00001f));
        }

        HashSet<Double> possibleThrustValues;
        if (overrideThrustValues.size() > 0) possibleThrustValues = overrideThrustValues;
        else possibleThrustValues = generatePossibleActionValues(currentThrust, THRUST_PRECISION, MAX_THRUST_INCREMENT_PER_TIMESTEP, MIN_THRUST, MAX_THRUST);

        HashSet<Double> possibleGimbleYValues;
        if (overrideGimbleYValues.size() > 0) possibleGimbleYValues = overrideGimbleYValues;
        else possibleGimbleYValues = generatePossibleActionValues((float)state.getDouble("gimbleY"), GIMBLEY_PRECISION, MAX_GIMBLEY_INCREMENT, MIN_GIMBLEY, MAX_GIMBLEY);

        HashSet<Double> possibleGimbleZValues;
        if (overrideGimbleZValues.size() > 0) possibleGimbleZValues = overrideGimbleZValues;
        else possibleGimbleZValues = generatePossibleActionValues((float)state.getDouble("gimbleZ"), GIMBLEZ_PRECISION, MAX_GIMBLEZ_INCREMENT, MIN_GIMBLEZ, MAX_GIMBLEZ);

        for (double possibleThrust: possibleThrustValues) {
            for (double possibleGimbleY: possibleGimbleYValues) {
                for (double possibleGimbleZ: possibleGimbleZValues) {
                    possibleActions.add(new Action(possibleThrust, possibleGimbleY, possibleGimbleZ));
                }
            }
        }

        return possibleActions;
    }

    // policy management

    private Action run_policy(State state, ArrayList<StateActionTuple> stateActionTuples) {
        State lastState = null;
        Action lastAction = null;
        if (stateActionTuples.size() > 0) {
            lastState = stateActionTuples.get(stateActionTuples.size() - 1).state;
            lastAction = stateActionTuples.get(stateActionTuples.size() - 1).action;
        }
        HashSet<Action> possibleActions = generatePossibleActions(state);
        Action action = null;
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            if (state.equals(lastState)) {
                System.out.println("SHOULD NEVER HAPPEN - DUPLICATE STATES with TRADITIONAL");
            }
            action = policy(state, possibleActions, primaryMethod::valueFunction, primaryMethod.getExplorationPercentage());
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            Action bestLanderAction = lastAction;
            if (!OptimizedMap.equivalentStateLander(state, lastState))
                bestLanderAction = policy(state, possibleActions, primaryMethod::landingValueFunction, primaryMethod.getExplorationPercentage());
            possibleActions = generatePossibleActions(state, new HashSet<>(Arrays.asList(state.getDouble("thrust"))));
            Action bestStabilizerAction = lastAction;
            if (!OptimizedMap.equivalentStateStabilizer(state, lastState))
                bestStabilizerAction = policy(state, possibleActions, secondaryMethod::stabilizingValueFunction, secondaryMethod.getExplorationPercentage());
            action = OptimizedMap.combineCoupledActions(bestLanderAction, bestStabilizerAction);
        }
        return action;
    }

    public Action generateAction(State state, ArrayList<StateActionTuple> stateActionTuples) {
        return run_policy(state, stateActionTuples);
    }

    private Action policy(State state, HashSet<Action> possibleActions, BiFunction<State, Action, Float> func, float explorationPercentage) {
        HashSet<Action> bestActions = new HashSet<>();
        bestActions.add(new Action(state.getDouble("thrust"), state.getDouble("gimbleY"), state.getDouble("gimbleZ")));

        float val = Float.NEGATIVE_INFINITY;
        boolean greedy = true;
        double randomDouble = randomGenerator.nextDouble();
        if (randomDouble <= explorationPercentage) {
            greedy = false;
        }

        for (Action action: possibleActions) {
            //try {
                float v = func.apply(state, action);
                if (greedy) {
                    if (v > val) {
                        // value is best compared to all previous encounters.  Reset bestAction ArrayList.
                        val = v;
                        bestActions = new HashSet<>();
                        bestActions.add(action);
                    } else if (v == val) {
                        // value is equal to other best value.  Add to bestAction ArrayList.
                        bestActions.add(action);
                    }
                } else {
                    bestActions.add(action);
                }
            //} catch (Exception e) {
            //    System.out.println("Failed to compute the value in the hashmap");
            //}
        }
        // ties broken completely at random
        ArrayList<Action> selectableActions = new ArrayList<>(bestActions);
        return selectableActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStepStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            primaryMethod.updateStepFunction(stateActionTuples);

        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            primaryMethod.updateStepLandingFunction(stateActionTuples);
            secondaryMethod.updateStepStabilizingFunction(stateActionTuples);
        }
    }

    public void updateTerminalStateActionValueFunction(ArrayList<StateActionTuple> stateActionTuples, TerminationBooleanTuple terminationBooleanTuple) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            primaryMethod.updateTerminalFunction(stateActionTuples);
        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            if ((simulationType == SimulationType._1D) || (terminationBooleanTuple.landingSucceeded()))
                primaryMethod.updateTerminalLandingFunction(stateActionTuples);
            if (simulationType != SimulationType._1D)
                secondaryMethod.updateTerminalStabilizingFunction(stateActionTuples);
        }
    }

    public void resetValueFunctionTable() {
        setValueFunctionTable(new OptimizedMap());
    }

    public OptimizedMap getValueFunctionTable() {
        return primaryMethod.getValueFunctionTable();
    }

    public void setValueFunctionTable(OptimizedMap valueFunctionTable) {
        primaryMethod.setValueFunctionTable(valueFunctionTable);
        secondaryMethod.setValueFunctionTable(valueFunctionTable);
    }

    public ModelBaseImplementation getPrimaryMethod() {
        return primaryMethod;
    }

    public void setPrimaryMethod(ModelBaseImplementation method) {
        // conserve the instance of the OptimizedMap
        OptimizedMap valueFunctionTable = primaryMethod.getValueFunctionTable();
        primaryMethod = method;
        secondaryMethod = primaryMethod;
        method.setValueFunctionTable(valueFunctionTable);
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





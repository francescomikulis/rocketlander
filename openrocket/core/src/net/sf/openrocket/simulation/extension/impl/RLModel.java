package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.methods.*;

import java.lang.reflect.Array;
import java.util.*;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import static java.lang.Math.PI;
import static net.sf.openrocket.simulation.extension.impl.OptimizedMap.*;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;


public class RLModel {
    private Random randomGenerator = new Random();
    private RLEpisodeManager episodeManager = null;

    // MonteCarlo or TD0

    //  MapMethod.Traditional
    /*
    private ModelBaseImplementation primaryMethod = new MonteCarlo(generalDefinition);
    private ModelBaseImplementation secondaryMethod = null;
    private ModelBaseImplementation tertiaryMethod = null;

    private HashMap<String, ModelBaseImplementation> methods = new HashMap<String, ModelBaseImplementation>() {{
            put("general", primaryMethod);
    }};
     */

    //  MapMethod.Coupled

    private ModelBaseImplementation primaryMethod = new MonteCarlo(landerDefinition);
    private ModelBaseImplementation secondaryMethod = new TD0(reacherDefinition);
    private ModelBaseImplementation tertiaryMethod = new TD0(stabilizerDefinition);

    private LinkedHashMap<String, ModelBaseImplementation> methods = new LinkedHashMap<String, ModelBaseImplementation>() {{
        put("lander", primaryMethod);
        put("reacher", secondaryMethod);
        put("stabilizer", tertiaryMethod);
    }};

    public String symmetryAxis2D = "X";
    public String symmetryAxis3D = "Y";
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
        if (symmetryAxis2D.equals(symmetryAxis3D)) {
            System.out.println("ISSUES IN SYMMETRY AXIS ASSIGNMENT!!!!");
            System.out.println("YOU MUST SELECT 2 DIFFERENT SYMMETRY AXIS FOR 2D AND 3D");
        }
        episodeManager = RLEpisodeManager.getInstance();
        episodeManager.safeActionValueFunctionInitialization();
    }

    public Set<String> getMethodNames() {
        return methods.keySet();
    }

    private HashSet<Integer> generatePossibleActionValuesInts(int value, int[] definition) {
        HashSet<Integer> possibleActionValues = new HashSet<>();
        for (int i = definition[0]; i <= definition[1]; i++) {
            possibleActionValues.add(i);
        }
        return possibleActionValues;
    }

    private boolean incrementIndeces(int[] indeces, int[] maxIndeces) {
        return _incrementIndeces(indeces, maxIndeces, indeces.length - 1);
    }

    public boolean _incrementIndeces(int[] indeces, int[] maxIndeces, int position) {
        if (position == -1) return false;
        if (indeces[position] < maxIndeces[position]) {
            indeces[position] += 1;
            return true;
        }
        indeces[position] = 0;
        return _incrementIndeces(indeces, maxIndeces, position - 1);
    }

    /***
    In the future need to also consider the state and the velocity we are currently at.
     ***/

    public HashSet<Action> generatePossibleActions(State state) {
        HashMap<String, int[]> actionDefinition = state.definition.get("actionDefinitionIntegers");
        ArrayList<ArrayList<Integer>> possibleActionInts = new ArrayList<>();

        HashSet<String> symmetryAxes = null;
        if (state.definition.get("meta").containsKey("symmetry")) {
            String[] symmetryAxesArray = ((String) state.definition.get("meta").get("symmetry")).split(",");
            symmetryAxes = new HashSet<>(Arrays.asList(symmetryAxesArray));
        }

        ArrayList<String> actionFields = new ArrayList<>(actionDefinition.keySet());
        ArrayList<String> actionStrings = new ArrayList<>();  // these have actual components (e.g. gimbalX - NOT gimbal)
        for (String actionField: actionFields) {
            String realField = actionField;
            if (symmetryAxes != null) {
                if (symmetryAxes.contains(actionField))
                    realField += state.symmetry;
            }
            actionStrings.add(realField);
            possibleActionInts.add(new ArrayList<>(generatePossibleActionValuesInts(state.get(realField), actionDefinition.get(actionField))));
        }

        int size = actionStrings.size();
        int[] indeces = new int[size];
        int[] maxIndeces = new int[size];
        for (int i = 0; i < size; i++) {
            indeces[i] = 0;
            maxIndeces[i] = possibleActionInts.get(i).size() - 1;
        }

        HashSet<Action> possibleActions = new HashSet<>();
        while (true) {
            HashMap<String, Integer> actionValueMap = new HashMap<>();
            for (int i = 0; i < size; i++) {
                actionValueMap.put(actionStrings.get(i), possibleActionInts.get(i).get(indeces[i]));
            }
            Action action = new Action(actionValueMap, state.definition);
            action.setAllSymmetries(state.symmetry);
            possibleActions.add(action);

            if (!incrementIndeces(indeces, maxIndeces)) {
                break;
            }
        }
        return possibleActions;
    }

    private void addHierarchicalMDPSelection(SimulationStatus status, State currentState, CoupledStates coupledStates, Action action) {
        if (action.definition.containsKey("MDPSelectionFormulas")) {
            for (Object entryObject : action.definition.get("MDPSelectionFormulas").entrySet()) {
                Map.Entry<String, ArrayList<String>> entry = (Map.Entry<String, ArrayList<String>>) entryObject;
                String fieldName = entry.getKey();
                ArrayList<String> advancedIfElseFormula = entry.getValue();

                String selectedMDPName = null;
                for (int i = 0; i < advancedIfElseFormula.size(); i += 2) {
                    String formulaConditions = advancedIfElseFormula.get(i);
                    ExpressionEvaluator.Formula formula = ExpressionEvaluator.getInstance().generateFormula(formulaConditions);
                    if (formula.evaluate(currentState) != 0) {
                        selectedMDPName = advancedIfElseFormula.get(i + 1);
                        break;
                    }
                }
                if (selectedMDPName == null) {
                    if (advancedIfElseFormula.size() == 0) {
                        System.out.println("Unable to generate MDP selection!");
                    } else {
                        // default to last one!
                        selectedMDPName = advancedIfElseFormula.get(advancedIfElseFormula.size() - 1);
                    }
                }

                boolean actuallyCreateNewState = true;
                String symmetryAxis = null;
                if (fieldName.contains("X")) symmetryAxis = "X";
                else if (fieldName.contains("Y")) symmetryAxis = "Y";

                if (symmetryAxis != null) {
                    if (simulationType == SimulationType._1D) {
                        actuallyCreateNewState = false;
                    }
                    if (simulationType == SimulationType._2D) {
                        if (!fieldName.contains(symmetryAxis2D)) {
                            actuallyCreateNewState = false;
                        }
                    }
                }

                if (selectedMDPName == null) actuallyCreateNewState = false;

                if (actuallyCreateNewState) {
                    State childState = new State(status, methods.get(selectedMDPName).definition);
                    childState.setAllSymmetries(symmetryAxis);
                    coupledStates.add(childState);
                }
            }
        }
    }

    // policy management

    private Object[] run_policy(SimulationStatus status, LinkedHashMap<String, ArrayList<StateActionTuple>> SA) {
        ArrayList<Action> actions = new ArrayList<>();
        CoupledStates coupledStates = new CoupledStates();
        coupledStates.add(new State(status, primaryMethod.definition));

        int methodCounter = 0;
        while (methodCounter != coupledStates.size()) {
            State state = coupledStates.get(methodCounter);
            String methodName = (String)state.definition.get("meta").get("name");
            ModelBaseImplementation method = methods.get(methodName);

            String storageName = methodName;
            if (state.symmetry != null) storageName += state.symmetry;
            if (!SA.containsKey(storageName)) SA.put(storageName, new ArrayList<>());
            ArrayList<StateActionTuple> stateActionTuples = SA.get(storageName);

            State lastState = getLastStateOrNull(stateActionTuples);
            Action bestAction = getLastActionOrNull(stateActionTuples);
            if (needToChooseNewAction(state, lastState, bestAction)) {
                HashSet<Action> possibleActions = generatePossibleActions(state);
                bestAction = policy(state, possibleActions, method);
                state.setAllSymmetries(bestAction.symmetry);
                addStateActionTupleIfNotDuplicate(new StateActionTuple(state, bestAction), stateActionTuples);
            }

            // dynamically adds entries to coupledStates - bad convention but required!
            addHierarchicalMDPSelection(status, state, coupledStates, bestAction);

            // propagate selection
            for (State s: coupledStates)
                bestAction.applyDefinitionValuesToState(s);
            actions.add(bestAction);
            methodCounter += 1;
        }

        coupledStates.freeze();

        // need to update the formulas after getting the new data
        for (State s: coupledStates)
            s.runMDPDefinitionFormulas();

        CoupledActions coupledActions = new CoupledActions(actions.toArray(new Action[actions.size()]));

        return new Object[]{coupledStates, coupledActions};
    }

    private void addStateActionTupleIfNotDuplicate(StateActionTuple stateActionTuple, ArrayList<StateActionTuple> SA) {
        if (SA.size() == 0)
            SA.add(stateActionTuple);
        else if (!equivalentState(SA.get(SA.size() - 1).state, stateActionTuple.state)) {
            SA.add(stateActionTuple);
        }
    }

    public CoupledStates generateCoupledStatesBasedOnLastActions(SimulationStatus status, CoupledActions coupledActions) {
        CoupledStates coupledStates = new CoupledStates();
        coupledStates.add(new State(status, primaryMethod.definition));

        int methodCounter = 0;
        while (methodCounter != coupledStates.size()) {
            State currentState = coupledStates.get(methodCounter);
            addHierarchicalMDPSelection(status, currentState, coupledStates, coupledActions.get(methodCounter));
            methodCounter += 1;
        }

        return coupledStates;
    }

    public Object[] generateStateAndActionAndStoreHistory(SimulationStatus status, LinkedHashMap<String, ArrayList<StateActionTuple>> SA) {
        return run_policy(status, SA);
    }

    private Action policy(State state, HashSet<Action> possibleActions, ModelBaseImplementation method) {
        HashSet<Action> bestActions = new HashSet<>();

        float explorationPercentage = method.getExplorationPercentage();

        float val = Float.NEGATIVE_INFINITY;
        boolean greedy = true;
        double randomDouble = randomGenerator.nextDouble();
        if (randomDouble <= explorationPercentage) {
            greedy = false;
        }

        for (Action action: possibleActions) {
            float v = method.valueFunction(state, action);
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
        }
        if (bestActions.size() == 0) {
            System.out.println("Size of best actions is 0.  This will cause failure - WHY IS IT HAPPENING?");
        }
        // ties broken completely at random
        ArrayList<Action> selectableActions = new ArrayList<>(bestActions);
        return selectableActions.get(randomGenerator.nextInt(bestActions.size()));
    }

    public void updateStepStateActionValueFunction(LinkedHashMap<String, ArrayList<StateActionTuple>> SA, LinkedHashMap<String, Integer> lastUpdateSizes) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            if (stateHasChanged(SA.get("lander"), lastUpdateSizes.get("lander")))
                methods.get("lander").updateStepFunction(SA.get("lander"));

        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            if (stateHasChanged(SA.get("lander"), lastUpdateSizes.get("lander"))) {
                methods.get("lander").updateStepLanderFunction(SA.get("lander"));
            }
            if (stateHasChanged(SA.get("stabilizerX"), lastUpdateSizes.get("stabilizerX"))) {
                methods.get("stabilizer").updateStepStabilizerFunction(SA.get("stabilizerX"));
            }
            if (stateHasChanged(SA.get("stabilizerY"), lastUpdateSizes.get("stabilizerY"))) {
                methods.get("stabilizer").updateStepStabilizerFunction(SA.get("stabilizerY"));
            }
            if (stateHasChanged(SA.get("reacherX"), lastUpdateSizes.get("reacherX"))) {
                methods.get("reacher").updateStepReachingFunction(SA.get("reacherX"));
            }
            if (stateHasChanged(SA.get("reacherY"), lastUpdateSizes.get("reacherY"))) {
                methods.get("reacher").updateStepReachingFunction(SA.get("reacherY"));
            }
        }
    }

    public void updateTerminalStateActionValueFunction(LinkedHashMap<String, ArrayList<StateActionTuple>> SA, TerminationBooleans terminationBooleans) {
        if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Traditional) {
            methods.get("lander").updateTerminalFunction(SA.get("lander"));

        } else if (OptimizedMap.mapMethod == OptimizedMap.MapMethod.Coupled) {
            methods.get("lander").updateTerminalLanderFunction(SA.get("lander"));
            methods.get("stabilizer").updateTerminalStabilizerFunction(SA.get("stabilizerX"));
            methods.get("stabilizer").updateTerminalStabilizerFunction(SA.get("stabilizerY"));
            methods.get("reacher").updateTerminalReachingFunction(SA.get("reacherX"));
            methods.get("reacher").updateTerminalReachingFunction(SA.get("reacherY"));
        }

        if (terminationBooleans.totalSuccess()) {
            System.out.print("+");
        } else {
            System.out.print("-");
        }
    }

    public void resetValueFunctionTable() {
        setValueFunctionTable(new OptimizedMap());
    }

    public OptimizedMap getValueFunctionTable() {
        return primaryMethod.getValueFunctionTable();
    }

    public void setValueFunctionTable(OptimizedMap valueFunctionTable) {
        for (Map.Entry<String, ModelBaseImplementation> entry: methods.entrySet()) {
            entry.getValue().setValueFunctionTable(valueFunctionTable);
        }
    }


    private boolean stateHasChanged(ArrayList<StateActionTuple> episodeStateActions, Integer previousSize){
        State lastStateOrNull = getLastStateOrNull(episodeStateActions);
        if (lastStateOrNull == null) return false;  // no state exists for reference
        State lastLastStateOrNull = getLastLastStateOrNull(episodeStateActions);
        if (lastStateOrNull.equals(lastLastStateOrNull)) return false;
        if ((previousSize == null) || (previousSize == 0)) return true;
        return (episodeStateActions.size() != previousSize);
    }

    private State getLastStateOrNull(ArrayList<StateActionTuple> episodeStateActions){
        if (episodeStateActions == null) return null;
        return (episodeStateActions.isEmpty()) ? null : episodeStateActions.get(episodeStateActions.size() - 1).state;
    }

    private State getLastLastStateOrNull(ArrayList<StateActionTuple> episodeStateActions){
        if (episodeStateActions == null) return null;
        return (episodeStateActions.size() <= 1) ? null : episodeStateActions.get(episodeStateActions.size() - 2).state;
    }

    private Action getLastActionOrNull(ArrayList<StateActionTuple> episodeStateActions){
        if (episodeStateActions == null) return null;
        return (episodeStateActions.isEmpty()) ? null : episodeStateActions.get(episodeStateActions.size() - 1).action;
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





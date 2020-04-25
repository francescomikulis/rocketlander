package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.methods.*;

import java.util.*;
import java.util.function.Function;

import net.sf.openrocket.simulation.extension.impl.StateActionTuple.*;

import static net.sf.openrocket.simulation.extension.impl.OptimizedMap.*;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;


public class RLModel {
    private static volatile RLModel instance;
    private Random randomGenerator = new Random();
    OptimizedMap valueFunctionTable;

    private LinkedHashMap<String, ModelBaseImplementation> methods;

    public String symmetryAxis2D = "X";
    public String symmetryAxis3D = "Y";
    public SimulationType simulationType = SimulationType._3D;

    enum SimulationType {
        _1D, _2D, _3D
    }

    private RLModel(){
        OptimizedMap.deepCopyElementsFromKnownHashMapToOtherMap(_generalDefinition, generalDefinition);
        OptimizedMap.deepCopyElementsFromKnownHashMapToOtherMap(_landerDefinition, landerDefinition);
        OptimizedMap.deepCopyElementsFromKnownHashMapToOtherMap(_reacherDefinition, reacherDefinition);
        OptimizedMap.deepCopyElementsFromKnownHashMapToOtherMap(_stabilizerDefinition, stabilizerDefinition);

        constructor(false);
    }

    private void constructor(boolean reset) {
        // this is the default constructor - these hashmap definition should be read from elsewhere

        constructor("X", "Y", SimulationType._3D, reset, landerDefinition, reacherDefinition, stabilizerDefinition);
        // constructor("X", "Y", SimulationType._3D, generalDefinition);
    }

    private void constructor(String symmetryAxis2D, String symmetryAxis3D, SimulationType simulationType, boolean reset, HashMap<String, LinkedHashMap> ... definitions) {
        valueFunctionTable = null;
        this.symmetryAxis2D = symmetryAxis2D;
        this.symmetryAxis3D = symmetryAxis3D;
        this.simulationType = simulationType;
        this.methods = new LinkedHashMap<>();
        for (HashMap<String, LinkedHashMap> definition: definitions) {
            addMethodFromDefinition(definition);
        }
        // setValueFunctionTable
        if (reset) {
            this.valueFunctionTable = new OptimizedMap(methods);
        } else {
            this.valueFunctionTable = RLObjectFileStore.getInstance().readActionValueFunctionFromMethods(methods);
        }
    }

    public void resetValueFunctionTable() {
        valueFunctionTable.resetValueFunctionTable(methods);
        constructor(true);
    }

    public OptimizedMap getValueFunctionTable() {
        return valueFunctionTable;
    }

    private void addMethodFromDefinition(HashMap<String, LinkedHashMap> definition) {
        String RLMethodName = (String)definition.get("meta").get("name");
        String RLMethod = (String)definition.get("meta").get("methodName");
        ModelBaseImplementation model = null;
        switch (RLMethod.toUpperCase()) {
            case "MC":
                model = new MonteCarlo(definition); break;
            case "TD0":
                model = new TD0(definition); break;
            case "SARSA":
                model = new Sarsa(definition); break;
            default:
                System.out.println("METHOD NAME NOT DEFINED IN THE IMPLEMENTATION.  Must choose between MC, TD0, SARSA.");
                return;
        }

        if (definition.get("meta").containsKey("discount")) {
            model.setTerminalDiscount((float)Double.parseDouble((String)definition.get("meta").get("discount")));
        }
        if (definition.get("meta").containsKey("stepDiscount")) {
            model.setStepDiscount((float)Double.parseDouble((String)definition.get("meta").get("stepDiscount")));
        }
        if (definition.get("meta").containsKey("alpha")) {
            model.setAlpha((float)Double.parseDouble((String)definition.get("meta").get("alpha")));
        }
        if (definition.get("meta").containsKey("exploration")) {
            model.setExploration((float)Double.parseDouble((String)definition.get("meta").get("exploration")));
        }

        methods.put(RLMethodName, model);
    }

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

    public LinkedHashMap<String, ModelBaseImplementation> getMethods() {
        return methods;
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

    private HashSet<Integer> generatePossibleActionValuesMDPInts(State state, String MDPActionSelectionField) {
        HashMap<String, LinkedHashMap> definition = state.definition;
        HashSet<Integer> possibleActionValues = new HashSet<>();

        if (definition.containsKey("MDPSelectionFormulas") && definition.get("MDPSelectionFormulas").containsKey(MDPActionSelectionField)) {
            ArrayList<String> advancedIfElseFormula = (ArrayList<String>)(definition.get("MDPSelectionFormulas").get(MDPActionSelectionField));

            String selectedMDPName = null;
            for (int i = 0; i < advancedIfElseFormula.size(); i += 2) {
                String formulaConditions = advancedIfElseFormula.get(i);
                ExpressionEvaluator.Formula formula = ExpressionEvaluator.getInstance().generateFormula(formulaConditions);
                if (formula.evaluate(state) != 0) {  // 0 is false, 1 is true
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
            // overrides the traditional options calculations of that MDP Field Name
            if (selectedMDPName != null)
                possibleActionValues.add((Integer)definition.get("childrenMDPIntegerOptions").get(selectedMDPName));
        } else {
            String stringMDPNames = (String) definition.get("childrenMDPOptions").get(MDPActionSelectionField);
            String[] MDPNames = stringMDPNames.split(",");
            for (String MDPName : MDPNames) {
                // convert MDPName to fixed integer number
                possibleActionValues.add((Integer) definition.get("childrenMDPIntegerOptions").get(MDPName));
            }
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
            if (state.definition.containsKey("childrenMDPOptions") && state.definition.get("childrenMDPOptions").containsKey(realField)) {
                ArrayList<Integer> childMDPOptions = new ArrayList<>(generatePossibleActionValuesMDPInts(state, realField));
                possibleActionInts.add(childMDPOptions);
                if (childMDPOptions.size() != 1) {
                    System.out.println("Formula evaluation for child MDP selection did not evaluate correctly.  This should not be possible!");
                }
            } else {
                possibleActionInts.add(new ArrayList<>(generatePossibleActionValuesInts(state.get(realField), actionDefinition.get(actionField))));
            }
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

    private void addHierarchicalMDPSelection(SimulationStatus status, CoupledStates coupledStates, Action action) {
        if (action.definition.containsKey("childrenMDPOptions")) {
            for (Object entryObject : action.definition.get("childrenMDPOptions").keySet()) {
                String MDPActionSelectionField = (String)entryObject;
                int SelectedMDPID = action.get(MDPActionSelectionField);

                // reverse hashmap and find the MDP name
                String selectedMDPName = null;
                for (Object internalEntryObject : action.definition.get("childrenMDPIntegerOptions").entrySet()) {
                    Map.Entry<String, Integer> internalEntry = (Map.Entry<String, Integer>) internalEntryObject;
                    if (internalEntry.getValue() == SelectedMDPID) {
                        selectedMDPName = internalEntry.getKey();
                    }
                }

                boolean actuallyCreateNewState = true;
                String symmetryAxis = null;
                if (MDPActionSelectionField.contains("X")) symmetryAxis = "X";
                else if (MDPActionSelectionField.contains("Y")) symmetryAxis = "Y";
                if (symmetryAxis != null) {
                    if (simulationType == SimulationType._1D) {
                        actuallyCreateNewState = false;
                    }
                    if (simulationType == SimulationType._2D) {
                        if (!MDPActionSelectionField.contains(symmetryAxis2D)) {
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
        HashMap<String, LinkedHashMap> firstDefinition = null;
        for (String key: methods.keySet()) { firstDefinition = methods.get(key).definition; break; }
        coupledStates.add(new State(status, firstDefinition));

        int methodCounter = 0;
        while (methodCounter != coupledStates.size()) {
            State state = coupledStates.get(methodCounter);
            String methodName = (String)state.definition.get("meta").get("name");
            ModelBaseImplementation method = methods.get(methodName);

            String storageName = methodName;
            if (state.symmetry != null) storageName += state.symmetry;
            if (!SA.containsKey(storageName)) SA.put(storageName, new ArrayList<>());
            ArrayList<StateActionTuple> stateActionTuples = SA.get(storageName);

            State lastState = null;
            Action bestAction = null;
            if ((stateActionTuples != null) && !stateActionTuples.isEmpty()) {
                lastState = stateActionTuples.get(stateActionTuples.size() - 1).state;
                bestAction = stateActionTuples.get(stateActionTuples.size() - 1).action;
            }

            if (needToChooseNewAction(state, lastState, bestAction)) {
                HashSet<Action> possibleActions = generatePossibleActions(state);
                bestAction = policy(state, possibleActions, method);
                state.setAllSymmetries(bestAction.symmetry);
                addStateActionTupleIfNotDuplicate(new StateActionTuple(state, bestAction), stateActionTuples);
            }

            // dynamically adds entries to coupledStates - bad convention but required!
            addHierarchicalMDPSelection(status, coupledStates, bestAction);

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
        coupledStates.add(new State(status, coupledActions.get(0).definition));

        int methodCounter = 0;
        while (methodCounter != coupledStates.size()) {
            addHierarchicalMDPSelection(status, coupledStates, coupledActions.get(methodCounter));
            methodCounter += 1;
        }

        return coupledStates;
    }

    public Object[] generateStateAndActionAndStoreHistory(SimulationStatus status, LinkedHashMap<String, ArrayList<StateActionTuple>> SA) {
        return run_policy(status, SA);
    }

    private Action policy(State state, HashSet<Action> possibleActions, ModelBaseImplementation method) {
        HashSet<Action> bestActions = new HashSet<>();

        float explorationPercentage = method.getExploration();

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
        for (Map.Entry<String, ModelBaseImplementation> entry: methods.entrySet()) {
            String methodName = entry.getKey();
            ModelBaseImplementation method = entry.getValue();
            ExpressionEvaluator.Formula reward = ExpressionEvaluator.getInstance().generateFormula((String)method.definition.get("meta").get("reward"));

            if (!methods.get(methodName).definition.get("meta").containsKey("symmetry")) {
                methods.get(methodName).updateStepCustomFunction(SA.get(methodName), reward);
            } else {
                if (0.5 < randomGenerator.nextDouble()) {
                    methods.get(methodName).updateStepCustomFunction(SA.get(methodName + "X"), reward);
                    methods.get(methodName).updateStepCustomFunction(SA.get(methodName + "Y"), reward);
                } else {
                    methods.get(methodName).updateStepCustomFunction(SA.get(methodName + "Y"), reward);
                    methods.get(methodName).updateStepCustomFunction(SA.get(methodName + "X"), reward);
                }
            }
        }
    }

    public void updateTerminalStateActionValueFunction(LinkedHashMap<String, ArrayList<StateActionTuple>> SA, TerminationBooleans terminationBooleans) {
        for (Map.Entry<String, ModelBaseImplementation> entry: methods.entrySet()) {
            String methodName = entry.getKey();
            ModelBaseImplementation method = entry.getValue();
            if (!method.definition.get("meta").containsKey("terminalReward")) continue;  // terminalReward not specified, skip method!
            ExpressionEvaluator.Formula terminalReward = ExpressionEvaluator.getInstance().generateFormula((String)method.definition.get("meta").get("terminalReward"));
            ExpressionEvaluator.Formula reward = ExpressionEvaluator.getInstance().generateFormula((String)method.definition.get("meta").get("reward"));

            if (!methods.get(methodName).definition.get("meta").containsKey("symmetry")) {
                methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName), terminalReward, reward);
            } else {
                if (0.5 < randomGenerator.nextDouble()) {
                    methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName + "X"), terminalReward, reward);
                    methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName + "Y"), terminalReward, reward);
                } else {
                    methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName + "Y"), terminalReward, reward);
                    methods.get(methodName).updateTerminalCustomFunction(SA.get(methodName + "X"), terminalReward, reward);
                }
            }
        }

        if (terminationBooleans.totalSuccess()) {
            System.out.print("+");
        } else {
            System.out.print("-");
        }
    }
}





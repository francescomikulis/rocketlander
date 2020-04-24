package net.sf.openrocket.simulation.extension.impl;

import java.util.*;
import java.io.Serializable;

import com.sun.istack.internal.NotNull;
import net.sf.openrocket.simulation.SimulationStatus;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.Formula;
import net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.*;
import net.sf.openrocket.util.ArrayList;
import net.sf.openrocket.util.Coordinate;
import net.sf.openrocket.util.Quaternion;

import static net.sf.openrocket.simulation.extension.impl.methods.ExpressionEvaluator.*;
import static net.sf.openrocket.simulation.extension.impl.methods.ModelBaseImplementation.*;


public class StateActionTuple implements Serializable {
    public State state;
    public Action action;
    public StateActionTuple(State state, Action action) {
        this.state = state;
        this.action = action;
        if (state.definition != action.definition) {
            System.out.println("State and action in same tuple have different definitions");
            this.action.definition = this.state.definition;
        }
        tupleRunAllDefinitionSpecial(this.state.definition);
    }

    private void tupleRunAllDefinitionSpecial(HashMap<String, LinkedHashMap> definition) {
        // formula logic

        HashSet<String> symmetryAxes = null;
        if (definition.get("meta").containsKey("symmetry")) {
            String[] symmetryAxesArray = ((String)definition.get("meta").get("symmetry")).split(",");
            symmetryAxes = new HashSet<>(Arrays.asList(symmetryAxesArray));

            String stateSym = state.symmetry;
            String actionSym = action.symmetry;
            if ((stateSym == null) && (actionSym == null)) {
                System.out.println("SYMMETRY MUST BE SET IN EITHER ACTION OR STATE!");
            }
            else if ((stateSym != null) && (actionSym != null) && (!stateSym.equals(actionSym))){
                System.out.println("DIFFERENT VALUES OF SYMMETRY ARE INCOMPATIBLE!");
            }
            else if (stateSym != null) action.symmetry = state.symmetry;
            else if (actionSym != null) state.symmetry = action.symmetry;
        }

        if (state.symmetry != null) {
            String formulaFieldName = "symmetryFormulas" + state.symmetry;
            if (definition.containsKey(formulaFieldName)) {  // assignment formulas
                for (Object entryObject : definition.get(formulaFieldName).entrySet()) {
                    Map.Entry<String, Formula> entry = (Map.Entry<String, Formula>) entryObject;
                    String assignToField = entry.getKey();
                    Formula formula = entry.getValue();
                    this.state.setDouble(assignToField, evaluateFormulaBestGuess(formula, state, action));
                    this.action.setDouble(assignToField, evaluateFormulaBestGuess(formula, action, state));
                }
            }
        }

        if (definition.containsKey("formulas")) {
            for (Object entryObject : definition.get("formulas").entrySet()) {
                Map.Entry<String, Formula> entry = (Map.Entry<String, Formula>) entryObject;
                String assignToField = entry.getKey();
                Formula formula = entry.getValue();
                this.state.setDouble(assignToField, evaluateFormulaBestGuess(formula, state, action));
                this.action.setDouble(assignToField, evaluateFormulaBestGuess(formula, action, state));
            }
        }
    }

    @Override
    public String toString() {
        String stateString = (state != null) ? state.toString() : "";
        String actionString = (action != null) ? action.toString() : "";
        return ("(" + stateString + ", " + actionString + ")");
    }

    @Override
    public int hashCode() {
        return state.hashCode() + action.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        StateActionTuple other = (StateActionTuple) obj;
        return this.action.equals(other.action) && this.state.equals(other.state);
    }

    public StateActionTuple deepcopy() {
        return new StateActionTuple(state.deepcopy(), action.deepcopy());
    }

    public static class StateActionClass implements Serializable {
        public HashMap<String, Integer> valueMap = new HashMap<>();
        public HashMap<String, LinkedHashMap> definition;
        public String symmetry = null;

        protected void runALLMDPDefinitionFormulas() {
            HashMap<String, LinkedHashMap> originalDefinition = this.definition;
            this.definition = landerDefinition;
            runMDPDefinitionFormulas();
            this.definition = reacherDefinition;
            runMDPDefinitionFormulas();
            this.definition = stabilizerDefinition;
            runMDPDefinitionFormulas();
            this.definition = originalDefinition;
        }

        protected void runMDPDefinitionFormulas() {
            if (symmetry != null) {
                String formulaFieldName = "symmetryFormulas" + symmetry;
                if (definition.containsKey(formulaFieldName)) {  // assignment formulas
                    for (Object entryObject : definition.get(formulaFieldName).entrySet()) {
                        Map.Entry<String, Formula> entry = (Map.Entry<String, Formula>) entryObject;
                        String assignToField = entry.getKey();
                        Formula formula = entry.getValue();
                        setDouble(assignToField, evaluateFormula(formula, this));
                    }
                }
            }

            if (definition.containsKey("formulas")) {
                for (Object entryObject : definition.get("formulas").entrySet()) {
                    Map.Entry<String, Formula> entry = (Map.Entry<String, Formula>) entryObject;
                    String assignToField = entry.getKey();
                    Formula formula = entry.getValue();
                    setDouble(assignToField, evaluateFormula(formula, this));
                }
            }
        }

        public void setAllSymmetries(String symmetryAxis) {
            if (definition.get("meta").containsKey("symmetry")) {
                String axes = (String)definition.get("meta").get("symmetry");
                String[] symmetryAxes = axes.split(",");
                for (String axis: symmetryAxes) {
                    setSymmetry(axis, symmetryAxis);
                }
            }
        }

        public void setSymmetry(String field, String axis) {
            symmetry = axis;
            if (symmetry == null) return;

            String originalField = field + axis;
            String copyToField = field;

            String formulaFieldName = "symmetryFormulas" + symmetry;

            if (!definition.containsKey(formulaFieldName)) {
                definition.put(formulaFieldName, new LinkedHashMap<String, Formula>());
            }
            if (!definition.get(formulaFieldName).containsKey(copyToField)) {
                definition.get(formulaFieldName).put(copyToField, ExpressionEvaluator.getInstance().generateAssignmentFormula(originalField));
            }
            runMDPDefinitionFormulas();
        }

        public static int group_by_precision(double value, double precision) {
            return (int) Math.floor(value / precision + 0.5);  // shift of 0.5 is required to create symmetrical values
        }

        /**
         * e.g. symmetric values around 0 [-10, 10]
         *
         * Without symmetric grouping:
         *  (-10, -5) --> -2
         *  ( -5, -0) --> -1
         *  ( 0,   5) --> 0
         *  ( 5,  10) --> 1
         *
         * With symmetric grouping [if (result < 0) result += 1;]
         *  (-10, -5) --> -2 + 1 = -1
         *  ( -5, -0) --> -1 + 1 = 0
         *  ( 0,   5) --> 0
         *  ( 5,  10) --> 1
         */
        protected static int group_by_precision_symmetric(double value, double precision) {
            int result = (int) Math.floor((double)value / (double)precision);
            if (result < 0)
                result += 1;
            return result;
        }

        protected static int group_angle_by_precision(double angle, double precision) {
            //if (angle == 2 * Math.PI) angle -= 0.00001;  // fall into right sector
            //double shiftedAngle = ((angle + (4 * Math.PI)) % (2 * Math.PI));
            return group_by_precision(angle, precision);
        }

        public int get(String field) {
            Object result = valueMap.get(field);
            if (result == null) return 0;
            return (int)result;
        }

        public static int get(StateActionClass object, String field) {
            return object.get(field);
        }

        public double getDouble(String field) {
            return get(field) * getPrecisionConstant(field);
        }

        public float[] getConstants(String field) {
            return getDefinitionConstants(this, field);
        }

        public double getMinConstant(String field) {
            return getDefinitionConstants(this, field)[0];
        }

        public double getMaxConstant(String field) {
            return getDefinitionConstants(this, field)[1];
        }

        public double getPrecisionConstant(String field) {
            return getDefinitionConstants(this, field)[2];
        }

        public static float[] getDefinitionConstants(StateActionClass object, String field) {
            float[] defaultDefinition = new float[]{-100.0f, 100.0f, 0.0000001f};
            float[] fieldDefinition;
            if ((object == null) || (object.definition == null))
                System.out.println("SERIOUS PROBLEM IN DEFINITION INITIALIZATION");
            if (object.definition.get("stateDefinition").containsKey(field)) {
                fieldDefinition = (float[])object.definition.get("stateDefinition").get(field);
            } else {
                fieldDefinition = (float[])object.definition.get("actionDefinition").get(field);
            }
            if (fieldDefinition == null) {
                // attempt to resolve by removing axial component
                if (field.contains("X")) {
                    fieldDefinition = getDefinitionConstants(object, field.replace("X", ""));
                } else if (field.contains("Y")) {
                    fieldDefinition = getDefinitionConstants(object, field.replace("Y", ""));
                }
            }
            if ((fieldDefinition == null) || (fieldDefinition == defaultDefinition)) {
                // System.out.println("FIELD NOT DEFINED after attemping resolution: " + field);
                // printDefinition(object.definition);
                return defaultDefinition;
            }
            return fieldDefinition;
        }


        public StateActionClass set(String field, int value) {
            valueMap.put(field, value);
            return this;
        }

        public static StateActionClass set(StateActionClass object, String field, int value){
            object.valueMap.put(field, value);
            return object;
        }

        public StateActionClass setDouble(String field, double value) {
            int realValue = 0;
            float[] definitionConstants = getDefinitionConstants(this, field);
            /*
            if (value <= definitionConstants[0])
                value = definitionConstants[0] + (definitionConstants[2] / 2);
            if (value >= definitionConstants[1])
                value = definitionConstants[1] - (definitionConstants[2] / 2);
             */

            if (field.contains("gimbal") || field.contains("angle"))
                realValue = group_angle_by_precision(value, definitionConstants[2]);
            //else if (field.contains("position"))
            //    realValue = group_by_precision_symmetric(value, getPrecisionConstant(field));
            else
                realValue = group_by_precision(value, definitionConstants[2]);
            valueMap.put(field, realValue);
            return this;
        }

        protected String stringifyStateOrAction(){
            String simpleClassName = this.getClass().getSimpleName();
            String definitionField = "";
            if (simpleClassName.toLowerCase().equals("state")) definitionField = "stateDefinition";
            else if (simpleClassName.toLowerCase().equals("action")) definitionField = "actionDefinition";
            StringBuilder stringBuilder = new StringBuilder();
            stringBuilder.append(simpleClassName);
            stringBuilder.append("(");
            ArrayList<String> keys = new ArrayList<>(
                    ((HashMap<String, HashMap>)definition.get(definitionField)).keySet()
            );
            for (int i = 0; i < keys.size(); i++) {
                String field = keys.get(i);
                stringBuilder.append(field);
                stringBuilder.append(": ");
                stringBuilder.append(this.getDouble(field));
                stringBuilder.append(" -> ");
                stringBuilder.append(this.get(field));
                stringBuilder.append(", ");
            }
            stringBuilder.append(")");
            String resultString = stringBuilder.toString();
            return resultString.replace(", )", ")");
        }

        public static void applyDeepcopy(StateActionClass fromObject, StateActionClass toObject, HashMap<String, LinkedHashMap> definition) {
            toObject.definition = definition;
            toObject.valueMap = new HashMap<>();
            if (fromObject.symmetry != null)
                toObject.symmetry = fromObject.symmetry + "";
            for (Map.Entry<String, Integer> entry: fromObject.valueMap.entrySet()) {
                toObject.valueMap.put(entry.getKey() + "", entry.getValue() + 0);
            }
            if (fromObject.definition == toObject.definition) {
                toObject.runMDPDefinitionFormulas();
            } else {
                System.out.println("NEVER DO THIS!!!");
            }
        }
    }

    // Required data structures.

    public static class State extends StateActionClass {
        public State(SimulationStatus status){
            if (status == null) return;
            constructorCode(status, landerDefinition);
            runALLMDPDefinitionFormulas();
        }

        public State(SimulationStatus status, HashMap<String, LinkedHashMap> definition) {
            if (status == null) return;
            constructorCode(status, definition);
        }

        private void constructorCode(SimulationStatus status, HashMap<String, LinkedHashMap> definition) {
            if (status == null) return;
            this.definition = definition;
            Coordinate rocketDirection = convertRocketStatusQuaternionToDirection(status);

            setDouble("positionX", status.getRocketPosition().x);
            setDouble("positionY", status.getRocketPosition().y);
            setDouble("positionZ", status.getRocketPosition().z);

            setDouble("velocityX", status.getRocketVelocity().x);
            setDouble("velocityY", status.getRocketVelocity().y);
            setDouble("velocityZ", status.getRocketVelocity().z);
            //setVelocity(Math.signum(status.getRocketVelocity().z) * status.getRocketVelocity().length());

            // new angle approach
            double angleX = Math.asin(rocketDirection.x);
            double angleY = Math.asin(rocketDirection.y);

            double _45_deg = Math.asin(Math.sqrt(2)/2.0) - 0.0000001;
            if (rocketDirection.z <= 0.0 || Math.abs(angleX) >= _45_deg) {
                angleX = _45_deg * Math.signum(rocketDirection.x);
            }
            if (rocketDirection.z <= 0.0 || Math.abs(angleY) >= _45_deg) {
                angleY = _45_deg * Math.signum(rocketDirection.y);
            }
            setDouble("angleX", angleX);
            setDouble("angleY", angleY);
            setDouble("angleZ", Math.sqrt(1.0 - angleX * angleX - angleY * angleY));

            // original angle approach
            //setDouble("angleX", Math.acos(rocketDirection.x) * Math.signum(rocketDirection.y));
            //setDouble("angleZ", Math.acos(rocketDirection.z));

            // NOTE THAT THIS IS INTENTIONALLY FLIPPED!!!
            setDouble("angleVelocityX", status.getRocketRotationVelocity().y);
            setDouble("angleVelocityY", -status.getRocketRotationVelocity().x);

            setDouble("time", status.getSimulationTime());

            runMDPDefinitionFormulas();
        }


        @Override
        public int hashCode() {
            return DynamicValueFunctionTable.computeIndexState(this);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            State other = (State) obj;
            boolean equivalent = true;
            for (Object objectField : definition.get("stateDefinition").keySet()) {
                String stateField = (String)objectField;
                equivalent = equivalent && this.get(stateField) == other.get(stateField);
            }
            return equivalent;
        }

        @Override
        public String toString() {
            return stringifyStateOrAction();
        }

        public State deepcopy() {
            return deepcopy(this.definition);
        }

        public State deepcopy(HashMap<String, LinkedHashMap> definition) {
            State newState = new State(null);
            applyDeepcopy(this, newState, definition);
            return newState;
        }
    }

    public static class Action extends StateActionClass {
        public Action(HashMap<String, Integer> values, HashMap<String, LinkedHashMap> definition) {
            constructorCodeInts(values, definition);
        }

        private Action(float thrust, float gimbalX, float gimbalY, HashMap<String, LinkedHashMap> definition) {
            HashMap<String, Float> values = new HashMap<String, Float>() {{
                put("thrust", thrust);
                put("gimbalX", gimbalX);
                put("gimbalY", gimbalY);
            }};
            constructorCodeDoubles(values, definition);
        }

        private void constructorCodeInts(HashMap<String, Integer> values, HashMap<String, LinkedHashMap> definition) {
            this.definition = definition;
            for (Map.Entry<String, Integer> entry: values.entrySet())
                set(entry.getKey(), entry.getValue());
            runMDPDefinitionFormulas();
        }

        private void constructorCodeDoubles(HashMap<String, Float> values, HashMap<String, LinkedHashMap> definition) {
            System.out.println("This constructor is not recommended because it loses precision!!!");
            this.definition = definition;
            for (Map.Entry<String, Float> entry: values.entrySet())
                setDouble(entry.getKey(), entry.getValue());
            runMDPDefinitionFormulas();
        }

        @Override
        public int hashCode() {
            return DynamicValueFunctionTable.computeIndexAction(this);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            Action other = (Action) obj;
            boolean equivalent = true;
            for (Object objectField : definition.get("actionDefinition").keySet()) {
                String actionField = (String)objectField;
                equivalent = equivalent && this.get(actionField) == other.get(actionField);
            }
            return equivalent;
        }

        @Override
        public String toString() {
            return stringifyStateOrAction();
        }

        public Action deepcopy() {
            return deepcopy(this.definition);
        }

        public Action deepcopy(HashMap<String, LinkedHashMap> definition) {
            Action newAction = new Action(0, 0,0 , this.definition);
            applyDeepcopy(this, newAction, definition);
            return newAction;
        }

        public void applyDefinitionValuesToState(State state) {
            HashSet<String> symmetryAxes = new HashSet<>();
            if (definition.get("meta").containsKey("symmetry")) {
                String[] symmetryAxesArray = ((String) definition.get("meta").get("symmetry")).split(",");
                symmetryAxes = new HashSet<>(Arrays.asList(symmetryAxesArray));
            }

            for (Object objectField : definition.get("actionDefinition").keySet()) {
                String actionField = (String)objectField;
                double actionFieldValue = getDouble(actionField);
                if (symmetryAxes.contains(actionField)) {  // double setting the value for certainty
                    state.setDouble(actionField + symmetry, actionFieldValue);
                }

                if (state.definition == definition) {
                    state.setDouble(actionField, actionFieldValue);
                } else { // different definitions! DO NOT OVERRIDE THE VALUE unless not part of the actionDefinition
                    if (!state.definition.get("actionDefinition").containsKey(actionField))
                        state.setDouble(actionField, actionFieldValue);
                }
            }
        }
    }

    public static class CoupledStates extends ArrayList<State> {
        boolean frozen = false;
        String toStringNames = null;
        public CoupledStates(State... _states) {
            for (State s : _states) {
                super.add(s);
            }
        }
        public CoupledStates deepcopy() {
            ArrayList<State> copiedStateArrayList = new ArrayList<>();
            for (State s: this) {
                copiedStateArrayList.add(s.deepcopy());
            }
            return new CoupledStates(copiedStateArrayList.toArray(new State[copiedStateArrayList.size()]));
        }
        public void freeze() { frozen = true; }
        public String toStringNames() {
            if (!frozen || toStringNames == null) {
                StringBuilder tempNames = new StringBuilder();
                for (State s: this)
                    tempNames.append(s.definition.get("meta").get("name"));
                toStringNames = tempNames.toString();
            }
            return toStringNames;
        }
        @Override
        public String toString() {
            return toStringNames();
        }
        @Override
        public boolean add(State state) {
            if (frozen) {
                System.out.println("Cannot add state to a frozen CoupledStates!");
                return false;
            }
            return super.add(state);
        }
        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            CoupledStates other = (CoupledStates) obj;
            if (this.size() != other.size()) return false;

            boolean equals = true;
            for (int i = 0; i < this.size(); i++)
                equals = equals && this.get(i).equals(other.get(i));
            return equals;
        }
    }

    public static class CoupledActions extends ArrayList<Action> {
        private Action thrustAction = null;
        private Action gimbalXAction = null;
        private Action gimbalYAction = null;


        public CoupledActions(Action... _actions) {
            constructorCode(_actions);
        }

        public CoupledActions(@NotNull Action thrustAction, @NotNull Action gimbalXAction, @NotNull Action gimbalYAction) {
            ArrayList<Action> actions = new ArrayList<>();
            actions.add(thrustAction);
            actions.add(gimbalXAction);
            actions.add(gimbalYAction);
            constructorCode(actions.toArray(new Action[actions.size()]));
        }

        private void constructorCode(Action... _actions) {
            for (Action a : _actions) {
                add(a);
            }
        }

        @Override
        public boolean add(Action action) {
            if (action.definition.get("actionDefinition").containsKey("thrust")) {
                thrustAction = action;
            } else if (action.definition.get("actionDefinition").containsKey("gimbalX") || (action.symmetry.equals("X") && action.definition.get("actionDefinition").containsKey("gimbal"))) {
                gimbalXAction = action;
            } else if (action.definition.get("actionDefinition").containsKey("gimbalY") || (action.symmetry.equals("Y") && action.definition.get("actionDefinition").containsKey("gimbal"))) {
                gimbalYAction = action;
            }
            return super.add(action);
        }

        private Action getAction(String field) {
            switch (field) {
                case "thrust":
                    return this.thrustAction;
                case "gimbalX":
                    return this.gimbalXAction;
                case "gimbalY":
                    return this.gimbalYAction;
            }
            System.out.println("attempting to pull field not contained in actions: " + field);
            return this.thrustAction;
        }

        public double getDouble(String field) {
            Action action = getAction(field);
            if (action == null) return 0.0;
            return action.getDouble(field);
        }

        public double getDouble(String field, String axis) {
            return getAction(field + axis).getDouble(field);
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj)
                return true;
            if (obj == null)
                return false;
            if (getClass() != obj.getClass())
                return false;
            CoupledActions other = (CoupledActions) obj;
            if (this.size() != other.size()) return false;

            boolean equals = true;
            for (int i = 0; i < this.size(); i++)
                equals = equals && this.get(i).equals(other.get(i));
            return equals;
        }
    }


    public static Quaternion getConjugateQuaternion(Quaternion quaternion) {
        return new Quaternion(quaternion.getW(), -quaternion.getX(), -quaternion.getY(), -quaternion.getZ());
    }

    public static Coordinate convertRocketStatusQuaternionToDirection(SimulationStatus status) {
        return status.getRocketOrientationQuaternion().rotateZ();
    }
}


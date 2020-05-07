package net.sf.openrocket.simulation.extension.impl.rocketlander.customexpressions;

import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple;

// public interfacing class
public final class Expression extends Term {
    Function function;
    String functionString = "";
    public Expression(Function function) {
        super("");
        this.function = function;
    }
    public float evaluateFloat(StateActionTuple.StateActionClass object) { return (float)function.evaluate(object); }
    @Override
    public double evaluate(StateActionTuple.StateActionClass object) {
        return function.evaluate(object);
    }
    @Override
    public double evaluateBestGuess(StateActionTuple.StateActionClass primary, StateActionTuple.StateActionClass fallback) {
        return function.evaluateBestGuess(primary, fallback);
    }
    @Override
    public String toString() {
        if (functionString.equals("")) { functionString = function.toString(); }
        return functionString;
    }
    public boolean isAssignmentExpression() {
        return (String.valueOf(this.toString().charAt(0)).equals("("));
    }
    public String getSymmetry() {
        if (!isAssignmentExpression()) return null;
        String stringExpression = toString();
        return stringExpression.substring(stringExpression.length() - 2, stringExpression.length() - 1);
    }
}

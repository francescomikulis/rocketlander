package net.sf.openrocket.simulation.extension.impl.rocketlander.customexpressions;

import net.sf.openrocket.simulation.extension.impl.rocketlander.StateActionTuple;

abstract class Term {
    String identifier;
    int sign = 1;
    public Term(String identifier) {
        this.identifier = identifier;
    }

    @Override
    public int hashCode() {
        return identifier.hashCode();
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        Term other = (Term) obj;
        return this.identifier.equals(other.identifier) && this.sign == other.sign;
    }

    @Override
    public String toString() { return "Term: " + identifier; }

    public abstract double evaluate(StateActionTuple.StateActionClass object);
    public abstract double evaluateBestGuess(StateActionTuple.StateActionClass primary, StateActionTuple.StateActionClass fallback);
}
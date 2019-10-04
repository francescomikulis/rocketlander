package net.sf.openrocket.simulation.extension.impl;

import net.sf.openrocket.simulation.extension.impl.RLModel.*;

import java.io.Serializable;

public class StateActionTuple implements Serializable {
    public State state;
    public Action action;
    public StateActionTuple(State state, Action action) {
        this.state = state;
        this.action = action;
    }

    @Override
    public String toString() {
        return ("(" + state.toString() + ", " + action.toString() + ")");
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
        if (
                (this.action.thrust != other.action.thrust) ||
                (this.action.gimble_x != other.action.gimble_x) ||
                (this.action.gimble_y != other.action.gimble_y) ||
                (this.state.altitude != other.state.altitude) ||
                (this.state.velocity != other.state.velocity)
        )
            return false;
        return true;
    }
}

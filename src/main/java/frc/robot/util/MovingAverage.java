package frc.robot.util;

import java.util.ArrayDeque;
import java.util.Deque;

public class MovingAverage { //For finding averages of the most recent 
    //certain number of values in a discrete continuous flow of values
    Deque<Double> dq; //Double ended queue: a linear storage that can 
    //be edited from both ends, pronounced "deck"
    int size;
    double sum;

    public MovingAverage(int size) {
        dq = new ArrayDeque<>();
        this.size = size;
        sum = 0;
    }

    public void nextVal(double val) { //Add value, remove oldest value if already at size
        sum += val;
        dq.add(val);
        if (dq.size() > size) {
            sum -= dq.getFirst();
            dq.removeFirst();
        }
    }

    public double get() { //Get average of values currently stored
        if (dq.size() == 0)
            return 0;
        return sum / dq.size();
    }
}
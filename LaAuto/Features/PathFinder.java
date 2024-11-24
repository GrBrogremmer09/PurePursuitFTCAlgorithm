package org.firstinspires.ftc.teamcode.LaAuto.Features;

import android.util.Pair;

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Vector;

public class PathFinder {

    public class Island implements Comparable<Island> {
        public double f, g, h, x, y; // i = intersections num
        public int i;

        Island(double f, double g, double h, double x, double y, int i) {
            this.f = f;
            this.g = g;
            this.h = h;
            this.x = x;
            this.y = y;
            this.i = i;
        }

        @Override
        public int compareTo(Island other) {
            return Double.compare(this.g, other.g);
        }

        public double getF(){
            return f;
        }

        public double getG(){
            return g;
        }

        public double getH(){
            return h;
        }

        public double getX(){
            return x;
        }

        public double getY(){
            return y;
        }

        public int getI(){
            return i;
        }
    }

    private Island end = new Island(0.0, 0.0, 0.0, 15, 78, 2);

    private List<Pair <Point, Integer>> rawList = new ArrayList <>();
    private List<Island> prossesedList = new ArrayList <>();
    private Vector<Island> currentSuccessors = new Vector <>();
    private List<Point> candidatesList = new ArrayList <>();

    private void calculateHeuristics() {
        double h;

        for (int i = 0; i < rawList.size(); ++i) {
            h = Math.hypot((end.x - rawList.get(i).first.x), (end.y - rawList.get(i).first.y));

            prossesedList.set(i, new Island(0.0, 0.0, h, rawList.get(i).first.x,
                                            rawList.get(i).first.y, rawList.get(i).second));
        }
    }

    private void findSuccessors(Island currentIsland) {
        double g;

        for (int i = 0; i < prossesedList.size() - 1; ++i) {
            g = Math.hypot((prossesedList.get(i).getX() - currentIsland.x),
                           (prossesedList.get(i).getY() - currentIsland.y));

            currentSuccessors.add(new Island(0.0, g,
                                                prossesedList.get(i).getH(),
                                                prossesedList.get(i).getX(),
                                                prossesedList.get(i).getY(),
                                                prossesedList.get(i).getI()
            ));
        }

        Collections.sort(currentSuccessors);

        while (currentSuccessors.size() > currentIsland.getI()) {
            currentSuccessors.remove(currentSuccessors.lastElement());
        }
    }

}

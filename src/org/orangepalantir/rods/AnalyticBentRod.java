package org.orangepalantir.rods;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Matt on 29/09/16.
 */
public class AnalyticBentRod implements DrawableRod{
    List<Point> points = new ArrayList<>();

    double A,B;
    int N = 100;
    public AnalyticBentRod(double length, double force){
        A = force/6.0;
        B = force*length/4;
        double ds = length/(N-1);
        for(int i = 0; i<N; i++){
            double s = ds*i - length/2;
            points.add(new Point(s, getHeight(s), 0));
        }
    }

    double getHeight(double s){
        if(s>0){
            return -A*s*s*s + B*s*s;
        } else{
            return A*s*s*s + B*s*s;
        }
    }






    @Override
    public List<Point> getPoints() {
        return points;
    }


}

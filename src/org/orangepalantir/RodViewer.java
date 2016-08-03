package org.orangepalantir;

import javax.swing.JFrame;
import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Path2D;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by melkor on 7/31/16.
 */
public class RodViewer {
    List<DrawableRod> rods = new ArrayList<>();
    BufferedImage display;
    String status = "";
    int width = 512;
    int height = 512;

    Dimension a = new Dimension(width, height);

    JPanel panel = new JPanel(){
        protected void paintComponent(Graphics g){
            if(display!=null){
                g.drawImage(display, 0, 0, this);
            }
        }

        @Override
        public Dimension getPreferredSize(){
            return a;
        }
        @Override
        public Dimension getMaximumSize(){
            return a;
        }
        @Override
        public  Dimension getMinimumSize(){
            return a;
        }

    };


    double simWidth = 3.0;
    double simHeight = 3.0;

    public RodViewer(){

    }
    public void addRod(DrawableRod rod){
        rods.add(rod);
    }

    public void buildGui(){
        JFrame frame= new JFrame();
        frame.setContentPane(panel);
        frame.pack();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setVisible(true);
    }


    public void repaint(){
        BufferedImage drawing = new BufferedImage(800, 600, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2d = (Graphics2D)drawing.getGraphics();
        g2d.fillRect(0, 0, width, height);
        double[] Axy = new double[2];
        double[] Bxy = new double[2];
        g2d.setColor(Color.BLACK);
        g2d.drawString(status, 50, 50);
        for(DrawableRod rod: rods){
            List<Point> points = rod.getPoints();
            for(int i = 0;i<points.size()-1; i++){

                getTransformed(points.get(i), Axy);
                getTransformed(points.get(i+1), Bxy);
                g2d.setColor(Color.RED);
                g2d.drawLine((int)Axy[0], (int)Axy[1],(int)Bxy[0], (int)Bxy[1]);
                g2d.setColor(Color.BLUE);
                g2d.fillOval((int)Axy[0]-2, (int)Axy[1] - 2, 4, 4);

            }

            getTransformed(points.get(points.size()-1), Axy);
            g2d.setColor(Color.BLUE);
            g2d.fillOval((int)Axy[0]-2, (int)Axy[1] - 2, 4, 4);

        }
        display = drawing;
        panel.repaint();
    }

    void getTransformed(Point r, double[] tran){

        tran[0] = (r.x + simWidth/2) *width/simWidth;
        tran[1] = (r.y + simHeight/2)* height/simHeight;

    }


    public void setStatus(String s) {
        status = s;
    }

    public boolean displays() {
        return true;
    }
}

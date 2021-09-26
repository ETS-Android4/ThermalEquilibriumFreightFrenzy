package org.firstinspires.ftc.teamcode.geometry;

public class pathVector3D extends Vector3D {

    // steady state error tolerance
    protected double steady_state_tolerance = 1.5;

    public pathVector3D(double x, double y, double angle) {
        super(x, y, angle);
    }

    public pathVector3D(double x, double y, double angle, double steady_state_tolerance) {
        super(x,y,angle);
        this.steady_state_tolerance = steady_state_tolerance;
    }
    public pathVector3D(double x, double y, double angle, double steady_state_tolerance, boolean accelerate_quicker) {
        super(x,y,angle,accelerate_quicker);
        this.steady_state_tolerance = steady_state_tolerance;

    }

    public pathVector3D(Vector3D pos) {
        super(pos.x,pos.y,pos.angle.getRadians());

    }


    public double getSteady_state_tolerance() {
        return steady_state_tolerance;
    }

}

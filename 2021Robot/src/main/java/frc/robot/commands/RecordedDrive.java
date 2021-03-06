package frc.robot.commands;

import com.fasterxml.jackson.databind.annotation.JsonAppend.Prop;

public class RecordedDrive implements java.io.Serializable   {

    public int tick;
    public double xSpeed;
    public double zRotation;
    public boolean runwheel;

    public RecordedDrive(int m_tick, double m_xSpeed, double m_zRotation, boolean m_runwheel){
        tick = m_tick;
        xSpeed = m_xSpeed;
        zRotation = m_zRotation;
        runwheel = m_runwheel;
    }

}
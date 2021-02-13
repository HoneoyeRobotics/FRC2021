package frc.robot.commands;

import com.fasterxml.jackson.databind.annotation.JsonAppend.Prop;

public class RecordedDrive {

    public int tick;
    public double xSpeed;
    public double zRotation;

    public RecordedDrive(int m_tick, double m_xSpeed, double m_zRotation){
        tick = m_tick;
        xSpeed = m_xSpeed;
        zRotation = m_zRotation;
    }

}
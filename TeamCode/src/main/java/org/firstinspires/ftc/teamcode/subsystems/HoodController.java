package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class HoodController {
    private final Servo left;
    private final Servo right;
    private final double minPos;
    private final double maxPos;
    private final double leftStep;
    private final double rightStep;
    private final long debounceMs;

    private double leftPos;
    private double rightPos;
    private long lastLeftMs = 0L;
    private long lastRightMs = 0L;

    public HoodController(Servo left, Servo right,
                          double initialLeft, double initialRight,
                          double minPos, double maxPos,
                          double leftStep, double rightStep,
                          long debounceMs) {
        this.left = left;
        this.right = right;
        this.minPos = minPos;
        this.maxPos = maxPos;
        this.leftStep = leftStep;
        this.rightStep = rightStep;
        this.debounceMs = debounceMs;
        setLeftPosition(initialLeft);
        setRightPosition(initialRight);
    }

    public void nudgeLeftUp(long nowMs) {
        if (nowMs - lastLeftMs < debounceMs) return;
        setLeftPosition(leftPos + leftStep);
        lastLeftMs = nowMs;
    }

    public void nudgeLeftDown(long nowMs) {
        if (nowMs - lastLeftMs < debounceMs) return;
        setLeftPosition(leftPos - leftStep);
        lastLeftMs = nowMs;
    }

    public void nudgeRightUp(long nowMs) {
        if (nowMs - lastRightMs < debounceMs) return;
        setRightPosition(rightPos + rightStep);
        lastRightMs = nowMs;
    }

    public void nudgeRightDown(long nowMs) {
        if (nowMs - lastRightMs < debounceMs) return;
        setRightPosition(rightPos - rightStep);
        lastRightMs = nowMs;
    }

    public void setRightPosition(double pos) {
        rightPos = clip(pos);
        right.setPosition(rightPos);
    }

    public double getLeftPos() { return leftPos; }
    public double getRightPos() { return rightPos; }

    private void setLeftPosition(double pos) {
        leftPos = clip(pos);
        left.setPosition(leftPos);
    }

    private double clip(double v) {
        return Range.clip(v, minPos, maxPos);
    }
}
package org.firstinspires.ftc.teamcode.utility.VoltageCompensator;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class DcMotorExStable extends DcMotorImplEx implements DcMotorEx {
    public DcMotorExStable(DcMotorEx motorEx) {
        super(motorEx.getController(), motorEx.getPortNumber(), Direction.FORWARD);
    }


    /**
     * Set the current motor power
     *
     * @param power from -1.0 to 1.0
     */
    @Override
    synchronized public void setPower(double power) {
        // Power must be positive when in RUN_TO_POSITION mode : in that mode, the
        // *direction* of rotation is controlled instead by the relative positioning
        // of the current and target positions.
        if ( getMode() == RunMode.RUN_TO_POSITION )
            power = Math.abs(power);
        else
            power = adjustPower(power);

        if( power > 0 ) // make sure the motor treats approaching 0 power like it is really approaching 0 power
            power += minMovePower;
        else if( power < 0 )
            power -= minMovePower;

        if( VoltageCompensator.isSetup() ) // compensate for robot voltage if able
            power *= VoltageCompensator.getRobotCompensationMult();

        internalSetPower(power);
    }


    private double minMovePower;
    public void setMinMovePower( double minMovePower ){
        this.minMovePower = minMovePower;
    }
}

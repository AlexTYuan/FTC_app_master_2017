package org.firstinspires.ftc.teamcode;

import android.location.Address;
import android.renderscript.Type;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

/**
 * Created by alexyuan on 11/17/17.
 */

@I2cSensor(name = "L3G4200D Gyroscope", description = "Gyroscope from Parallax", xmlTag = "L3G4200D")

public class L3G4200D extends I2cDeviceSynchDevice<I2cDeviceSynch>{

    public short getRawX() {
        return readShort(Register.OUT_X_H);
    }

    public short getRawY() {
        return readShort(Register.OUT_Y_H);
    }

    public short getRawZ() {
        return readShort(Register.OUT_Z_H);
    }

    public double getX() {
        short dataRaw = getRawX();

        // The first 3 bits are alert bits that we don't care about here. We need to force them to
        // be 0s or 1s if the number is positive or negative depending on the sign
        if((dataRaw & 0x1000) == 0x1000) // Negative
            dataRaw |= 0xE000;
        else // Positive
            dataRaw &= 0x1FFF;

        // Multiply by least significant bit (2^-4 = 1/16) to scale
        return dataRaw / 16.0;
    }

    public double getY() {
        short dataRaw = getRawY();

        // The first 3 bits are alert bits that we don't care about here. We need to force them to
        // be 0s or 1s if the number is positive or negative depending on the sign
        if((dataRaw & 0x1000) == 0x1000) // Negative
            dataRaw |= 0xE000;
        else // Positive
            dataRaw &= 0x1FFF;

        // Multiply by least significant bit (2^-4 = 1/16) to scale
        return dataRaw / 16.0;
    }

    public double getZ() {
        short dataRaw = getRawZ();

        // The first 3 bits are alert bits that we don't care about here. We need to force them to
        // be 0s or 1s if the number is positive or negative depending on the sign
        if((dataRaw & 0x1000) == 0x1000) // Negative
            dataRaw |= 0xE000;
        else // Positive
            dataRaw &= 0x1FFF;

        // Multiply by least significant bit (2^-4 = 1/16) to scale
        return dataRaw / 16.0;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.valueOf("Parallax");
    }

    @Override
    protected  synchronized  boolean doInitialize() {
        return true;
    }

    @Override
    public String getDeviceName() {
        return "Parallax Gyroscope Module 3-Axis L3G4200D";
    }

    public enum Register {
        FIRST(0),
        WHO_AM_I(0x0F),
        CTRL_REG1(0X20),
        CTRL_REG2(0X21),
        CTRL_REG3(0X22),
        CTRL_REG4(0X23),
        CTRL_REG5(0X24),
        REFERENCE(0X25),
        OUT_TEMP(0X26),
        STATUS_REG(0X27),
        OUT_X_L(0X28),
        OUT_X_H(0X29),
        OUT_Y_L(0X2A),
        OUT_Y_H(0X2B),
        OUT_Z_L(0X2C),
        OUT_Z_H(0X2D),
        FIFO_CTRL_REG(0X2E),
        FIFO_SRC_REG(0X2F),
        INT1_CFG(0X30),
        INT1_SRC(0X31),
        INT1_TSH_XH(0X32),
        INT1_TSH_XL(0X33),
        INT1_TSH_YH(0X34),
        INT1_TSH_YL(0X35),
        INT1_TSH_ZH(0X36),
        INT1_TSH_ZL(0X37),
        INT1_DURATION(0X38),
        LAST(INT1_DURATION.bVal);

        public int bVal;

        Register(int bval) {
            this.bVal = bval;
        }
    }

    protected void setOptimalReadWindow() {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected  void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public short getManufacturerIDRaw() {
        return readShort(Register.WHO_AM_I);
    }

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0xD3);

    public L3G4200D(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.teamcode.CCITTCrc16;
import org.firstinspires.ftc.teamcode.DbgLog;

import java.nio.ByteOrder;
import java.util.Arrays;
import java.util.EnumSet;

/**
 * Created by Cherry Pi on 12/27/2018.
 */
@DeviceProperties(name = "OpticalFlowBoard", description = "Optical Flow Board By Team 11411 Cherry Pi", xmlTag = "OpticalFlowBoard")
@I2cDeviceType
public class OpticalFlowBoard extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    private static final byte EXPECTED_PRODUCT_ID = 0x49;
    private static final byte EXPECTED_CP_PRODUCT_ID = 0x42;

    private static final int WHITE_LED = 1;
    private static final int BLUE_LED = 2;
    private static final int GREEN_LED = 4;
    private static final int RED_LED = 8;

    private enum Register {
        PRODUCT_ID(0x00), // RO
        REVISION_ID(0x01), // RO
        CP_PRODUCT_ID(0x20), // RO
        CP_REVISION_ID(0x21), // RO
        LEDSTATUS(0x23), // R/W
        READ_RELATIVE(0x24), // RO
        REREAD_RELATIVE(0x25), // RO
        ZERO_ABSOLUTE(0x26), // WO
        READ_ABSOLUTE(0x27), // RO
        RESET(0x28); // WO

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    public class Delta {
        private int _x_offset;
        private int _y_offset;
        private int _timestamp;
        private int _x_overflow_count;
        private int _y_overflow_count;

        public Delta(int timestamp, short x_offset, short y_offset, byte overflow) {
            _timestamp = timestamp;
            _x_offset = x_offset;
            _y_offset = y_offset;
            _x_overflow_count = (overflow >> 4) & 0xf;
            _y_overflow_count = overflow & 0xf;
        }

        public int getXOffset() {
            return _x_offset;
        }

        public int getYOffset() {
            return _y_offset;
        }

        public int getTimestamp() {
            return _timestamp;
        }

        public int getXOverlowCount() {
            return _x_overflow_count;
        }

        public int getYOverflowCount() {
            return _y_overflow_count;
        }
    }

    public class Absolute {
        private int _x;
        private int _y;
        private int _timestamp;

        public Absolute(int timestamp, int x, int y) {
            _timestamp = timestamp;
            _x = x;
            _y = y;
        }

        public int getX() {
            return _x;
        }

        public int getY() {
            return _y;
        }

        public int getTimestamp() {
            return _timestamp;
        }
    }

    public enum LED {
        WHITE,
        BLUE,
        GREEN,
        RED
    }

    private static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x29);

    public OpticalFlowBoard(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        I2cDeviceSynch.ReadWindow window = deviceClient.getReadWindow();
        if (window == null) {
            DbgLog.msg("Read window is null");
        } else {
            DbgLog.msg("Read window first %d count %d", window.getRegisterFirst(), window.getRegisterCount());
        }
        byte id = getProductIDRaw();
        if (id != EXPECTED_PRODUCT_ID) {
            DbgLog.error("Product id doesn't match. Expected 0x49 got %02x", id);
            return false;
        }
        id = getCPProductIDRaw();
        if (id != EXPECTED_CP_PRODUCT_ID) {
            DbgLog.error("CP Product id doesn't match. Expected 0x42 got %02x", id);
            return false;
        }

        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "OpticalFlowBoard";
    }

    public byte getProductIDRaw() {
        DbgLog.msg("Reading product id");
        byte b = deviceClient.read8(Register.PRODUCT_ID.bVal);
        return b;
    }

    public byte getCPProductIDRaw() {
        DbgLog.msg("Reading cp product id");
        byte b = deviceClient.read8(Register.CP_PRODUCT_ID.bVal);
        return b;
    }

    public Delta readRelative() {
        byte[] bytes = deviceClient.read(Register.READ_RELATIVE.bVal, 11);
        return createDelta(bytes);
    }

    public Delta rereadRelative() {
        byte[] bytes = deviceClient.read(Register.REREAD_RELATIVE.bVal, 11);
        return createDelta(bytes);
    }

    public Absolute readAbsolute() {
        byte[] bytes = deviceClient.read(Register.READ_ABSOLUTE.bVal, 14);
        return createAbsolute(bytes);
    }

    public void zeroAbsolute() {
        deviceClient.write8(Register.ZERO_ABSOLUTE.bVal, 0);
    }

    public void reset() {
        deviceClient.write8(Register.RESET.bVal, 0);
    }

    public void setLedStatus(boolean whiteOn, boolean redOn, boolean greenOn, boolean blueOn) {
        byte status = 0;
        if (redOn) {
            status |= RED_LED;
        }
        if (greenOn) {
            status |= GREEN_LED;
        }
        if (blueOn) {
            status |= BLUE_LED;
        }
        if (whiteOn) {
            status |= WHITE_LED;
        }
        deviceClient.write8(Register.LEDSTATUS.bVal, status);
    }

    public void setLedStatus(EnumSet<LED> ledsOn) {
        setLedStatus(ledsOn.contains(LED.WHITE), ledsOn.contains(LED.RED), ledsOn.contains(LED.GREEN), ledsOn.contains(LED.BLUE));
    }

    public byte getLedStatusRaw() {
        return deviceClient.read8(Register.LEDSTATUS.bVal);
    }

    public EnumSet<LED> getLedStatus() {
        EnumSet<LED> result = EnumSet.noneOf(LED.class);
        byte status = getLedStatusRaw();
        if ((status & WHITE_LED) == WHITE_LED) {
            result.add(LED.WHITE);
        }
        if ((status & BLUE_LED) == BLUE_LED) {
            result.add(LED.BLUE);
        }
        if ((status & GREEN_LED) == GREEN_LED) {
            result.add(LED.GREEN);
        }
        if ((status & RED_LED) == RED_LED) {
            result.add(LED.RED);
        }

        return result;
    }

    private Delta createDelta(byte[] bytes) {
        if (bytes.length != 11) {
            return null;
        }
        int timestamp = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bytes, 0, 4), ByteOrder.LITTLE_ENDIAN);
        short x_offset = TypeConversion.byteArrayToShort(Arrays.copyOfRange(bytes, 4, 6), ByteOrder.LITTLE_ENDIAN);
        short y_offset = TypeConversion.byteArrayToShort(Arrays.copyOfRange(bytes, 6, 8), ByteOrder.LITTLE_ENDIAN);
        byte overflow = bytes[8];
        short crc = TypeConversion.byteArrayToShort(Arrays.copyOfRange(bytes, 9, 11), ByteOrder.LITTLE_ENDIAN);

        if (crc != calculateCrc(Arrays.copyOfRange(bytes, 0, 9))) {
            return null;
        }

        return new Delta(timestamp, x_offset, y_offset, overflow);
    }

    private Absolute createAbsolute(byte[] bytes) {
        if (bytes.length != 14) {
            DbgLog.msg("Bytes received are %d expected 14", bytes.length);
            return null;
        }
        int timestamp = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bytes, 0, 4), ByteOrder.LITTLE_ENDIAN);
        int x = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bytes, 4, 8), ByteOrder.LITTLE_ENDIAN);
        int y = TypeConversion.byteArrayToInt(Arrays.copyOfRange(bytes, 8, 12), ByteOrder.LITTLE_ENDIAN);
        short crc = TypeConversion.byteArrayToShort(Arrays.copyOfRange(bytes, 12, 14), ByteOrder.LITTLE_ENDIAN);

        short calculateCrc = calculateCrc(Arrays.copyOfRange(bytes, 0, 12));
        if (crc != calculateCrc) {
            DbgLog.msg("Crc mismatch: got %d expected %d", crc, calculateCrc);
            return null;
        }

        return new Absolute(timestamp, x, y);
    }

    private short calculateCrc(byte[] bytes) {
        return (short) CCITTCrc16.calculate(bytes);
    }
}

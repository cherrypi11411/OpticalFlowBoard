package org.firstinspires.ftc.teamcode;

public class CCITTCrc16 {

    private static final int POLYNOMIAL = 0x8408;
    private static final int PRESET = 0;
    static private int[] tab;

    static {
        tab = new int[256];
        for (int i = 0; i < 256; i++) {
            tab[i] = initial((byte) i);
        }
    }

    private static int initial(byte c) {
        int crc = 0;
        for (int j = 0; j < 8; j++) {
            if (((crc ^ c) & 1) == 1) {
                crc = ((crc >> 1) ^ POLYNOMIAL);
            } else {
                crc = (crc >> 1);
            }
            c = (byte) (c >> 1);
        }
        return crc;
    }

    private static int update_crc(int crc, byte c) {
        int cc = (0xff & c);

        int tmp = (crc ^ cc);
        crc = (crc >> 8) ^ tab[tmp & 0xff];

        return crc;
    }

    private static int swab(int n) {
        return (((n & 0xFF00) >> 8) + ((n & 0xFF) << 8));
    }

    public static int calculate(String str) {
        return calculate(str.getBytes());
    }

    public static int calculate(byte... i) {
        int crc = PRESET;
        for (byte c : i) {
            crc = update_crc(crc, c);
        }
        return swab(crc);
    }
}

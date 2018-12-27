#include "CCITTCrc16.h"

#define PRESET      0
#define POLYNOMIAL  0x8408

CCITTCrc16::CCITTCrc16()
{
    for (int i = 0; i < 256; i++) {
        tab[i] = initial((char) i);
    }
}

int CCITTCrc16::calculate(char * mem, int len)
{
  int crc = PRESET;
  for (int i = 0; i < len; i++) {
      crc = update_crc(crc, mem[i]);
  }
  return swab(crc);
}

int CCITTCrc16::initial(char c)
{
  int crc = 0;
  for (int j = 0; j < 8; j++) {
      if (((crc ^ c) & 1) == 1) {
          crc = ((crc >> 1) ^ POLYNOMIAL);
      } else {
          crc = (crc >> 1);
      }
      c = (char) (c >> 1);
  }
  return crc;
}

int CCITTCrc16::update_crc(int crc, char c)
{
  int cc = (0xff & c);

  int tmp = (crc ^ cc);
  crc = (crc >> 8) ^ tab[tmp & 0xff];

  return crc;
}

int CCITTCrc16::swab(int n)
{
  return (((n & 0xFF00) >> 8) + ((n & 0xFF) << 8));
}


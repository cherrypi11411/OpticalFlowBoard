#ifndef _CCITTCRC16_HEADER
#define _CCITTCRC16_HEADER

class CCITTCrc16
{
public:
  CCITTCrc16();
  int calculate(char * mem, int len);

private:
  int initial(char c);
  int update_crc(int crc, char c);
  int swab(int n);

  int tab[256];
};

#endif // _CCITTCRC16_HEADER



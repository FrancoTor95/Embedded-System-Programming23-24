#include <stdlib.h>
#include <stdio.h>


/* The main thing that this program does */
int main(void) {
  // Declaration
  double A[5] = {
    [0] = 9,
    [1] = 2.9,
    [4] = 3.E+25,
    [3] = .0007,
  };
  // Doing some work
  for (size_t i = 0; i < 5; i++) {
    printf("Element %zu is %g,\t\t its square is %g.\n",
          i,
          A[i],
          A[i]*A[i]);
  };

  return EXIT_SUCCESS;
}








  void main2()
    {
    float a = -1;
    unsigned int exp,sign,mantisa;
    int i;

        for(i = 0;i<4;i++)
        {
            exp      = (*((unsigned int *)&a) >>23) & 0xFF;
            sign     = (*((unsigned int *)&a) >>31) & 0x01;
            mantisa  = (*((unsigned int *)&a)) & 0x7FFFFF | 0x800000;

            printf("a       = %04x\r\n",*((unsigned int *)&a));
            printf("a       = %f\r\n",a);
            printf("exp     = %i, %02x\r\n",exp,exp);
            printf("sign    = %i, %02x\r\n",sign,sign);
            printf("mantisa = %i, %02x\r\n\r\n",mantisa,mantisa);
            a = -a / 2;
      }
    }
#include "mbed.h"

RawSerial pc(USBTX, USBRX);

// main() runs in its own thread in the OS
int main()
{
    char a;
    pc.baud(9600);
    pc.printf("Hello WOrld!\n");
    while(1)
    {
        wait(0.2);
        if(pc.readable())
        {
            a = pc.getc();
            pc.putc(a);
            pc.putc('\n');
           
        }
    }
}


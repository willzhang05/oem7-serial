#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define BAUDRATE B9600
#define DEVICE "/dev/ttyUSB0"

int main() {
    int fd = open(DEVICE, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        fprintf(stderr, "Could not open /dev/ttyUSB0\n");
        return 1;
    } else {
        printf("Opened /dev/ttyUSB0 successfully.");
    }
    struct termios settings;
    memset(&settings, 0, sizeof(settings));
    tcgetattr(fd, &settings);
    cfsetispeed(&settings, BAUDRATE);
    cfsetospeed(&settings,BAUDRATE);
    /*
    settings.c_cflag &= ~PARENB;
    settings.c_cflag &= ~CSTOPB;
    settings.c_cflag &= ~CSIZE;
    settings.c_cflag |= CS8;
    settings.c_cflag &= ~CRTSCTS;
    settings.c_cflag |= CREAD | CLOCAL;
    settings.c_iflag &= ~(IXON | IXOFF | IXANY);
    settings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    settings.c_oflag &= ~OPOST;
    */
    settings.c_iflag = 0;
    settings.c_oflag = 0;
    settings.c_cflag=CS8|CREAD|CLOCAL;
    settings.c_lflag=0;
    settings.c_cc[VMIN] = 1; /* Read at least 1 char */
    settings.c_cc[VTIME] = 0; /* Wait indefinitely */
    
    if((tcsetattr(fd, TCSANOW, &settings)) != 0) {
        fprintf(stderr, "\n\tERROR in setting attributes\n");
    } else {
        printf("\n\tSet to 9600 8N1\n");
    }
    tcflush(fd, TCIFLUSH);
    char r_buff[32];
    memset(&r_buff, 0, sizeof(r_buff));
    while (1) {
        int r_bytes = 0;
        r_bytes = read(fd, &r_buff, 32);
        for(int i=0;i<r_bytes;i++) {
            printf("%c", r_buff[i]);
        }
    }
    free(&settings);
    free(&r_buff);
    return 0;
}

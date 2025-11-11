#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

// ------------------- I2C & CAPTEUR --------------------

int write8(int fd, uint16_t reg, uint8_t val) {
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, val };
    if (write(fd, buf, 3) != 3) return -1;
    return 0;
}

uint8_t read8(int fd, uint16_t reg) {
    uint8_t buf[2] = { reg >> 8, reg & 0xFF };
    write(fd, buf, 2);
    uint8_t val;
    read(fd, &val, 1);
    return val;
}

void apply_tuning(int fd) {
    write8(fd, 0x0207, 0x01);
    write8(fd, 0x0208, 0x01);
    write8(fd, 0x0096, 0x00);
    write8(fd, 0x0097, 0xfd);
    write8(fd, 0x00e3, 0x00);
    write8(fd, 0x00e4, 0x04);
    write8(fd, 0x00e5, 0x02);
    write8(fd, 0x00e6, 0x01);
    write8(fd, 0x00e7, 0x03);
    write8(fd, 0x00f5, 0x02);
    write8(fd, 0x00d9, 0x05);
    write8(fd, 0x00db, 0xce);
    write8(fd, 0x00dc, 0x03);
    write8(fd, 0x00dd, 0xf8);
    write8(fd, 0x009f, 0x00);
    write8(fd, 0x00a3, 0x3c);
    write8(fd, 0x00b7, 0x00);
    write8(fd, 0x00bb, 0x3c);
    write8(fd, 0x00b2, 0x09);
    write8(fd, 0x00ca, 0x09);
    write8(fd, 0x0198, 0x01);
    write8(fd, 0x01b0, 0x17);
    write8(fd, 0x01ad, 0x00);
    write8(fd, 0x00ff, 0x05);
    write8(fd, 0x0100, 0x05);
    write8(fd, 0x0199, 0x05);
    write8(fd, 0x01a6, 0x1b);
    write8(fd, 0x01ac, 0x3e);
    write8(fd, 0x01a7, 0x1f);
    write8(fd, 0x0030, 0x00);
    write8(fd, 0x0011, 0x10); // Enables polling for ‘New Sample Ready’
    write8(fd, 0x010a, 0x30); // Set averaging sample period
    write8(fd, 0x003f, 0x46); // Set light/dark gain
    write8(fd, 0x0031, 0xff); // Auto calibration period
    write8(fd, 0x0040, 0x63); // ALS integration time
    write8(fd, 0x002e, 0x01); // Temperature calibration
    write8(fd, 0x001b, 0x09); // Default ranging inter-measurement period
    write8(fd, 0x003e, 0x31); // Default ALS inter-measurement period
    write8(fd, 0x0014, 0x24); // Interrupt config
    write8(fd, 0x0016, 0x00); // Fresh out of set to 0
}

// Fonction qui lit une distance unique
int lire_distance_unique(int fd) {
    write8(fd, 0x018, 0x01); // SYSRANGE__START

    // Attendre mesure prête
    uint8_t status = 0;
    int timeout = 0;
    do {
        status = read8(fd, 0x04f);
        usleep(1000);
        timeout++;
    } while (((status & 0x04) == 0) && timeout < 100);

    uint8_t range = read8(fd, 0x062);
    write8(fd, 0x015, 0x07); // clear interrupt
    return range;
}

// ------------------- PIPE & PROCESSUS --------------------

int main() {

struct termios SerialPortSettings;
tcgetattr(STDIN_FILENO, &SerialPortSettings);
SerialPortSettings.c_lflag &= ~ICANON;
tcsetattr(STDIN_FILENO, TCSANOW, &SerialPortSettings);
	SerialPortSettings.c_cc[VMIN] = 0; 
	SerialPortSettings.c_cc[VTIME] = 30; 

    int pere_fils[2];
    int fils_pf[2];
    int pf_fils[2];

    pipe(pere_fils); // père -> fils
    pipe(fils_pf);   // fils -> petit-fils
    pipe(pf_fils);   // petit-fils -> fils

    fcntl(pere_fils[0], F_SETFL, O_NONBLOCK);
    fcntl(fils_pf[0], F_SETFL, O_NONBLOCK);
    fcntl(pf_fils[0], F_SETFL, O_NONBLOCK);

    pid_t pidFils = fork();

    if (pidFils == 0) {
        pid_t pidPetitFils = fork();

        if (pidPetitFils == 0) {
            // === PETIT-FILS ===
            close(fils_pf[1]);
            close(pf_fils[0]);

            // Initialisation du capteur une seule fois
            int fd = open("/dev/i2c-1", O_RDWR);
            if (fd < 0) {
                perror("open i2c");
                exit(1);
            }
            ioctl(fd, I2C_SLAVE, 0x29);
            apply_tuning(fd);

            int lecture_active = 0;
            char commande;

            while (1) {
                int n = read(fils_pf[0], &commande, 1);
                if (n > 0) {
                    if (commande == 'S') {
                        lecture_active = 1;
                    } else if (commande == 'Q') {
                        close(fd);
                        exit(0);
                    }
                }

                if (lecture_active) {
                    int d = lire_distance_unique(fd);
                    char msg[50];
                    sprintf(msg, "Distance = %d mm\n", d);
                    write(pf_fils[1], msg, strlen(msg) + 1);
                    usleep(300000); // lecture toutes les 300 ms
                } else {
                    usleep(100000);
                }
            }

        } else {
            // === FILS ===
            close(pere_fils[1]);
            close(fils_pf[0]);
            close(pf_fils[1]);

            char commande;
            char message[100];
            while (1) {
                int n = read(pere_fils[0], &commande, 1);
                if (n > 0) {
                    write(fils_pf[1], &commande, 1);
                    if (commande == 'Q') exit(0);
                }

                int r = read(pf_fils[0], message, sizeof(message));
                if (r > 0) {
                    printf("[Fils] %s", message);
                    fflush(stdout);
                }
                usleep(100000);
            }
        }

    } else {
        // === PÈRE ===
        close(pere_fils[0]);

        printf("Commandes:\n");
        printf("  S = démarrer la lecture continue\n");
        printf("  Q = quitter\n");

        char c;
        while (1) {
            c = getchar();
            write(pere_fils[1], &c, 1);
            if (c == 'Q') break;
        }

        close(pere_fils[1]);
        wait(NULL);
    }

    return 0;
}

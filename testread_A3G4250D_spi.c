#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <math.h>

/* ── Registres ── */
#define WHO_AM_I        0x0F
#define CTRL_REG1       0x20
#define CTRL_REG4       0x23
#define CTRL_REG5       0x24
#define FIFO_CTRL_REG   0x2E
#define FIFO_SRC_REG    0x2F
#define OUT_X_L         0x28

#define READ_FLAG       0x80
#define MULTI_FLAG      0x40
#define SENSITIVITY     0.00875f
#define SPI_DEV         "/dev/spidev0.0"
#define PI 3.14159265358979323846   //Est qq part dans math.h mais on le redéfinit pour être sûr de sa précision

#define CALCULANGLE    1    //Calcul angulaire à partir de la vitesse mesurée et de la frequence mesurée (dt)

/* ── SPI ── */
static int fd;

static void spi_open(void) {
    uint8_t mode = SPI_MODE_3, bits = 8;
    uint32_t speed = 1000000;

    fd = open(SPI_DEV, O_RDWR);
    if (fd < 0) { perror("open"); exit(1); }

    ioctl(fd, SPI_IOC_WR_MODE,          &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed);
}

static void spi_xfer(uint8_t *tx, uint8_t *rx, int len) {
    struct spi_ioc_transfer tr = {
        .tx_buf        = (__u64)(uintptr_t)tx,
        .rx_buf        = (__u64)(uintptr_t)rx,
        .len           = len,
        .speed_hz      = 1000000,
        .delay_usecs   = 0,
        .bits_per_word = 8,
    };
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 0) { perror("ioctl"); exit(1); }
}

static uint8_t reg_read(uint8_t reg) {
    uint8_t tx[2] = { reg | READ_FLAG, 0 }, rx[2];
    spi_xfer(tx, rx, 2);
    return rx[1];
}

static void reg_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { reg, val }, rx[2];
    spi_xfer(tx, rx, 2);
}

static void burst_read(uint8_t reg, uint8_t *buf, int n) {
    uint8_t tx[n + 1], rx[n + 1];
    memset(tx, 0, sizeof(tx));
    tx[0] = reg | READ_FLAG | MULTI_FLAG;
    spi_xfer(tx, rx, n + 1);
    memcpy(buf, rx + 1, n);
}

/* ── Init ── */
static void gyro_init(void) {
    uint8_t id = reg_read(WHO_AM_I);
    printf("WHO_AM_I = 0x%02X %s\n", id, id == 0xD3 ? "OK" : "ERREUR");
    if (id != 0xD3) exit(1);

    reg_write(CTRL_REG1,     0x0F); /* 100Hz, tous axes ON       */
    reg_write(CTRL_REG4,     0x80); /* BDU=1, ±245 dps           */
    reg_write(CTRL_REG5,     0x00); /* FIFO désactivé au départ  */

    /* Reset FIFO : bypass → stream */
    reg_write(CTRL_REG5,     0x40); /* FIFO enable               */
    reg_write(FIFO_CTRL_REG, 0x00); /* Bypass                    */
    usleep(10000);
    reg_write(FIFO_CTRL_REG, 0x40); /* Stream, WTM=0             */

    usleep(200000); /* 200ms = ~20 échantillons à 100Hz */
    printf("Init OK\n\n");
}

// Calcule l'angle (en radians) à partir de la fréquence et du temps
/*
    Vitesse angulaire : ω = 2πf
    Angle en fonction du temps : θ = ω × t
    Donc : θ = 2πf × t
*/
double calcul_angle(double frequence, double temps) {
    double omega = 2.0 * PI * frequence;  // vitesse angulaire (rad/s)
    double angle = omega * temps;          // angle (radians)
    return angle;
}

// Conversion radians -> degrés
/*
    ω(rad/s)​=ω(°/s)​×180π​
*/
double rad_to_deg(double radians) {
    return radians * (180.0 / PI);
}

double deg_s_to_rad_s(double deg_par_seconde) {
    return deg_par_seconde * (PI / 180.0);
}

/* ── Main ── */
int main(void) {
    spi_open(); //Ouverture du bus SPI
    gyro_init(); //Initialisation du A3G4250D (MKI125V1)

    float angle_x = 0.0f, angle_y = 0.0f, angle_z = 0.0f;
    struct timespec t_prev, t_now;

    //Recuperation heure de départ pour calcul dt
    clock_gettime(CLOCK_MONOTONIC, &t_prev); 

    float cumulx = 0;

    while (1) {
        usleep(100000); // Attendre 100ms = 10Hz pour ne pas saturer le terminal

        //Recuperation de l'etat du FIFO
        uint8_t src = reg_read(FIFO_SRC_REG);
        int fss = src & 0x1F; //00011111 Permet de connaitre le niveau du FIFO (nombre d'échantillons disponibles)

        if (src & 0x20) continue; //Overrun : les données n'ont pas été lues assez vite, on perd des échantillons, on attend le prochain cycle pour lire les données suivantes
        if (fss == 0)   continue; //Pas d'échantillon disponible, on attend le prochain cycle pour lire les données suivantes

        /* ── Calcul dt réel ── */
        clock_gettime(CLOCK_MONOTONIC, &t_now);
        float dt = (t_now.tv_sec  - t_prev.tv_sec)
                 + (t_now.tv_nsec - t_prev.tv_nsec) * 1e-9f; 
        t_prev = t_now;
 
        for (int i = 0; i < fss; i++) {
            uint8_t raw[6];
            burst_read(OUT_X_L, raw, 6);

            float gx = (int16_t)((raw[1] << 8) | raw[0]) * SENSITIVITY; 
            float gy = (int16_t)((raw[3] << 8) | raw[2]) * SENSITIVITY;
            float gz = (int16_t)((raw[5] << 8) | raw[4]) * SENSITIVITY;

            /* ── Seuil de zéro : ignorer le bruit sous 0.1 °/s ── */
            if (gx < 0.1f && gx > -0.1f) gx = 0.0f;
            if (gy < 0.1f && gy > -0.1f) gy = 0.0f;
            if (gz < 0.1f && gz > -0.1f) gz = 0.0f;

            if(CALCULANGLE){
                float dtech = dt / fss; /* dt par échantillon */
                angle_x = gx*dtech;
                angle_y = gy*dtech;
                angle_z = gz*dtech;

                cumulx += angle_x; //Cumul de l'angle en X

                printf("Angles →  X: %2.2f° | %2.2f°  Y: %2.2f°  Z: %2.2f° dt: %3.2fHz Nb echantillons: %2d\r", angle_x, cumulx, angle_y, angle_z, 1/dt, fss);
            }
            else{
                printf("Angles →  X: %7.2f°/s  Y: %7.2f°/s  Z: %7.2f°/s dt: %7.4fHz Nb echantillons: %2d\r", 
               gx, gy, gz, 1/dt, fss);
            }
        }

        fflush(stdout);
    }

    close(fd);
    return 0;
}
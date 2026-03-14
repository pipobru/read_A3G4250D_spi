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
#include <stdbool.h>

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
#define LOGFILE        1    //Log dans un fichier pour analyse postérieure (ex : Excel)

/*
Fonction de filtrage de claude
gyro_raw ──► [ - biais ] ──► gyro_corrected
                  ▲                │
                  │         < seuil de repos ?
             BIAS_ALPHA              │
                  │            oui ──► fige l'angle
                  └── correction       + affine le biais
                      progressive
                                   non ──► intègre l'angle
*/
/* ─── Paramètres à ajuster selon votre capteur ─────────────────────────── */

#define GYRO_NOISE_THRESHOLD   0.05f   // rad/s — seuil de détection du repos
#define BIAS_ALPHA             0.002f  // vitesse d'adaptation du biais (0..1)
#define BIAS_INIT_SAMPLES      200     // échantillons pour la calibration initiale

/* ─── Structure d'état ──────────────────────────────────────────────────── */

typedef struct {
    float angle;          // angle intégré (rad)
    float bias;           // biais estimé du gyro (rad/s)
    float bias_acc;       // accumulateur pour calibration initiale
    uint16_t init_count;  // compteur de calibration
    bool  calibrated;     // calibration initiale terminée ?
} GyroFilter;

/* ─── Initialisation ────────────────────────────────────────────────────── */

void gyro_filter_init(GyroFilter *f)
{
    f->angle       = 0.0f;
    f->bias        = 0.0f;
    f->bias_acc    = 0.0f;
    f->init_count  = 0;
    f->calibrated  = false;
}

/* ─── Mise à jour — appeler à chaque échantillon ────────────────────────── */
/*                                                                            */
/*  gyro_raw : lecture brute du capteur (rad/s)                              */
/*  dt       : pas de temps depuis le dernier appel (secondes)               */
/*  Retourne : angle estimé (rad)                                            */

float gyro_filter_update(GyroFilter *f, float gyro_raw, float dt)
{
    /* ── Phase 1 : calibration initiale (capteur immobile au démarrage) ── */
    if (!f->calibrated) {
        f->bias_acc   += gyro_raw;
        f->init_count += 1;

        if (f->init_count >= BIAS_INIT_SAMPLES) {
            f->bias      = f->bias_acc / (float)BIAS_INIT_SAMPLES;
            f->calibrated = true;
        }
        return f->angle;   // on ne bouge pas encore
    }

    /* ── Phase 2 : correction du biais en temps réel ─────────────────── */
    /*                                                                      */
    /*  Si le capteur est quasi-immobile, la lecture brute ≈ biais pur.    */
    /*  On affine l'estimation par un filtre passe-bas exponentiel.        */

    float gyro_corrected = gyro_raw - f->bias;

    if (fabsf(gyro_corrected) < GYRO_NOISE_THRESHOLD) {
        /* Repos détecté : recalage du biais */
        f->bias += BIAS_ALPHA * gyro_corrected;
        gyro_corrected = 0.0f;   // on fige l'angle pendant le repos
    }

    /* ── Phase 3 : intégration ──────────────────────────────────────── */
    f->angle += gyro_corrected * dt;

    return f->angle;
}

/* ─── Remise à zéro de l'angle (sans toucher au biais) ─────────────────── */

void gyro_filter_reset_angle(GyroFilter *f)
{
    f->angle = 0.0f;
}
/*
Fin des fonctions de filtrage de claude
*/


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

    float cumulbrutx = 0, cumulbruty = 0, cumulbrutz = 0;
    float cumulx = 0, cumuly = 0, cumulz = 0;

    // Filtre pour limiter la dérive : estimation du biais gyro
    float bias_x = 0.0f, bias_y = 0.0f, bias_z = 0.0f;
    const float alpha = 0.001f; // Facteur d'adaptation du biais (petit pour convergence lente)
    /*
    Initialisation du filtrage de claude
    */
    GyroFilter filter;
    gyro_filter_init(&filter);

    if(LOGFILE){
        FILE *f = fopen("gyro_log.csv", "w");
        if (f) {
//            fprintf(f, "Time;RawX;RawY;RawZ;AngleX_Copilot;AngleY_Copilot;AngleZ_Copilot;AngleX_Claude;AngleY_Claude;AngleZ_Claude\n");
            fprintf(f, "Time;RawX;AngleX_Copilot;AngleX_Claude;RawY;AngleY_Copilot;AngleY_Claude;RawZ;AngleZ_Copilot;AngleZ_Claude\n");
            fclose(f);
        }
    }
 
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

        float cumuldt = 0.0f; //Cumul du temps pour les échantillons traités dans ce cycle
 
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

            //Filtre de claude
            float angle_x_claude = gyro_filter_update(&filter, deg_s_to_rad_s(gx), dt/fss);
            angle_x_claude = rad_to_deg(angle_x_claude);
            float angle_y_claude = gyro_filter_update(&filter, deg_s_to_rad_s(gy), dt/fss);
            angle_y_claude = rad_to_deg(angle_y_claude);
            float angle_z_claude = gyro_filter_update(&filter, deg_s_to_rad_s(gz), dt/fss);
            angle_z_claude = rad_to_deg(angle_z_claude);

            // Filtre pour limiter la dérive : estimation et soustraction du biais (copilot)
            bias_x = alpha * gx + (1.0f - alpha) * bias_x;
            bias_y = alpha * gy + (1.0f - alpha) * bias_y;
            bias_z = alpha * gz + (1.0f - alpha) * bias_z;

            float gx_corr = gx - bias_x;
            float gy_corr = gy - bias_y;
            float gz_corr = gz - bias_z;

            if(CALCULANGLE){
                float dtech = dt / fss; /* dt par échantillon */
                angle_x = gx_corr*dtech;
                angle_y = gy_corr*dtech;
                angle_z = gz_corr*dtech;

                cumulx += angle_x; //Cumul de l'angle en X
                cumuly += angle_y; //Cumul de l'angle en X
                cumulz += angle_z; //Cumul de l'angle en X

                //Attention ici on va cumuler les erreurs de mesure, il faudrait faire un filtrage pour limiter la dérive (ex : filtre de Kalman, complément
                cumulbrutx += gx*dtech;
                cumulbruty += gy*dtech;
                cumulbrutz += gz*dtech;

                printf("Angles (aucun filtre) →                    X: %7.2f°/s  Y: %7.2f°/s  Z: %7.2f°/s dt: %7.4fHz Nb echantillons: %2d\n", 
               cumulbrutx, cumulbruty, cumulbrutz, 1/dt, fss);
                printf("Angles (filtre Copilot integre à l'IDE) →  X: %7.2f°/s  Y: %7.2f°/s  Z: %7.2f°/s dt: %7.4fHz Nb echantillons: %2d\n", 
               cumulx, cumuly, cumulz, 1/dt, fss);
                printf("Angles (filtre claude) →                   X: %7.2f°/s  Y: %7.2f°/s  Z: %7.2f°/s dt: %7.4fHz Nb echantillons: %2d\n", 
               angle_x_claude, angle_y_claude, angle_z_claude, 1/dt, fss);
               printf("\033[3A");

               if(LOGFILE){
                    FILE *f = fopen("gyro_log.csv", "a");
                    if (f) {
                        cumuldt += dt;
                        /*
                        fprintf(f, "%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n", 
                            cumuldt, gx, gy, gz, 
                            cumulx, cumuly, cumulz, 
                            angle_x_claude, angle_y_claude, angle_z_claude);*/
                        fprintf(f, "%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n", 
                            cumuldt, gx, cumulx, angle_x_claude,
                            gy, cumuly, angle_y_claude,
                            gz, cumulz, angle_z_claude);
                        fclose(f);
                    }
                }

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

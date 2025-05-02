#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>

/* ----------------------------------------------------------
 *  Cross‑platform sleep + non‑blocking keyboard utilities
 * --------------------------------------------------------*/
#ifdef _WIN32
#   include <conio.h>
#   include <windows.h>
#   define SLEEP_MS(ms) Sleep(ms)
#   define KBHIT()      _kbhit()
#   define GETCH()      _getch()
#else
#   include <unistd.h>
#   include <termios.h>
#   include <fcntl.h>
#   include <sys/select.h>
#   define SLEEP_MS(ms) usleep((ms)*1000)
static int kbhit(void) {
    struct timeval tv = {0};
    fd_set fds; FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}
static int getch_nonblock(void) {
    struct termios oldt, newt; int ch = -1;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt; newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
#   define KBHIT()      kbhit()
#   define GETCH()      getch_nonblock()
#endif

#include "kuka_rsi.h"

/* ----------------------------------------------------------
 *  Globals & synchronisation
 * --------------------------------------------------------*/
static volatile bool g_exit            = false;
static volatile bool g_wiggle_enabled  = false;       /* <‑‑ space‑bar toggles this */
static pthread_mutex_t corr_mutex      = PTHREAD_MUTEX_INITIALIZER;
static RSI_CartesianCorrection shared_correction;

static double start_x      = 0.0;
static bool   start_pos_set = false;

/* ----------------------------------------------------------
 *  Real‑time callback: always returns IPOC + current correction
 * --------------------------------------------------------*/
static void on_data_callback(const RSI_CartesianPosition* cart,
                             const RSI_JointPosition*    joint,
                             void*                       user_data)
{
    (void)cart; (void)joint; (void)user_data;

    RSI_CartesianCorrection corr;
    pthread_mutex_lock(&corr_mutex);
    corr = shared_correction;                    /* take any pending deltas          */
    memset(&shared_correction, 0, sizeof(shared_correction));
    pthread_mutex_unlock(&corr_mutex);

    RSI_SetCartesianCorrection(&corr);           /* reply within IPOC window         */
}

/* ----------------------------------------------------------*/
static void on_signal(int sig) { (void)sig; g_exit = true; }

/* ----------------------------------------------------------
 *  Motion thread – generates ±0.1 mm steps while enabled
 * --------------------------------------------------------*/
static void* motion_thread_fn(void* arg)
{
    const double   STEP  = 0.1;     /* mm per command  */
    const double   LIMIT = 4.0;     /* ±4 mm envelope  */
    const uint32_t INTERVAL_MS = 32; /* send every ~32 ms */
    bool increasing = true;

    while (!g_exit) {
        if (!g_wiggle_enabled) {        /* paused – just idle briefly            */
            SLEEP_MS(INTERVAL_MS);
            continue;
        }

        RSI_CartesianPosition pos;
        if (RSI_GetCartesianPosition(&pos) != RSI_SUCCESS) {
            SLEEP_MS(INTERVAL_MS);
            continue;                   /* keep trying until data arrives        */
        }

        if (!start_pos_set) {           /* remember initial X once               */
            start_x        = pos.x;
            start_pos_set  = true;
        }

        double rel_x = pos.x - start_x;
        RSI_CartesianCorrection delta = {0};

        if (increasing) {
            if (rel_x <  LIMIT - 0.05) delta.x =  STEP;
            else                       increasing = false;
        } else {
            if (rel_x > -LIMIT + 0.05) delta.x = -STEP;
            else                       increasing = true;
        }

        if (delta.x != 0.0) {
            pthread_mutex_lock(&corr_mutex);
            shared_correction.x = delta.x;
            pthread_mutex_unlock(&corr_mutex);
        }

        SLEEP_MS(INTERVAL_MS);
    }
    return NULL;
}

/* ----------------------------------------------------------*/
int main(void)
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    RSI_Config cfg = {
        .local_ip   = "0.0.0.0",
        .local_port = 59152,
        .timeout_ms = 1000,
        .verbose    = true
    };

    puts("=== KUKA RSI wiggle utility ===");
    printf("Initializing RSI with configuration:\n");
    printf("  Local IP: %s\n", cfg.local_ip);
    printf("  Local Port: %d\n", cfg.local_port);
    printf("  Timeout: %d ms\n", cfg.timeout_ms);
    printf("  Verbose mode: %s\n", cfg.verbose ? "Enabled" : "Disabled");

    if (RSI_Init(&cfg) != RSI_SUCCESS) {
        fprintf(stderr, "RSI_Init failed\n");
        return 1;
    }

    if (RSI_SetCallbacks(on_data_callback, NULL, NULL) != RSI_SUCCESS) {
        fprintf(stderr, "RSI_SetCallbacks failed\n");
        RSI_Cleanup();
        return 1;
    }

    RSI_Error err = RSI_Start();
    if (err != RSI_SUCCESS) {
        fprintf(stderr, "RSI_Start failed with error code: %d\n", err);
        const char* error_msg = "Unknown error";
        switch(err) {
            case RSI_ERROR_SOCKET_FAILED:     error_msg = "Socket creation failed"; break;
            case RSI_ERROR_THREAD_FAILED:     error_msg = "Thread creation failed"; break;
            case RSI_ERROR_INVALID_PARAM:     error_msg = "Invalid parameter";      break;
            case RSI_ERROR_TIMEOUT:           error_msg = "Connection timeout";     break;
            case RSI_ERROR_ALREADY_RUNNING:   error_msg = "Already running";        break;
            case RSI_ERROR_NOT_RUNNING:       error_msg = "Not running";            break;
            case RSI_ERROR_UNKNOWN:           error_msg = "Unknown error";          break;
        }
        fprintf(stderr, "Error details: %s\n", error_msg);
        RSI_Cleanup();
        return 1;
    }

    pthread_t motion_thread;
    if (pthread_create(&motion_thread, NULL, motion_thread_fn, NULL) != 0) {
        fprintf(stderr, "motion thread creation failed\n");
        g_exit = true;
    }

    puts("Press <space> to start/stop wiggle, Esc or Ctrl‑C to quit.");
    while (!g_exit) {

        /* ---- status display ------------------------------------------------*/
        RSI_CartesianPosition pos;
        if (RSI_GetCartesianPosition(&pos) == RSI_SUCCESS) {
            printf("\rIPOC %6u | X: %7.3f mm | Wiggle: %s ",
                   pos.ipoc, pos.x,
                   g_wiggle_enabled ? "ON " : "OFF");
            fflush(stdout);
        }

        /* ---- handle keyboard ----------------------------------------------*/
        if (KBHIT()) {
            int ch = GETCH();
            switch (ch) {
                case ' ':   /* toggle wiggle */
                    g_wiggle_enabled = !g_wiggle_enabled;
                    printf("\n>>> Wiggle %s\n",
                           g_wiggle_enabled ? "ENABLED" : "DISABLED");

                    if (!g_wiggle_enabled) {  /* clear any queued correction   */
                        pthread_mutex_lock(&corr_mutex);
                        memset(&shared_correction, 0, sizeof(shared_correction));
                        pthread_mutex_unlock(&corr_mutex);
                    }
                    break;

                case 27:    /* Esc */
                    g_exit = true;
                    break;

                default:
                    break;
            }
        }

        SLEEP_MS(50);  /* reduce CPU load in the UI loop */
    }

    /* ----- cleanup ---------------------------------------------------------*/
    pthread_join(motion_thread, NULL);
    RSI_Stop();
    RSI_Cleanup();
    puts("\nShutdown complete.");
    return 0;
}

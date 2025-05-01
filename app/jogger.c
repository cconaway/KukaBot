#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>

#ifdef _WIN32
#   include <conio.h>
#   include <windows.h>
#   define KBHIT()      _kbhit()
#   define GETCH()      _getch()
#else
#   include <unistd.h>
#   include <termios.h>
#   include <fcntl.h>
#   include <sys/select.h>
static int kbhit(void) {
    struct timeval tv = {0};
    fd_set         fds;
    FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}
static int getch_nonblock(void) {
    struct termios oldt, newt;
    int            ch = -1;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
#   define KBHIT()      kbhit()
#   define GETCH()      getch_nonblock()
#   define SLEEP_MS(ms) usleep((ms)*1000)
#endif

#include "kuka_rsi.h"

static volatile bool g_exit = false;
static void on_signal(int sig) { (void)sig; g_exit = true; }

/**
 * Mandatory callback for RSI data.
 */
static void on_data_callback(const RSI_CartesianPosition* cart,
                             const RSI_JointPosition* joint,
                             void* user_data) {
    (void)cart;
    (void)joint;
    (void)user_data;
}

static void zero_correction(RSI_CartesianCorrection* c) {
    memset(c, 0, sizeof(*c));
}

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

    const double STEP = 0.1;
    RSI_CartesianCorrection corr;
    zero_correction(&corr);
    RSI_CartesianPosition pos = {0};

    puts("Keyboard jogger ready – press Esc or Ctrl-C to quit.");

    while (!g_exit) {
        if (RSI_GetCartesianPosition(&pos) == RSI_SUCCESS) {
            printf("\rIPOC %6u  XYZ %.1f %.1f %.1f mm   ",
                   pos.ipoc, pos.x, pos.y, pos.z);
            fflush(stdout);
        }

        if (KBHIT()) {
            int ch = GETCH();
            // Log key press to terminal
            printf("\nKey pressed: %d (ASCII: '%c')\n",
                    ch, (ch >= 32 && ch <= 126) ? (char)ch : ' ');

            switch (ch) {
                case 'w':
                    printf("Command: Move +Z (%.1f mm)\n", STEP);
                corr.z += STEP;
                break;
                case 's':
                    printf("Command: Move -Z (%.1f mm)\n", STEP);
                corr.z -= STEP;
                break;
                case 'a':
                    printf("Command: Move -X (%.1f mm)\n", STEP);
                corr.x -= STEP;
                break;
                case 'd':
                    printf("Command: Move +X (%.1f mm)\n", STEP);
                corr.x += STEP;
                break;
                case 'q':
                    printf("Command: Move -Y (%.1f mm)\n", STEP);
                corr.y -= STEP;
                break;
                case 'e':
                    printf("Command: Move +Y (%.1f mm)\n", STEP);
                corr.y += STEP;
                break;
                case ' ':
                    printf("Command: Zero correction\n");
                zero_correction(&corr);
                break;
                case 27:
                    printf("Command: Exit program\n");
                g_exit = true;
                break; // Esc
                default:
                    printf("Unhandled key: %d\n", ch);
                break;
            }
        }

        RSI_SetCartesianCorrection(&corr);
        zero_correction(&corr);
    }

    puts("\nStopping …");
    RSI_Stop();
    RSI_Cleanup();
    puts("Done.");
    return 0;
}

/**
 * @file kuka_rsi.c
 * @brief Implementation of the KUKA RSI communication library
 */

#include "../include/kuka_rsi.h"
#include "internal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #include <mmsystem.h>  // For timeBeginPeriod
    #include <process.h>   // For _beginthreadex
    
    typedef SOCKET socket_t;
    #define SOCKET_ERROR_CODE SOCKET_ERROR
    #define INVALID_SOCKET_VALUE INVALID_SOCKET
    #define GET_SOCKET_ERROR WSAGetLastError()
    #define CLOSE_SOCKET(s) closesocket(s)
    #define SOCKET_CLEANUP() WSACleanup()
#else
    #include <unistd.h>
    #include <sys/socket.h>
    #include <sys/mman.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <errno.h>
    #include <fcntl.h>
    #include <pthread.h>
    
    typedef int socket_t;
    #define SOCKET_ERROR_CODE -1
    #define INVALID_SOCKET_VALUE -1
    #define GET_SOCKET_ERROR errno
    #define CLOSE_SOCKET(s) close(s)
    #define SOCKET_CLEANUP()
#endif

/* Constants */
#define DEFAULT_LOCAL_IP "0.0.0.0"
#define DEFAULT_PORT 59152
#define DEFAULT_TIMEOUT_MS 1000
#define MAX_BUFFER_SIZE 4096
#define RESPONSE_BUFFER_SIZE 512

/* XML Tag Constants */
#define TAG_IPOC_START "<IPOC>"
#define TAG_IPOC_END "</IPOC>"
#define TAG_RIST_START "<RIst"
#define TAG_AIPOS_START "<AIPos"

/* Response template */
static const char *RESPONSE_TEMPLATE = 
    "<Sen Type=\"ImFree\">\n"
    "<EStr>RSI Monitor</EStr>\n"
    "<RKorr X=\"%.4f\" Y=\"%.4f\" Z=\"%.4f\" A=\"%.4f\" B=\"%.4f\" C=\"%.4f\" />\n"
    "<IPOC>%s</IPOC>\n"
    "</Sen>";

/* Global state */
typedef struct {
    bool initialized;
    bool running;
    
    /* Configuration */
    RSI_Config config;
    
    /* Socket */
    socket_t sock;
    struct sockaddr_in robot_addr;
    
    /* Buffers */
    char recv_buffer[MAX_BUFFER_SIZE] __attribute__((aligned(64)));
    char send_buffer[RESPONSE_BUFFER_SIZE] __attribute__((aligned(64)));
    
    /* Thread */
    #ifdef _WIN32
    HANDLE network_thread;
    CRITICAL_SECTION data_lock;
    #else
    pthread_t network_thread;
    pthread_mutex_t data_lock;
    #endif
    
    /* Callbacks */
    RSI_DataCallback data_callback;
    RSI_ConnectionCallback connection_callback;
    void* callback_user_data;
    
    /* Robot state */
    RSI_CartesianPosition cartesian;
    RSI_JointPosition joints;
    RSI_CartesianCorrection correction;
    
    /* Statistics */
    RSI_Statistics stats;
    
} RSI_Context;

/* Global context instance */
static RSI_Context g_context = {0};

/* Thread running flag */
static volatile bool g_exit_requested = false;

/**
 * Get high-precision timestamp in microseconds
 */
static uint64_t get_time_us(void) {
    #ifdef _WIN32
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (uint64_t)(count.QuadPart * 1000000ULL / freq.QuadPart);
    #else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
    #endif
}

/**
 * Extract IPOC value from XML data
 */
static bool extract_ipoc(const char* xml_data, char* ipoc_buffer, size_t buffer_size, uint32_t* ipoc_value) {
    const char* start_pos = strstr(xml_data, TAG_IPOC_START);
    if (!start_pos) return false;
    
    start_pos += strlen(TAG_IPOC_START);
    const char* end_pos = strstr(start_pos, TAG_IPOC_END);
    if (!end_pos) return false;
    
    size_t len = end_pos - start_pos;
    if (len >= buffer_size) len = buffer_size - 1;
    
    memcpy(ipoc_buffer, start_pos, len);
    ipoc_buffer[len] = '\0';
    
    if (ipoc_value) {
        *ipoc_value = (uint32_t)strtoul(ipoc_buffer, NULL, 10);
    }
    
    return true;
}

/**
 * Parse position value from attribute
 */
static double parse_position_attr(const char* xml, const char* attr_name) {
    char search_str[32];
    snprintf(search_str, sizeof(search_str), "%s=\"", attr_name);
    
    const char* attr_pos = strstr(xml, search_str);
    if (!attr_pos) return 0.0;
    
    attr_pos += strlen(search_str);
    return atof(attr_pos);
}

/**
 * Parse Cartesian position from XML data
 */
static bool parse_cartesian_position(const char* xml_data, RSI_CartesianPosition* position) {
    const char* rist_tag = strstr(xml_data, TAG_RIST_START);
    if (!rist_tag) return false;
    
    position->x = parse_position_attr(rist_tag, "X");
    position->y = parse_position_attr(rist_tag, "Y");
    position->z = parse_position_attr(rist_tag, "Z");
    position->a = parse_position_attr(rist_tag, "A");
    position->b = parse_position_attr(rist_tag, "B");
    position->c = parse_position_attr(rist_tag, "C");
    position->timestamp_us = get_time_us();
    
    return true;
}

/**
 * Parse Joint position from XML data
 */
static bool parse_joint_position(const char* xml_data, RSI_JointPosition* position) {
    const char* aipos_tag = strstr(xml_data, TAG_AIPOS_START);
    if (!aipos_tag) return false;
    
    position->axis[0] = parse_position_attr(aipos_tag, "A1");
    position->axis[1] = parse_position_attr(aipos_tag, "A2");
    position->axis[2] = parse_position_attr(aipos_tag, "A3");
    position->axis[3] = parse_position_attr(aipos_tag, "A4");
    position->axis[4] = parse_position_attr(aipos_tag, "A5");
    position->axis[5] = parse_position_attr(aipos_tag, "A6");
    position->timestamp_us = get_time_us();
    
    return true;
}

/**
 * Generate response XML with correction values
 */
static int generate_response(const char* ipoc, const RSI_CartesianCorrection* correction, char* buffer, size_t buffer_size) {
    int written = snprintf(buffer, buffer_size, RESPONSE_TEMPLATE,
                         correction->x, correction->y, correction->z,
                         correction->a, correction->b, correction->c,
                         ipoc);
    
    if (written < 0 || written >= (int)buffer_size) {
        return 0;
    }
    
    return written;
}

/**
 * Process a packet from the robot
 */
static void process_packet(const char* data, int data_len, struct sockaddr_in* robot_addr) {
    uint64_t start_time = get_time_us();
    char ipoc_buffer[32] = {0};
    uint32_t ipoc_value = 0;
    bool ipoc_extracted;
    bool cartesian_parsed;
    bool joints_parsed;
    int response_len;
    
    // Update connection status if needed
    if (!g_context.stats.is_connected) {
        g_context.stats.is_connected = true;
        if (g_context.connection_callback) {
            g_context.connection_callback(true, g_context.callback_user_data);
        }
    }
    
    // Extract IPOC
    ipoc_extracted = extract_ipoc(data, ipoc_buffer, sizeof(ipoc_buffer), &ipoc_value);
    if (!ipoc_extracted) {
        return;
    }
    
    // Lock data
    #ifdef _WIN32
    EnterCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_lock(&g_context.data_lock);
    #endif
    
    // Parse positions
    cartesian_parsed = parse_cartesian_position(data, &g_context.cartesian);
    joints_parsed = parse_joint_position(data, &g_context.joints);
    
    // Update IPOC values
    if (ipoc_extracted) {
        g_context.cartesian.ipoc = ipoc_value;
        g_context.joints.ipoc = ipoc_value;
    }
    
    // Generate response
    response_len = generate_response(ipoc_buffer, &g_context.correction, 
                                   g_context.send_buffer, RESPONSE_BUFFER_SIZE);
    
    // Make a local copy of the robot address
    struct sockaddr_in addr_copy = *robot_addr;
    
    #ifdef _WIN32
    LeaveCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_unlock(&g_context.data_lock);
    #endif
    
    // Call data callback if registered
    if (g_context.data_callback && cartesian_parsed && joints_parsed) {
        g_context.data_callback(&g_context.cartesian, &g_context.joints, 
                              g_context.callback_user_data);
    }
    
    // Send response
    if (response_len > 0) {
        sendto(g_context.sock, g_context.send_buffer, response_len, 0,
              (struct sockaddr*)&addr_copy, sizeof(addr_copy));
        g_context.stats.packets_sent++;
    }
    
    // Calculate processing time
    uint64_t end_time = get_time_us();
    uint64_t processing_time = end_time - start_time;
    double processing_time_ms = (double)processing_time / 1000.0;
    
    // Update statistics
    g_context.stats.packets_received++;
    g_context.stats.last_packet_timestamp_us = end_time;
    g_context.stats.avg_response_time_ms = 
        ((g_context.stats.avg_response_time_ms * (g_context.stats.packets_received - 1)) + 
         processing_time_ms) / g_context.stats.packets_received;
    
    if (processing_time_ms < g_context.stats.min_response_time_ms || 
        g_context.stats.min_response_time_ms == 0.0) {
        g_context.stats.min_response_time_ms = processing_time_ms;
    }
    
    if (processing_time_ms > g_context.stats.max_response_time_ms) {
        g_context.stats.max_response_time_ms = processing_time_ms;
    }
    
    if (processing_time_ms > 4.0) {
        g_context.stats.late_responses++;
        
        if (g_context.config.verbose) {
            printf("WARNING: Slow response: %.3f ms\n", processing_time_ms);
        }
    }
}

/**
 * Check for connection timeout
 */
static void check_connection_timeout(void) {
    if (g_context.config.timeout_ms == 0 || !g_context.stats.is_connected) {
        return;
    }
    
    uint64_t current_time = get_time_us();
    uint64_t time_since_last_packet = current_time - g_context.stats.last_packet_timestamp_us;
    
    if (time_since_last_packet > (uint64_t)g_context.config.timeout_ms * 1000) {
        // Connection timeout
        g_context.stats.is_connected = false;
        g_context.stats.connection_lost_count++;
        
        if (g_context.connection_callback) {
            g_context.connection_callback(false, g_context.callback_user_data);
        }
        
        if (g_context.config.verbose) {
            printf("RSI: Connection timeout after %u ms\n", g_context.config.timeout_ms);
        }
    }
}

/**
 * Network thread function
 */
#ifdef _WIN32
static unsigned __stdcall network_thread_func(void* param) {
#else
static void* network_thread_func(void* param) {
#endif
    struct sockaddr_in robot_addr;
    socklen_t addr_len = sizeof(robot_addr);
    int recv_len;
    
    // Set thread priority to time-critical
    #ifdef _WIN32
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
    #else
    struct sched_param schedParam;
    schedParam.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedParam);
    
    #endif
    
    if (g_context.config.verbose) {
        printf("RSI: Network thread started with high priority\n");
    }
    
    while (!g_exit_requested) {
        // Receive packet (non-blocking)
        recv_len = recvfrom(g_context.sock, g_context.recv_buffer, MAX_BUFFER_SIZE - 1, 0,
                         (struct sockaddr*)&robot_addr, &addr_len);
        
        if (recv_len > 0) {
            // Null-terminate received data
            g_context.recv_buffer[recv_len] = '\0';
            
            // Process and respond
            process_packet(g_context.recv_buffer, recv_len, &robot_addr);
        }
        
        // Check for connection timeout
        check_connection_timeout();
        
        // Yield thread briefly without sleeping
        #ifdef _WIN32
        SwitchToThread();
        #else
        sched_yield();
        #endif
    }
    
    if (g_context.config.verbose) {
        printf("RSI: Network thread exiting\n");
    }
    
    return 0;
}

/**
 * Create and configure socket for minimal latency
 */
static RSI_Error create_optimized_socket(const char* local_ip, uint16_t local_port) {
    struct sockaddr_in local_addr;
    int reuse = 1;
    
    // Create UDP socket
    g_context.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_context.sock == INVALID_SOCKET_VALUE) {
        if (g_context.config.verbose) {
            printf("RSI: Failed to create socket, error: %d\n", GET_SOCKET_ERROR);
        }
        return RSI_ERROR_SOCKET_FAILED;
    }
    
    // Allow address reuse
    if (setsockopt(g_context.sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0) {
        if (g_context.config.verbose) {
            printf("RSI: setsockopt(SO_REUSEADDR) failed, error: %d\n", GET_SOCKET_ERROR);
        }
    }
    
    // Set large socket buffers
    int rcvbuf_size = 1048576;  // 1MB
    int sndbuf_size = 1048576;  // 1MB
    
    if (setsockopt(g_context.sock, SOL_SOCKET, SO_RCVBUF, (const char*)&rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
        if (g_context.config.verbose) {
            printf("RSI: setsockopt(SO_RCVBUF) failed, error: %d\n", GET_SOCKET_ERROR);
        }
    }
    
    if (setsockopt(g_context.sock, SOL_SOCKET, SO_SNDBUF, (const char*)&sndbuf_size, sizeof(sndbuf_size)) < 0) {
        if (g_context.config.verbose) {
            printf("RSI: setsockopt(SO_SNDBUF) failed, error: %d\n", GET_SOCKET_ERROR);
        }
    }
    
    // Set up local address
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(local_port);
    
    if (strcmp(local_ip, "0.0.0.0") == 0) {
        local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        local_addr.sin_addr.s_addr = inet_addr(local_ip);
    }
    
    if (g_context.config.verbose) {
        printf("RSI: Binding to %s:%d\n", local_ip, local_port);
    }
    
    // Bind socket to local address
    if (bind(g_context.sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) == SOCKET_ERROR_CODE) {
        if (g_context.config.verbose) {
            printf("RSI: Bind failed, error: %d\n", GET_SOCKET_ERROR);
        }
        CLOSE_SOCKET(g_context.sock);
        return RSI_ERROR_SOCKET_FAILED;
    }
    
    // Set socket to non-blocking mode
    #ifdef _WIN32
    u_long mode = 1;
    if (ioctlsocket(g_context.sock, FIONBIO, &mode) == SOCKET_ERROR) {
        if (g_context.config.verbose) {
            printf("RSI: ioctlsocket failed with error: %d\n", GET_SOCKET_ERROR);
        }
        CLOSE_SOCKET(g_context.sock);
        return RSI_ERROR_SOCKET_FAILED;
    }
    #else
    int flags = fcntl(g_context.sock, F_GETFL, 0);
    if (flags < 0 || fcntl(g_context.sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        if (g_context.config.verbose) {
            printf("RSI: fcntl failed with error: %d\n", errno);
        }
        CLOSE_SOCKET(g_context.sock);
        return RSI_ERROR_SOCKET_FAILED;
    }
    #endif
    
    if (g_context.config.verbose) {
        printf("RSI: Socket configured for minimal latency\n");
    }
    
    return RSI_SUCCESS;
}

/**
 * Initialize system optimizations
 */
static void init_system_optimizations(void) {
    // Set Windows timer resolution to 1ms
    #ifdef _WIN32
    timeBeginPeriod(1);
    #endif
    
    // Initialize synchronization primitives
    #ifdef _WIN32
    InitializeCriticalSectionAndSpinCount(&g_context.data_lock, 4000);
    #else
    pthread_mutex_init(&g_context.data_lock, NULL);
    #endif
    
    // Set process priority to high
    #ifdef _WIN32
    SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS);
    #else
    /* Linux priority setup would go here */
    #endif
    
    if (g_context.config.verbose) {
        printf("RSI: System optimizations applied\n");
    }
}

/**
 * Clean up system optimizations
 */
static void cleanup_system_optimizations(void) {
    // Reset Windows timer resolution
    #ifdef _WIN32
    timeEndPeriod(1);
    #endif
    
    // Clean up synchronization primitives
    #ifdef _WIN32
    DeleteCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_destroy(&g_context.data_lock);
    #endif
    
    if (g_context.config.verbose) {
        printf("RSI: System optimizations cleaned up\n");
    }
}

/**
 * Initialize network subsystem
 */
static RSI_Error init_network(void) {
    #ifdef _WIN32
    WSADATA wsa_data;
    int result = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    if (result != 0) {
        if (g_context.config.verbose) {
            printf("RSI: WSAStartup failed with error: %d\n", result);
        }
        return RSI_ERROR_INIT_FAILED;
    }
    
    if (g_context.config.verbose) {
        printf("RSI: Windows Sockets initialized successfully\n");
    }
    #else
    if (g_context.config.verbose) {
        printf("RSI: POSIX Sockets ready\n");
    }
    #endif
    
    return RSI_SUCCESS;
}

/* Public API Implementation */

RSI_Error RSI_Init(const RSI_Config* config) {
    // Check if already initialized
    if (g_context.initialized) {
        return RSI_ERROR_ALREADY_RUNNING;
    }
    
    // Clear context
    memset(&g_context, 0, sizeof(g_context));
    
    // Initialize stats with default values
    g_context.stats.min_response_time_ms = 9999.0;
    
    // Set configuration (use defaults if NULL)
    if (config) {
        memcpy(&g_context.config, config, sizeof(RSI_Config));
    } else {
        g_context.config.local_ip = DEFAULT_LOCAL_IP;
        g_context.config.local_port = DEFAULT_PORT;
        g_context.config.timeout_ms = DEFAULT_TIMEOUT_MS;
        g_context.config.verbose = false;
    }
    
    // Apply system optimizations
    init_system_optimizations();
    
    // Initialize network
    RSI_Error err = init_network();
    if (err != RSI_SUCCESS) {
        cleanup_system_optimizations();
        return err;
    }
    
    g_context.initialized = true;
    return RSI_SUCCESS;
}

RSI_Error RSI_SetCallbacks(RSI_DataCallback data_callback,
                         RSI_ConnectionCallback connection_callback,
                         void* user_data) {
    if (!g_context.initialized) {
        return RSI_ERROR_INIT_FAILED;
    }
    
    if (g_context.running) {
        return RSI_ERROR_ALREADY_RUNNING;
    }
    
    g_context.data_callback = data_callback;
    g_context.connection_callback = connection_callback;
    g_context.callback_user_data = user_data;
    
    return RSI_SUCCESS;
}

RSI_Error RSI_Start(void) {
    RSI_Error err;
    
    // Check if initialized
    if (!g_context.initialized) {
        return RSI_ERROR_INIT_FAILED;
    }
    
    // Check if already running
    if (g_context.running) {
        return RSI_ERROR_ALREADY_RUNNING;
    }
    
    // Create and configure socket
    const char* local_ip = g_context.config.local_ip ? g_context.config.local_ip : DEFAULT_LOCAL_IP;
    uint16_t local_port = g_context.config.local_port ? g_context.config.local_port : DEFAULT_PORT;
    
    err = create_optimized_socket(local_ip, local_port);
    if (err != RSI_SUCCESS) {
        return err;
    }
    
    // Initialize exit flag
    g_exit_requested = false;
    
    // Start network thread
    #ifdef _WIN32
    g_context.network_thread = (HANDLE)_beginthreadex(NULL, 0, network_thread_func, NULL, 0, NULL);
    if (g_context.network_thread == NULL) {
        if (g_context.config.verbose) {
            printf("RSI: Failed to create network thread\n");
        }
        CLOSE_SOCKET(g_context.sock);
        return RSI_ERROR_THREAD_FAILED;
    }
    #else
    if (pthread_create(&g_context.network_thread, NULL, network_thread_func, NULL) != 0) {
        if (g_context.config.verbose) {
            printf("RSI: Failed to create network thread\n");
        }
        CLOSE_SOCKET(g_context.sock);
        return RSI_ERROR_THREAD_FAILED;
    }
    #endif
    
    g_context.running = true;
    
    if (g_context.config.verbose) {
        printf("RSI: Started successfully\n");
    }
    
    return RSI_SUCCESS;
}

RSI_Error RSI_Stop(void) {
    // Check if initialized and running
    if (!g_context.initialized) {
        return RSI_ERROR_INIT_FAILED;
    }
    
    if (!g_context.running) {
        return RSI_ERROR_NOT_RUNNING;
    }
    
    // Signal thread to exit
    g_exit_requested = true;
    
    // Wait for thread to exit
    #ifdef _WIN32
    WaitForSingleObject(g_context.network_thread, 1000);
    CloseHandle(g_context.network_thread);
    g_context.network_thread = NULL;
    #else
    pthread_join(g_context.network_thread, NULL);
    #endif
    
    // Close socket
    CLOSE_SOCKET(g_context.sock);
    
    g_context.running = false;
    
    if (g_context.config.verbose) {
        printf("RSI: Stopped successfully\n");
    }
    
    return RSI_SUCCESS;
}

RSI_Error RSI_Cleanup(void) {
    // Check if initialized
    if (!g_context.initialized) {
        return RSI_ERROR_INIT_FAILED;
    }
    
    // Stop if still running
    if (g_context.running) {
        RSI_Error err = RSI_Stop();
        if (err != RSI_SUCCESS) {
            return err;
        }
    }
    
    // Clean up network
    #ifdef _WIN32
    WSACleanup();
    #endif
    
    // Clean up system optimizations
    cleanup_system_optimizations();
    
    g_context.initialized = false;
    
    if (g_context.config.verbose) {
        printf("RSI: Cleaned up successfully\n");
    }
    
    return RSI_SUCCESS;
}

RSI_Error RSI_GetCartesianPosition(RSI_CartesianPosition* position) {
    // Check if initialized and running
    if (!g_context.initialized) {
        return RSI_ERROR_INIT_FAILED;
    }
    
    if (!g_context.running) {
        return RSI_ERROR_NOT_RUNNING;
    }
    
    if (!position) {
        return RSI_ERROR_INVALID_PARAM;
    }
    
    // Lock data
    #ifdef _WIN32
    EnterCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_lock(&g_context.data_lock);
    #endif
    
    // Copy data
    memcpy(position, &g_context.cartesian, sizeof(RSI_CartesianPosition));
    
    // Unlock data
    #ifdef _WIN32
    LeaveCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_unlock(&g_context.data_lock);
    #endif
    
    return RSI_SUCCESS;
}

RSI_Error RSI_GetJointPosition(RSI_JointPosition* position) {
    // Check if initialized and running
    if (!g_context.initialized) {
        return RSI_ERROR_INIT_FAILED;
    }
    
    if (!g_context.running) {
        return RSI_ERROR_NOT_RUNNING;
    }
    
    if (!position) {
        return RSI_ERROR_INVALID_PARAM;
    }
    
    // Lock data
    #ifdef _WIN32
    EnterCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_lock(&g_context.data_lock);
    #endif
    
    // Copy data
    memcpy(position, &g_context.joints, sizeof(RSI_JointPosition));
    
    // Unlock data
    #ifdef _WIN32
    LeaveCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_unlock(&g_context.data_lock);
    #endif
    
    return RSI_SUCCESS;
}

RSI_Error RSI_SetCartesianCorrection(const RSI_CartesianCorrection* correction) {
    // Check if initialized and running
    if (!g_context.initialized) {
        return RSI_ERROR_INIT_FAILED;
    }
    
    if (!g_context.running) {
        return RSI_ERROR_NOT_RUNNING;
    }
    
    if (!correction) {
        return RSI_ERROR_INVALID_PARAM;
    }
    
    // Lock data
    #ifdef _WIN32
    EnterCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_lock(&g_context.data_lock);
    #endif
    
    // Copy correction data
    memcpy(&g_context.correction, correction, sizeof(RSI_CartesianCorrection));
    
    // Unlock data
    #ifdef _WIN32
    LeaveCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_unlock(&g_context.data_lock);
    #endif
    
    return RSI_SUCCESS;
}

RSI_Error RSI_GetStatistics(RSI_Statistics* stats) {
    // Check if initialized
    if (!g_context.initialized) {
        return RSI_ERROR_INIT_FAILED;
    }
    
    if (!stats) {
        return RSI_ERROR_INVALID_PARAM;
    }
    
    // Lock data
    #ifdef _WIN32
    EnterCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_lock(&g_context.data_lock);
    #endif
    
    // Copy statistics
    memcpy(stats, &g_context.stats, sizeof(RSI_Statistics));
    
    // Unlock data
    #ifdef _WIN32
    LeaveCriticalSection(&g_context.data_lock);
    #else
    pthread_mutex_unlock(&g_context.data_lock);
    #endif
    
    return RSI_SUCCESS;
}

const char* RSI_GetErrorString(RSI_Error error) {
    switch (error) {
        case RSI_SUCCESS:
            return "Success";
        case RSI_ERROR_INIT_FAILED:
            return "Initialization failed";
        case RSI_ERROR_ALREADY_RUNNING:
            return "RSI is already running";
        case RSI_ERROR_NOT_RUNNING:
            return "RSI is not running";
        case RSI_ERROR_SOCKET_FAILED:
            return "Socket creation or binding failed";
        case RSI_ERROR_THREAD_FAILED:
            return "Thread creation failed";
        case RSI_ERROR_INVALID_PARAM:
            return "Invalid parameter provided";
        case RSI_ERROR_TIMEOUT:
            return "Operation timed out";
        case RSI_ERROR_UNKNOWN:
        default:
            return "Unknown error";
    }
}

/**
 * @file kuka_rsi.h
 * @brief KUKA Robot Sensor Interface (RSI) communication library
 * 
 * This library provides high-performance communication with KUKA robots
 * using the Robot Sensor Interface (RSI) protocol. It handles the
 * real-time requirements of RSI communication with sub-4ms response times.
 * 
 * Usage:
 * 1. Initialize the library with RSI_Init()
 * 2. Set callback functions with RSI_SetCallbacks()
 * 3. Start communication with RSI_Start()
 * 4. Use accessors to get robot state
 * 5. Use correction functions to send corrections
 * 6. Stop communication with RSI_Stop()
 * 7. Clean up with RSI_Cleanup()
 */


// Only allows one header file to be called.
#ifndef KUKA_RSI_H
#define KUKA_RSI_H

#include <stdbool.h>
#include <stdint.h>

//C++ support
#ifdef __cplusplus
extern "C" {
#endif


// @brief Error codes returned by RSI functions
typedef enum {
    RSI_SUCCESS = 0,           /**< Operation successful */
    RSI_ERROR_INIT_FAILED,     /**< Initialization failed */
    RSI_ERROR_ALREADY_RUNNING, /**< RSI is already running */
    RSI_ERROR_NOT_RUNNING,     /**< RSI is not running */
    RSI_ERROR_SOCKET_FAILED,   /**< Socket creation or binding failed */
    RSI_ERROR_THREAD_FAILED,   /**< Thread creation failed */
    RSI_ERROR_INVALID_PARAM,   /**< Invalid parameter provided */
    RSI_ERROR_TIMEOUT,         /**< Operation timed out */
    RSI_ERROR_UNKNOWN          /**< Unknown error */
} RSI_Error;


//@brief RSI connection configuration
typedef struct {
    const char* local_ip;      /**< Local IP address (0.0.0.0 for any) */
    uint16_t local_port;       /**< Local port to bind to (default: 59152) */
    uint32_t timeout_ms;       /**< Connection timeout in milliseconds (0 for no timeout) */
    bool verbose;              /**< Enable verbose logging */
} RSI_Config;

//Robot position in Cartesian coordinates
typedef struct {
    double x;              /**< X position in mm */
    double y;              /**< Y position in mm */
    double z;              /**< Z position in mm */
    double a;              /**< A rotation in degrees */
    double b;              /**< B rotation in degrees */
    double c;              /**< C rotation in degrees */
    uint64_t timestamp_us; /**< Timestamp in microseconds */
    uint32_t ipoc;         /**< IPOC value from robot */
} RSI_CartesianPosition;


//Robot position in joint coordinates
typedef struct {
    double axis[6];        /**< Joint angles in degrees (A1-A6) */
    uint64_t timestamp_us; /**< Timestamp in microseconds */
    uint32_t ipoc;         /**< IPOC value from robot */
} RSI_JointPosition;

//Correction data to send to robot
// How the Robot is moved.
typedef struct {
    /* Position correction in Cartesian coordinates */
    double x;              /**< X correction in mm */
    double y;              /**< Y correction in mm */
    double z;              /**< Z correction in mm */
    double a;              /**< A correction in degrees */
    double b;              /**< B correction in degrees */
    double c;              /**< C correction in degrees */
} RSI_CartesianCorrection;


//Statistics about RSI communication
typedef struct {
    uint64_t packets_received;           /**< Total packets received */
    uint64_t packets_sent;               /**< Total packets sent */
    double avg_response_time_ms;         /**< Average response time in ms */
    double min_response_time_ms;         /**< Minimum response time in ms */
    double max_response_time_ms;         /**< Maximum response time in ms */
    uint64_t late_responses;             /**< Number of responses over 4ms */
    uint64_t connection_lost_count;      /**< Number of connection losses */
    bool is_connected;                   /**< Current connection status */
    uint64_t last_packet_timestamp_us;   /**< Timestamp of last packet */
} RSI_Statistics;

/**
 * @brief Callback for robot data
 * 
 * This callback is called every time a new packet is received from the robot.
 * It runs in the high-priority networking thread, so it should be as fast as possible.
 * 
 * @param cartesian Cartesian position data
 * @param joints Joint position data
 * @param user_data User data pointer provided in RSI_SetCallbacks
 */
typedef void (*RSI_DataCallback)(const RSI_CartesianPosition* cartesian, 
                                const RSI_JointPosition* joints,
                                void* user_data);

/**
 * @brief Callback for connection status changes
 * 
 * This callback is called when the connection status changes.
 * 
 * @param connected True if connected, false if disconnected
 * @param user_data User data pointer provided in RSI_SetCallbacks
 */
typedef void (*RSI_ConnectionCallback)(bool connected, void* user_data);

/**
 * @brief Initialize the RSI library
 * 
 * This function initializes the RSI library and configures it for communication.
 * It must be called before any other RSI function.
 * 
 * @param config Configuration parameters (or NULL for defaults)
 * @return RSI_SUCCESS on success, error code otherwise
 */
RSI_Error RSI_Init(const RSI_Config* config);

/**
 * @brief Set callback functions
 * 
 * @param data_callback Callback for robot data (can be NULL)
 * @param connection_callback Callback for connection status changes (can be NULL)
 * @param user_data User data pointer passed to callbacks
 * @return RSI_SUCCESS on success, error code otherwise
 */
RSI_Error RSI_SetCallbacks(RSI_DataCallback data_callback,
                          RSI_ConnectionCallback connection_callback,
                          void* user_data);

/**
 * @brief Start RSI communication
 * 
 * This function starts the RSI communication thread and begins listening for
 * packets from the robot.
 * 
 * @return RSI_SUCCESS on success, error code otherwise
 */
RSI_Error RSI_Start(void);

/**
 * @brief Stop RSI communication
 * 
 * This function stops the RSI communication thread and closes the socket.
 * 
 * @return RSI_SUCCESS on success, error code otherwise
 */
RSI_Error RSI_Stop(void);

/**
 * @brief Clean up RSI library
 * 
 * This function cleans up resources used by the RSI library.
 * It should be called after RSI_Stop() and before the program exits.
 * 
 * @return RSI_SUCCESS on success, error code otherwise
 */
RSI_Error RSI_Cleanup(void);

/**
 * @brief Get the latest Cartesian position
 * 
 * @param position Pointer to structure to receive position data
 * @return RSI_SUCCESS on success, error code otherwise
 */
RSI_Error RSI_GetCartesianPosition(RSI_CartesianPosition* position);

/**
 * @brief Get the latest joint position
 * 
 * @param position Pointer to structure to receive position data
 * @return RSI_SUCCESS on success, error code otherwise
 */
RSI_Error RSI_GetJointPosition(RSI_JointPosition* position);

/**
 * @brief Send Cartesian correction to the robot
 * 
 * This function sets the correction values to be sent in the next response.
 * 
 * @param correction Correction values
 * @return RSI_SUCCESS on success, error code otherwise
 */
RSI_Error RSI_SetCartesianCorrection(const RSI_CartesianCorrection* correction);

/**
 * @brief Get statistics about RSI communication
 * 
 * @param stats Pointer to structure to receive statistics
 * @return RSI_SUCCESS on success, error code otherwise
 */
RSI_Error RSI_GetStatistics(RSI_Statistics* stats);

/**
 * @brief Get string representation of error code
 * 
 * @param error Error code
 * @return String representation of error code
 */
const char* RSI_GetErrorString(RSI_Error error);

#ifdef __cplusplus
}
#endif

#endif /* KUKA_RSI_H */

# KUKA RSI Communication Library API Documentation

## Overview

The KUKA RSI (Robot Sensor Interface) Communication Library provides a high-performance interface for communicating with KUKA robots using the RSI protocol. The library handles the low-level networking details and strict timing requirements (responses under 4ms) of the RSI protocol, allowing application developers to focus on robot control logic rather than communication implementation.

## Features

- High-performance networking with response times consistently below 4ms
- Thread-safe API with callback support
- Cartesian and joint position monitoring
- Position correction sending
- Connection status monitoring
- Detailed performance statistics

## Requirements

- **Windows**: Visual Studio 2015 or newer, MinGW, or other compatible C compiler
- **Linux**: GCC 4.8.5 or newer
- **Libraries**:
  - Windows: ws2_32.lib (Winsock), winmm.lib (Multimedia timers)
  - Linux: pthread

## Installation

1. Include the library files in your project:
   - `include/kuka_rsi.h` - Public API header
   - `src/kuka_rsi.c` - Implementation
   - `src/internal.h` - Internal declarations

2. Add library dependencies to your build system:
   ```cmake
   # For Windows
   target_link_libraries(your_app ws2_32 winmm)
   
   # For Linux
   target_link_libraries(your_app pthread)
   ```

## Basic Usage

```c
#include "kuka_rsi.h"

// Initialize with default configuration
RSI_Init(NULL);

// Register callbacks
RSI_SetCallbacks(my_data_callback, my_connection_callback, user_data);

// Start communication
RSI_Start();

// Application main loop
while (running) {
    // Get current position
    RSI_CartesianPosition pos;
    RSI_GetCartesianPosition(&pos);
    
    // Process position data...
    
    // Optionally send corrections
    RSI_CartesianCorrection corr = {0};
    corr.x = 1.0;  // Move 1mm in X direction
    RSI_SetCartesianCorrection(&corr);
    
    // Sleep or perform other tasks
    sleep(100);
}

// Clean up
RSI_Stop();
RSI_Cleanup();
```

## API Reference

### Data Types

#### RSI_Error

```c
typedef enum {
    RSI_SUCCESS = 0,           /* Operation successful */
    RSI_ERROR_INIT_FAILED,     /* Initialization failed */
    RSI_ERROR_ALREADY_RUNNING, /* RSI is already running */
    RSI_ERROR_NOT_RUNNING,     /* RSI is not running */
    RSI_ERROR_SOCKET_FAILED,   /* Socket creation or binding failed */
    RSI_ERROR_THREAD_FAILED,   /* Thread creation failed */
    RSI_ERROR_INVALID_PARAM,   /* Invalid parameter provided */
    RSI_ERROR_TIMEOUT,         /* Operation timed out */
    RSI_ERROR_UNKNOWN          /* Unknown error */
} RSI_Error;
```

Error codes returned by RSI functions.

#### RSI_Config

```c
typedef struct {
    const char* local_ip;      /* Local IP address (0.0.0.0 for any) */
    uint16_t local_port;       /* Local port to bind to (default: 59152) */
    uint32_t timeout_ms;       /* Connection timeout in milliseconds (0 for no timeout) */
    bool verbose;              /* Enable verbose logging */
} RSI_Config;
```

Configuration structure for initializing the RSI library.

#### RSI_CartesianPosition

```c
typedef struct {
    double x;              /* X position in mm */
    double y;              /* Y position in mm */
    double z;              /* Z position in mm */
    double a;              /* A rotation in degrees */
    double b;              /* B rotation in degrees */
    double c;              /* C rotation in degrees */
    uint64_t timestamp_us; /* Timestamp in microseconds */
    uint32_t ipoc;         /* IPOC value from robot */
} RSI_CartesianPosition;
```

Robot position in Cartesian coordinates.

#### RSI_JointPosition

```c
typedef struct {
    double axis[6];        /* Joint angles in degrees (A1-A6) */
    uint64_t timestamp_us; /* Timestamp in microseconds */
    uint32_t ipoc;         /* IPOC value from robot */
} RSI_JointPosition;
```

Robot position in joint coordinates.

#### RSI_CartesianCorrection

```c
typedef struct {
    double x;              /* X correction in mm */
    double y;              /* Y correction in mm */
    double z;              /* Z correction in mm */
    double a;              /* A correction in degrees */
    double b;              /* B correction in degrees */
    double c;              /* C correction in degrees */
} RSI_CartesianCorrection;
```

Correction data to send to the robot.

#### RSI_Statistics

```c
typedef struct {
    uint64_t packets_received;           /* Total packets received */
    uint64_t packets_sent;               /* Total packets sent */
    double avg_response_time_ms;         /* Average response time in ms */
    double min_response_time_ms;         /* Minimum response time in ms */
    double max_response_time_ms;         /* Maximum response time in ms */
    uint64_t late_responses;             /* Number of responses over 4ms */
    uint64_t connection_lost_count;      /* Number of connection losses */
    bool is_connected;                   /* Current connection status */
    uint64_t last_packet_timestamp_us;   /* Timestamp of last packet */
} RSI_Statistics;
```

Statistics about RSI communication.

#### Callback Types

```c
typedef void (*RSI_DataCallback)(const RSI_CartesianPosition* cartesian, 
                                const RSI_JointPosition* joints,
                                void* user_data);
```

Callback for robot data, called when a new packet is received from the robot.

```c
typedef void (*RSI_ConnectionCallback)(bool connected, void* user_data);
```

Callback for connection status changes.

### Functions

#### RSI_Init

```c
RSI_Error RSI_Init(const RSI_Config* config);
```

Initializes the RSI library with the given configuration. Use `NULL` for default configuration.

**Parameters:**
- `config`: Configuration parameters or `NULL` for defaults

**Returns:**
- `RSI_SUCCESS` on success, error code otherwise

**Default Configuration:**
- Local IP: "0.0.0.0" (all interfaces)
- Local Port: 59152
- Timeout: 1000 ms
- Verbose: false

#### RSI_SetCallbacks

```c
RSI_Error RSI_SetCallbacks(RSI_DataCallback data_callback,
                          RSI_ConnectionCallback connection_callback,
                          void* user_data);
```

Sets callback functions for data and connection events.

**Parameters:**
- `data_callback`: Callback for robot data (can be `NULL`)
- `connection_callback`: Callback for connection status changes (can be `NULL`)
- `user_data`: User data pointer passed to callbacks

**Returns:**
- `RSI_SUCCESS` on success, error code otherwise

#### RSI_Start

```c
RSI_Error RSI_Start(void);
```

Starts the RSI communication thread and begins listening for packets from the robot.

**Returns:**
- `RSI_SUCCESS` on success, error code otherwise

#### RSI_Stop

```c
RSI_Error RSI_Stop(void);
```

Stops the RSI communication thread and closes the socket.

**Returns:**
- `RSI_SUCCESS` on success, error code otherwise

#### RSI_Cleanup

```c
RSI_Error RSI_Cleanup(void);
```

Cleans up resources used by the RSI library. Call after `RSI_Stop()` and before the program exits.

**Returns:**
- `RSI_SUCCESS` on success, error code otherwise

#### RSI_GetCartesianPosition

```c
RSI_Error RSI_GetCartesianPosition(RSI_CartesianPosition* position);
```

Gets the latest Cartesian position of the robot.

**Parameters:**
- `position`: Pointer to structure to receive position data

**Returns:**
- `RSI_SUCCESS` on success, error code otherwise

#### RSI_GetJointPosition

```c
RSI_Error RSI_GetJointPosition(RSI_JointPosition* position);
```

Gets the latest joint position of the robot.

**Parameters:**
- `position`: Pointer to structure to receive position data

**Returns:**
- `RSI_SUCCESS` on success, error code otherwise

#### RSI_SetCartesianCorrection

```c
RSI_Error RSI_SetCartesianCorrection(const RSI_CartesianCorrection* correction);
```

Sends Cartesian correction values to the robot. These corrections will be sent in the next response packet.

**Parameters:**
- `correction`: Correction values

**Returns:**
- `RSI_SUCCESS` on success, error code otherwise

#### RSI_GetStatistics

```c
RSI_Error RSI_GetStatistics(RSI_Statistics* stats);
```

Gets statistics about RSI communication.

**Parameters:**
- `stats`: Pointer to structure to receive statistics

**Returns:**
- `RSI_SUCCESS` on success, error code otherwise

#### RSI_GetErrorString

```c
const char* RSI_GetErrorString(RSI_Error error);
```

Gets a string representation of the given error code.

**Parameters:**
- `error`: Error code

**Returns:**
- String representation of the error code

## Thread Safety

The library is thread-safe for data access. Multiple threads can safely call the API functions concurrently.

Internally, the library uses a dedicated high-priority thread for network communication, which runs separately from the application threads. Critical sections protect access to shared data structures.

## Performance Considerations

### Callbacks

The data callback runs in the context of the high-priority network thread. For best performance:

1. Keep callback functions as fast and lightweight as possible
2. Avoid blocking operations in the callback
3. Don't perform memory allocations in the callback
4. Consider just copying data and signaling another thread for processing

### System Configuration

To achieve optimal performance:

1. Run your application with administrator/root privileges to enable high thread priorities
2. Disable unnecessary background processes and services
3. On Windows, consider using a dedicated network adapter with updated drivers
4. Set your network adapter to use a fixed speed/duplex setting rather than auto-negotiation

## Error Handling

Always check the return values of API functions:

```c
RSI_Error err = RSI_Start();
if (err != RSI_SUCCESS) {
    printf("Failed to start RSI: %s\n", RSI_GetErrorString(err));
    // Handle error...
}
```

## Examples

See the `examples/` directory for complete examples:

- **simple_monitor.c**: Basic position monitoring application
- **jogging.c**: Interactive robot jogging with keyboard controls

## Limitations

- The library only implements the UDP-based version of the RSI protocol
- Only Cartesian corrections are currently supported (no joint corrections)
- The library does not handle RSI configuration on the robot side

## Troubleshooting

### High Response Times

If you experience response times over 4ms:

1. Check system load and background processes
2. Ensure your application has sufficient privileges for high-priority threads
3. Reduce the complexity of your callback functions
4. Check network adapter settings and drivers

### Connection Issues

If the robot fails to connect:

1. Verify the robot's RSI configuration matches your application's settings
2. Check firewall settings and network connectivity
3. Ensure the correct port is configured (default: 59152)
4. Verify the robot's IP address is correct in the RSI configuration file

## License

This library is provided as-is without any warranty. Use at your own risk.

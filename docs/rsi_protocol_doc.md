# KUKA RSI Protocol Documentation

## Overview

The KUKA Robot Sensor Interface (RSI) is a real-time interface for KUKA robots that allows external systems to influence robot movement through a network connection. The RSI protocol operates over UDP and requires extremely low latency responses from the external system.

This document provides an overview of the RSI protocol as implemented in our library, focusing on the essential aspects needed for successful communication.

## Protocol Basics

### Connection Parameters

- **Protocol**: UDP/IP
- **Default Port**: 59152
- **Cycle Time**: 4-12ms (robot dependent, typically 4ms)
- **Data Format**: XML

### Communication Flow

1. The robot sends an XML packet to the external system at each control cycle
2. The external system must parse the packet, extract relevant data, and send a response
3. The response must include the IPOC value from the received packet
4. The robot expects responses within the control cycle time (typically <4ms)
5. Late or missing responses may trigger the robot to stop motion or disconnect

## XML Format

### Robot to External System (Input)

The robot sends XML data containing its current state:

```xml
<Rob Type="KUKA">
  <RIst X="445.0" Y="0.0" Z="790.0" A="180.0" B="0.0" C="180.0"/>
  <RSol X="445.0" Y="0.0" Z="790.0" A="-180.0" B="0.0" C="-180.0"/>
  <AIPos A1="-0.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0"/>
  <ASol A1="-0.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0"/>
  <Delay D="0"/>
  <Tech C11="0.0" C12="0.0" C13="0.0" C14="0.0" C15="0.0" C16="0.0" 
        C17="0.0" C18="0.0" C19="0.0" C110="0.0"/>
  <DiL>0</DiL>
  <Digout o1="0" o2="0" o3="0"/>
  <Source1>8.0</Source1>
  <IPOC>435413237</IPOC>
</Rob>
```

Key elements:
- `<RIst>`: Current Cartesian position (X, Y, Z, A, B, C)
- `<RSol>`: Target Cartesian position
- `<AIPos>`: Current joint angles (A1-A6)
- `<ASol>`: Target joint angles
- `<IPOC>`: Packet counter/timestamp value that must be echoed in the response

### External System to Robot (Output)

The external system responds with XML data containing correction values:

```xml
<Sen Type="ImFree">
  <EStr>RSI Monitor</EStr>
  <RKorr X="0.0000" Y="0.0000" Z="0.0000" A="0.0000" B="0.0000" C="0.0000" />
  <IPOC>435413237</IPOC>
</Sen>
```

Key elements:
- `<Sen Type="ImFree">`: Sensor type identifier (configured on the robot)
- `<EStr>`: Optional status message
- `<RKorr>`: Cartesian correction values (X, Y, Z, A, B, C)
- `<IPOC>`: Must match the IPOC value from the received packet

### Alternative Correction Format

The library also supports joint angle corrections:

```xml
<Sen Type="ImFree">
  <EStr>RSI Monitor</EStr>
  <AKorr A1="0.0000" A2="0.0000" A3="0.0000" A4="0.0000" A5="0.0000" A6="0.0000" />
  <IPOC>435413237</IPOC>
</Sen>
```

Key elements:
- `<AKorr>`: Joint angle corrections (A1-A6)

## Timing Requirements

The RSI protocol has strict timing requirements:

- **Cycle Time**: Typically 4ms (may vary by robot model and configuration)
- **Response Time**: Must be less than the cycle time
- **Late Responses**: Robot may slow down or stop if responses are consistently late
- **Missing Responses**: Robot will stop after a certain number of missing responses

## Robot Configuration

For the RSI interface to work, the robot controller must be configured correctly:

1. RSI software option must be installed on the robot controller
2. An RSI configuration file must be created and loaded
3. The RSI program (usually "RSI_Ethernet.src") must be running on the robot

The RSI configuration file specifies the communication parameters, including:
- IP address and port of the external system
- Sensor type identifier
- Data elements to send and receive

## Error Handling

The RSI protocol has limited error handling capabilities:

- No packet acknowledgment (relies on UDP)
- No explicit error messages
- Robot stops or disconnects if timing requirements are not met
- XML parsing errors may cause disconnection

Our library implements the following error handling strategies:
- Connection timeout detection
- Late response tracking
- Continuous packet counting

## Performance Optimization

To meet the strict timing requirements:

1. The library uses a dedicated high-priority thread for network communication
2. Non-blocking socket operations with continuous polling
3. Minimal XML parsing (direct string operations instead of DOM parsing)
4. Pre-allocated buffers to avoid dynamic memory allocation
5. Careful synchronization to minimize thread contention

## Testing and Troubleshooting

To verify proper RSI communication:

1. Monitor response times using the statistics functions
2. Check for late responses (>4ms)
3. Verify packet counts are continuous
4. Examine connection status changes

Common issues:
- Network latency or packet loss
- Insufficient thread priority
- XML formatting errors
- Incorrect IP address or port configuration
- Missing or incorrect IPOC value in responses

## References

For more detailed information about the RSI protocol, refer to the official KUKA documentation:

- KUKA.RobotSensorInterface 4.0 documentation
- KUKA System Software (KSS) manual
- KUKA Robot Language (KRL) reference

Note that the official KUKA documentation is required for comprehensive understanding of the RSI protocol and its capabilities beyond what is covered in this overview.

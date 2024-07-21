#include <cstdint>
#include <cstring>

// Structure for Set_Input_Vel message
struct Set_Input_Vel {
    uint32_t Input_Torque_FF;
    uint32_t Input_Vel;
};

// Structure for Set_Input_Torque message
struct Set_Input_Torque {
    uint32_t Input_Torque;
};

// Structure for Set_Axis_State message
struct Set_Axis_State {
    uint32_t Axis_Requested_State;
};

// Structure for Clear_Errors message
// This message does not contain any signals
struct Clear_Errors {
};

// Structure for Get_Encoder_Estimates message
struct Get_Encoder_Estimates {
    uint32_t Vel_Estimate;
    uint32_t Pos_Estimate;
};

// Example of encoding a Set_Input_Vel message into a byte array
void encodeSetInputVel(const Set_Input_Vel& message, uint8_t* buffer) {
    std::memset(buffer, 0, 8); // Clear buffer
    // Assuming big-endian byte order
    buffer[0] = (message.Input_Vel >> 24) & 0xFF;
    buffer[1] = (message.Input_Vel >> 16) & 0xFF;
    buffer[2] = (message.Input_Vel >> 8) & 0xFF;
    buffer[3] = message.Input_Vel & 0xFF;
    buffer[4] = (message.Input_Torque_FF >> 24) & 0xFF;
    buffer[5] = (message.Input_Torque_FF >> 16) & 0xFF;
    buffer[6] = (message.Input_Torque_FF >> 8) & 0xFF;
    buffer[7] = message.Input_Torque_FF & 0xFF;
}

// Example of decoding a Set_Input_Vel message from a byte array
void decodeSetInputVel(const uint8_t* buffer, Set_Input_Vel& message) {
    message.Input_Vel = (uint32_t(buffer[0]) << 24) | (uint32_t(buffer[1]) << 16) | (uint32_t(buffer[2]) << 8) | buffer[3];
    message.Input_Torque_FF = (uint32_t(buffer[4]) << 24) | (uint32_t(buffer[5]) << 16) | (uint32_t(buffer[6]) << 8) | buffer[7];
}


// Structure for Set_Input_Torque message
struct Set_Input_Torque {
    uint32_t Input_Torque;
};

// Function to encode Set_Input_Torque message into a byte array
void encodeSetInputTorque(const Set_Input_Torque& message, uint8_t* buffer) {
    std::memset(buffer, 0, 8); // Clear buffer to ensure it starts clean
    // Encode Input_Torque, assuming big-endian byte order
    buffer[0] = (message.Input_Torque >> 24) & 0xFF;
    buffer[1] = (message.Input_Torque >> 16) & 0xFF;
    buffer[2] = (message.Input_Torque >> 8) & 0xFF;
    buffer[3] = message.Input_Torque & 0xFF;
}

// Function to decode Set_Input_Torque message from a byte array
void decodeSetInputTorque(const uint8_t* buffer, Set_Input_Torque& message) {
    // Decode Input_Torque, assuming big-endian byte order
    message.Input_Torque = (uint32_t(buffer[0]) << 24) |
                           (uint32_t(buffer[1]) << 16) |
                           (uint32_t(buffer[2]) << 8) |
                           buffer[3];
}


// Function to encode Set_Axis_State message into a byte array
void encodeSetAxisState(const Set_Axis_State& message, uint8_t* buffer) {
    std::memset(buffer, 0, 8); // Clear buffer to ensure it starts clean
    // Encode Axis_Requested_State, assuming big-endian byte order
    buffer[0] = (message.Axis_Requested_State >> 24) & 0xFF;
    buffer[1] = (message.Axis_Requested_State >> 16) & 0xFF;
    buffer[2] = (message.Axis_Requested_State >> 8) & 0xFF;
    buffer[3] = message.Axis_Requested_State & 0xFF;
}

// Function to decode Set_Axis_State message from a byte array
void decodeSetAxisState(const uint8_t* buffer, Set_Axis_State& message) {
    // Decode Axis_Requested_State, assuming big-endian byte order
    message.Axis_Requested_State = (uint32_t(buffer[0]) << 24) |
                                   (uint32_t(buffer[1]) << 16) |
                                   (uint32_t(buffer[2]) << 8) |
                                   buffer[3];
}


// Structure for Get_Encoder_Estimates message
struct Get_Encoder_Estimates {
    uint32_t Vel_Estimate;
    uint32_t Pos_Estimate;
};

// Function to encode Get_Encoder_Estimates message into a byte array
void encodeGetEncoderEstimates(const Get_Encoder_Estimates& message, uint8_t* buffer) {
    std::memset(buffer, 0, 8); // Clear buffer to ensure it starts clean
    // Encode Vel_Estimate, assuming big-endian byte order
    buffer[0] = (message.Vel_Estimate >> 24) & 0xFF;
    buffer[1] = (message.Vel_Estimate >> 16) & 0xFF;
    buffer[2] = (message.Vel_Estimate >> 8) & 0xFF;
    buffer[3] = message.Vel_Estimate & 0xFF;
    // Encode Pos_Estimate, assuming big-endian byte order
    buffer[4] = (message.Pos_Estimate >> 24) & 0xFF;
    buffer[5] = (message.Pos_Estimate >> 16) & 0xFF;
    buffer[6] = (message.Pos_Estimate >> 8) & 0xFF;
    buffer[7] = message.Pos_Estimate & 0xFF;
}

// Function to decode Get_Encoder_Estimates message from a byte array
void decodeGetEncoderEstimates(const uint8_t* buffer, Get_Encoder_Estimates& message) {
    // Decode Vel_Estimate, assuming big-endian byte order
    message.Vel_Estimate = (uint32_t(buffer[0]) << 24) |
                           (uint32_t(buffer[1]) << 16) |
                           (uint32_t(buffer[2]) << 8) |
                           buffer[3];
    // Decode Pos_Estimate, assuming big-endian byte order
    message.Pos_Estimate = (uint32_t(buffer[4]) << 24) |
                           (uint32_t(buffer[5]) << 16) |
                           (uint32_t(buffer[6]) << 8) |
                           buffer[7];
}


// Function to encode Clear_Errors message into a byte array
void encodeClearErrors(uint8_t* buffer) {
    std::memset(buffer, 0, 8); // Clear buffer to ensure it starts clean
    // No data to encode, but buffer is prepared
}

// Function to decode Clear_Errors message from a byte array
void decodeClearErrors(const uint8_t* buffer, Clear_Errors& message) {
    // No data to decode
    // This function primarily serves to handle the message format
}

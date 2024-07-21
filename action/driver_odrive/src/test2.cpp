#include <stdint.h>
#include <iostream>
#include <stdint.h>
#include <cstring> // For memset

struct SetInputVel {
    // The actual CAN message ID should be defined elsewhere or handled by a CAN manager
    static const uint32_t CAN_ID = 13; // Message ID
    static const uint8_t CAN_DLC = 8; // Data Length Code

    // Signal: Input_Torque_FF
    uint32_t Input_Torque_FF; // 32 bits unsigned

    // Signal: Input_Vel
    uint32_t Input_Vel; // 32 bits unsigned

    // A method to serialize this structure's data into a CAN frame
    void serialize(uint8_t* buffer) {
        buffer[0] = (Input_Torque_FF >> 24) & 0xFF;
        buffer[1] = (Input_Torque_FF >> 16) & 0xFF;
        buffer[2] = (Input_Torque_FF >> 8) & 0xFF;
        buffer[3] = Input_Torque_FF & 0xFF;

        buffer[4] = (Input_Vel >> 24) & 0xFF;
        buffer[5] = (Input_Vel >> 16) & 0xFF;
        buffer[6] = (Input_Vel >> 8) & 0xFF;
        buffer[7] = Input_Vel & 0xFF;
    }

    // A method to deserialize CAN frame data into this structure's fields
    void deserialize(const uint8_t* buffer) {
        Input_Torque_FF = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
        Input_Vel = (buffer[4] << 24) | (buffer[5] << 16) | (buffer[6] << 8) | buffer[7];
    }
};

int main() {
    SetInputVel message;
    message.Input_Torque_FF = 123456; // Example torque value
    message.Input_Vel = 654321; // Example velocity value

    // Create a buffer to hold the serialized data
    uint8_t buffer[SetInputVel::CAN_DLC];
    memset(buffer, 0, SetInputVel::CAN_DLC); // Initialize buffer to zero

    // Serialize the message into the buffer
    message.serialize(buffer);

    // Print the buffer contents in hexadecimal
    std::cout << "Serialized CAN Data: ";
    for (int i = 0; i < SetInputVel::CAN_DLC; ++i) {
        std::cout << std::hex << (int)buffer[i] << " ";
    }
    std::cout << std::endl;

    // At this point, you would typically send the buffer over the CAN network
    // Example: canInterface.send(message.CAN_ID, buffer, SetInputVel::CAN_DLC);

    return 0;
}
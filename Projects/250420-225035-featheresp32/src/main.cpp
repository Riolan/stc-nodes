#include <SPI.h>
#include <Seeed_Arduino_SSCMA.h> // Assuming this is needed for AI

#include "node_shared.hpp" // Includes LoRaPacket, constants, rf95 declaration
// #include "ReceiveManager.hpp" // Include if receiver object is actually used beyond the callback setup
#include "SendManager.hpp"    // Include SendManager definition for getInstance()
#include "JoinManager.hpp"    // Contains declaration for performJoin
#include "packet_types.h"     // Contains packet type definitions
#include "jpeg_test.hpp" // Include if using JPEG sample data

// --- Function Prototype ---
int estimateBatteryPercentage(float voltage);



/// --- Battery Information ---
const int VBAT_PIN = 35; // ESP32 Huzzah internal battery sense pin (A13) - Verify for your board!
const float ADC_REF_VOLTAGE = 3.3; // Or measure your actual 3.3V rail
const int ADC_RESOLUTION = 4096; // 12-bit ADC
// Check your board's schematic! The divider factor might vary.
// Adafruit Feathers often use two 100k resistors (Factor = 2.0)
// Some other boards might use different values.
const float VOLTAGE_DIVIDER_FACTOR = 2.0;
const float EMA_ALPHA = 0.2; // Smoothing factor for battery percentage

// How often to check the battery voltage (e.g., every 60 seconds)
const unsigned long BATTERY_CHECK_INTERVAL = 60000;
// How often to send the battery status over LoRa (e.g., every 10 minutes)
const unsigned long BATTERY_SEND_INTERVAL = 600000;
float smoothedBatteryVoltage = -1.0; // Use voltage for EMA for better accuracy
uint8_t lastSentBatteryPercent = 255; // Store last sent value (init to invalid)
unsigned long lastBatteryCheck = 0;
unsigned long lastBatterySend = 0;


// --- Image Upload State ---
enum class ImageUploadState {
    IDLE,             // Not currently uploading
    SENDING_CHUNK,    // Ready to send the next chunk (or waiting for SendManager to be idle)
    WAITING_CHUNK_ACK // Chunk sent, waiting for specific ACK from Mother Node
};
ImageUploadState imageUploadState = ImageUploadState::IDLE;
const uint8_t* imageDataSourcePtr = nullptr; // Pointer to image data (e.g., testImageJpegBytes)
size_t totalImageSize = 0;             // Total size of the image being sent
size_t imageBytesSent = 0;             // How many bytes have been successfully ACKed
uint16_t currentChunkNumber = 0;       // Index of the next chunk to *send* (0-based)
uint16_t totalChunks = 0;              // Total number of chunks for the current image
uint32_t currentImageIdentifier = 0;   // ID of the image being uploaded (from request)
uint8_t imageUploadTargetNodeID = 0;  // Who requested the image (Mother Node ID)
uint8_t lastSentChunkSeqNum = 0;      // Track sequence number of the chunk waiting for ACK
unsigned long chunkSendTimestamp = 0;   // Time the chunk waiting for ACK was *successfully initiated* by SendManager
const unsigned long IMAGE_CHUNK_ACK_TIMEOUT = ACK_TIMEOUT + 2000; // Allow more time for chunk ACKs (ACK_TIMEOUT is from node_shared.hpp)
uint8_t chunkResendAttempts = 0;
const uint8_t MAX_CHUNK_RESEND_ATTEMPTS = 3;

// --- Global Variables ---
// lora::ReceiveManager receiver; // Declare if used. Callback is set, but loop handles reception directly?
SSCMA AI;

bool joined = false;         // Flag indicating if node has joined the network
uint8_t myNodeID = 0xFF;     // This node's assigned ID (0xFF = unassigned)
uint32_t deviceID = 0;       // Unique device ID (e.g., from MAC)
unsigned long lastSend = 0;      // Timestamp for last data send attempt
unsigned long lastHeartbeat = 0; // Timestamp for last heartbeat send
unsigned long lastMotherHeartbeat = 0;

// Timeout in milliseconds for Mother Node heartbeat
#define MOTHER_HEARTBEAT_TIMEOUT 30000 // e.g., 30 seconds (adjust as needed)
// Assume Mother Node ID is 0x00
#define MOTHER_NODE_ID 0x00

// --- Function Implementations ---

// Simple check if channel is free using CAD
bool channelIsClear() {
    // Note: waitCAD() is blocking, but usually very short.
    // For truly non-blocking, a state machine checking CAD results would be needed.
    return lora::rf95.waitCAD();
}





// --- Helper to initiate the upload process ---
void startImageUpload(uint32_t imageId, uint8_t targetNodeId) {
    // Check if already uploading
    if (imageUploadState != ImageUploadState::IDLE) {
        Serial.println("WARN: Already uploading an image. Ignoring new request.");
        return;
    }

    // For now, we only have one test image
    // In a real scenario, imageId would be used to select the correct image data source (e.g., from SD card)
    Serial.print("Image upload requested for ID: "); Serial.print(imageId);
    Serial.print(" to Node: 0x"); Serial.println(targetNodeId, HEX);

    // --- Select Image Source ---
    // TODO: Replace this with logic to load image 'imageId' from SD card
    imageDataSourcePtr = testImageJpegBase64; // Point to the test data from the header
    totalImageSize = 4036 ; //Hard coded for now. // Use the size variable from the header
    // --- End Image Source Selection ---

    if (totalImageSize == 0 || imageDataSourcePtr == nullptr) {
        Serial.println("ERROR: Image data is empty or invalid!");
        // TODO: Send an error status back to mother node?
        imageUploadState = ImageUploadState::IDLE;
        return;
    }

    imageUploadTargetNodeID = targetNodeId;
    currentImageIdentifier = imageId; // Store the requested ID
    imageBytesSent = 0;
    currentChunkNumber = 0; // Start with chunk 0
    chunkResendAttempts = 0;

    // Calculate total chunks needed
    // Overhead: ImgID(4)+Chunk#(2)+TotalChunks(2) = 8 bytes in payload before chunk data
    size_t payloadCapacity = MAX_PAYLOAD_SIZE - 8;
    if (payloadCapacity <= 0 || payloadCapacity > MAX_PAYLOAD_SIZE) { // Basic sanity check
         Serial.println("ERROR: MAX_PAYLOAD_SIZE too small or invalid for image chunk overhead!");
         imageUploadState = ImageUploadState::IDLE;
         return;
    }
    totalChunks = (totalImageSize + payloadCapacity - 1) / payloadCapacity; // Ceiling division

    imageUploadState = ImageUploadState::SENDING_CHUNK; // Set state to start sending

    Serial.print("Starting image upload: Size="); Serial.print(totalImageSize);
    Serial.print(" bytes, Chunks="); Serial.println(totalChunks);
}

// --- Helper to send the next image chunk ---
void sendNextImageChunk() {
    // Check if we are in the correct state and if sender is ready
     if (imageUploadState != ImageUploadState::SENDING_CHUNK || !lora::SendManager::getInstance().isIdle()) {
        // Not ready to send (either wrong state or sender busy with previous TX confirmation)
        return;
    }

    if (imageBytesSent >= totalImageSize) {
        // Should not happen if state is SENDING_CHUNK, but safety check
        Serial.println("INFO: All image bytes already marked as sent. Completing upload.");
        imageUploadState = ImageUploadState::IDLE; // Finished
        // TODO: Send IMAGE_UPLOAD_STATUS (Success) packet?
        return;
    }

    // Calculate chunk size
    size_t remainingBytes = totalImageSize - imageBytesSent;
    // Overhead: ImgID(4)+Chunk#(2)+TotalChunks(2) = 8 bytes
    size_t payloadCapacity = MAX_PAYLOAD_SIZE - 8;
    size_t chunkSize = std::min(remainingBytes, payloadCapacity);

    // Prepare payload buffer
    // Use dynamic allocation or ensure MAX_PAYLOAD_SIZE is large enough for header+data
    uint8_t chunkPayload[MAX_PAYLOAD_SIZE]; // Use max size buffer

    // 1. Image Identifier (4 bytes) - Network Byte Order (Big Endian) often preferred
    chunkPayload[0] = (currentImageIdentifier >> 24) & 0xFF;
    chunkPayload[1] = (currentImageIdentifier >> 16) & 0xFF;
    chunkPayload[2] = (currentImageIdentifier >> 8) & 0xFF;
    chunkPayload[3] = currentImageIdentifier & 0xFF;
    // 2. Current Chunk Number (2 bytes) - Network Byte Order
    chunkPayload[4] = (currentChunkNumber >> 8) & 0xFF;
    chunkPayload[5] = currentChunkNumber & 0xFF;
    // 3. Total Chunks (2 bytes) - Network Byte Order
    chunkPayload[6] = (totalChunks >> 8) & 0xFF;
    chunkPayload[7] = totalChunks & 0xFF;
    // 4. Chunk Data (chunkSize bytes)
    memcpy(&chunkPayload[8], imageDataSourcePtr + imageBytesSent, chunkSize);

    Serial.print("Sending IMAGE_CHUNK #"); Serial.print(currentChunkNumber);
    Serial.print("/"); Serial.print(totalChunks); // Use 1-based total for clarity
    Serial.print(" (Offset: "); Serial.print(imageBytesSent);
    Serial.print(", Size: "); Serial.print(chunkSize); Serial.println(" bytes)");

    // Send the chunk using SendManager, require ACK
    // Get the sequence number used by SendManager *before* sending
    lastSentChunkSeqNum = lora::SendManager::getInstance().getNextSeqNum(); // Need this method in SendManager

    bool initiated = lora::SendManager::getInstance().send(
        imageUploadTargetNodeID,
        lora::PACKET_TYPE_IMAGE_CHUNK,
        0, // Subtype not used for IMAGE_CHUNK type
        chunkPayload,
        8 + chunkSize, // Total payload size (metadata + data)
        true // Require ACK for each chunk
    );

    if (initiated) {
        Serial.print("  Chunk send initiated with SeqNum: "); Serial.println(lastSentChunkSeqNum);
        imageUploadState = ImageUploadState::WAITING_CHUNK_ACK; // Move to waiting state
        chunkSendTimestamp = millis(); // Record time send was initiated
        chunkResendAttempts = 0; // Reset resend attempts for this chunk
        // DO NOT increment imageBytesSent or currentChunkNumber here. Do it when ACK is received.
    } else {
         Serial.println("  Image chunk send failed to initiate (radio busy?). Will retry.");
         // Stay in SENDING_CHUNK state, will try again next loop if sender is idle
    }
}

// --- Helper to handle receiving an ACK for an image chunk ---
void handleImageChunkAck(uint8_t ackedSeqNum) {
    if (imageUploadState == ImageUploadState::WAITING_CHUNK_ACK && ackedSeqNum == lastSentChunkSeqNum) {
        Serial.print("ACK received for Image Chunk #"); Serial.println(currentChunkNumber);

        // Calculate how many bytes were in the ACKed chunk
        size_t remainingBytes = totalImageSize - imageBytesSent;
        size_t payloadCapacity = MAX_PAYLOAD_SIZE - 8;
        size_t chunkSize = std::min(remainingBytes, payloadCapacity);

        // Advance counters
        imageBytesSent += chunkSize;
        currentChunkNumber++;

        // Check if upload is complete
        if (imageBytesSent >= totalImageSize) {
            Serial.println("Image upload complete!");
            imageUploadState = ImageUploadState::IDLE;
            // TODO: Optionally send IMAGE_UPLOAD_STATUS (Success) packet
        } else {
            // More chunks to send, go back to sending state
            imageUploadState = ImageUploadState::SENDING_CHUNK;
            chunkResendAttempts = 0; // Reset for next chunk
            // Immediately try sending next chunk in the same loop iteration if possible
            sendNextImageChunk();
        }
    } else if (imageUploadState == ImageUploadState::WAITING_CHUNK_ACK) {
         Serial.print("WARN: Received ACK with SeqNum "); Serial.print(ackedSeqNum);
         Serial.print(" but was waiting for "); Serial.println(lastSentChunkSeqNum);
         // Ignore this ACK for the image upload process, might be for something else
    }
    // If not waiting for chunk ACK, ignore the ACK in this context
}

// --- Helper to resend the last image chunk ---
void resendLastImageChunk() {
     if (imageUploadState != ImageUploadState::WAITING_CHUNK_ACK) return; // Should not happen

     chunkResendAttempts++;
     Serial.print("Resending IMAGE_CHUNK #"); Serial.print(currentChunkNumber);
     Serial.print(" (Attempt "); Serial.print(chunkResendAttempts);
     Serial.print("/"); Serial.print(MAX_CHUNK_RESEND_ATTEMPTS); Serial.println(")...");

     // Recalculate chunk size and payload (same logic as sendNextImageChunk)
     size_t remainingBytes = totalImageSize - imageBytesSent;
     size_t payloadCapacity = MAX_PAYLOAD_SIZE - 8;
     size_t chunkSize = std::min(remainingBytes, payloadCapacity);
     uint8_t chunkPayload[MAX_PAYLOAD_SIZE];
     chunkPayload[0] = (currentImageIdentifier >> 24) & 0xFF; // ImgID
     chunkPayload[1] = (currentImageIdentifier >> 16) & 0xFF;
     chunkPayload[2] = (currentImageIdentifier >> 8) & 0xFF;
     chunkPayload[3] = currentImageIdentifier & 0xFF;
     chunkPayload[4] = (currentChunkNumber >> 8) & 0xFF; // Chunk #
     chunkPayload[5] = currentChunkNumber & 0xFF;
     chunkPayload[6] = (totalChunks >> 8) & 0xFF; // Total Chunks
     chunkPayload[7] = totalChunks & 0xFF;
     memcpy(&chunkPayload[8], imageDataSourcePtr + imageBytesSent, chunkSize); // Data

     // Use the *same* sequence number for the resend if SendManager supports it,
     // otherwise SendManager will assign a new one. Let's assume SendManager handles it.
     // We still need to track the *new* seq num if SendManager assigns one.
     // For simplicity now, let SendManager assign a new one and update our tracking.
     lastSentChunkSeqNum = lora::SendManager::getInstance().getNextSeqNum();

     bool initiated = lora::SendManager::getInstance().send(
        imageUploadTargetNodeID, lora::PACKET_TYPE_IMAGE_CHUNK, 0,
        chunkPayload, 8 + chunkSize, true );

     if (initiated) {
        Serial.print("  Chunk RESEND initiated with SeqNum: "); Serial.println(lastSentChunkSeqNum);
        imageUploadState = ImageUploadState::WAITING_CHUNK_ACK; // Stay waiting
        chunkSendTimestamp = millis(); // Reset ACK timeout timer
     } else {
        Serial.println("  Chunk RESEND failed to initiate (radio busy?).");
        // Stay in WAITING_CHUNK_ACK, timeout will trigger again later.
        // Decrement attempt count since it didn't even start? Or keep it? Let's keep it.
     }
}

// Helper function to send a standard ACK packet
void sendPacketAck(uint8_t targetNodeID, uint8_t seqNumToAck) {
    // Serial.print("Sending ACK to Node 0x"); Serial.print(targetNodeID, HEX); // Can be verbose
    // Serial.print(" for SeqNum: "); Serial.println(seqNumToAck);
    uint8_t ackPayload[] = { seqNumToAck }; // ACK payload is the sequence number being acknowledged
    // Use SendManager. ACKs don't require an ACK (requireAck = false).
    bool initiated = lora::SendManager::getInstance().send(
        targetNodeID,
        lora::PACKET_TYPE_ACK,
        0,
        ackPayload,
        sizeof(ackPayload),
        false
    );
    // if (!initiated) { Serial.println("  WARN: Failed to initiate ACK send."); }
  }


void setup() {
    // Generate a unique ID for this device (e.g., from MAC address)
    deviceID = (uint32_t)ESP.getEfuseMac();

    // Radio Reset Sequence
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH); delay(10);
    digitalWrite(RFM95_RST, LOW); delay(10);
    digitalWrite(RFM95_RST, HIGH); delay(10);

    Serial.begin(115200);
    while (!Serial && millis() < 2000); // Wait for serial console (with timeout)
    Serial.println("\nEdge Node Starting...");
    Serial.print("Device ID: 0x"); Serial.println(deviceID, HEX);

    // Initialize AI module
    if (!AI.begin()) {
        Serial.println("FATAL: AI module initialization failed.");
        while (1); // Halt
    }
    Serial.println("AI module initialized.");

    // Initialize LoRa Radio
    Serial.println("Initializing LoRa Radio...");
    if (!lora::rf95.init()) {
        Serial.println("FATAL: LoRa radio init failed");
        while (1); // Halt
    }
    Serial.println("LoRa radio init OK!");

    // Apply Radio Settings (Frequency, Power, etc.)
    // Consider putting these in lora::configureRadioSettings()
     if (!lora::rf95.setFrequency(915.0)) {
         Serial.println("setFrequency failed"); /* handle error */
         while(1);
    }
    lora::rf95.setTxPower(13, false); // Adjust power as needed



    // --- Configure ADC for Battery Reading ---
    Serial.println("Configuring ADC for battery monitoring...");
    analogSetPinAttenuation(VBAT_PIN, ADC_11db); // Use 11dB attenuation for full range (0-3.3V+) reading

    // --- Initial Battery Read (Optional but good) ---
    // Perform an initial read to start the EMA
    int rawValue = analogRead(VBAT_PIN);
    float pinVoltage = (float)rawValue / (ADC_RESOLUTION - 1) * ADC_REF_VOLTAGE;
    smoothedBatteryVoltage = pinVoltage * VOLTAGE_DIVIDER_FACTOR; // Initialize smoothed value
    int initialPercent = estimateBatteryPercentage(smoothedBatteryVoltage);
    lastSentBatteryPercent = (uint8_t)initialPercent; // Initialize last sent value
    Serial.print("Initial Battery Voltage: "); Serial.print(smoothedBatteryVoltage, 2);
    Serial.print("V ("); Serial.print(initialPercent); Serial.println("%)");
    lastBatteryCheck = millis();
    lastBatterySend = millis(); // Send initial status soon


    // --- Join Network ---
    // IMPORTANT: lora::performJoin MUST be modified to be non-blocking
    // and use lora::SendManager::getInstance() internally.
    // Alternatively, implement join logic as a state machine in loop().
    Serial.println("Attempting to join network...");
    if (lora::performJoin(myNodeID, deviceID, -1)) { 
        Serial.print("Join successful! Assigned Node ID: "); Serial.println(myNodeID);
        joined = true;
    } else {
        Serial.println("Join failed after retries. System halted.");
        // Consider implementing retry logic or deep sleep here instead of halting
        while (1);
    }

    // --- Setup Complete ---
    Serial.println("Setup complete. Starting main loop.");
    lastSend = millis(); // Initialize timestamps
    lastHeartbeat = millis();
}


void loop() {
    unsigned long now = millis(); // Get time once per loop

    // ----- 1. MANDATORY: Update the SendManager state machine -----
    // Handles TX Done confirmations and general ACK timeouts for SendManager's buffer
    lora::SendManager::getInstance().update();

    // ----- 2. Check for Incoming LoRa Packets -----
    lora::LoRaPacket packet;
    if (lora::receivePacket(&packet)) { // Non-blocking check for received packets
        lastMotherHeartbeat = now; // Start heartbeat timer upon joining

        // Update Mother Node heartbeat timer if packet received from it
        // (Counts any valid packet from Mother as proof of connection)
        if (packet.nodeID == MOTHER_NODE_ID) {
            lastMotherHeartbeat = now;
            if (!joined) { // Reconnected via non-join packet
                 Serial.println("Re-established connection with Mother Node.");
                 // If we lost connection, we might need the assigned ID again if it was reset
                 // For now, just mark as joined. Mother should ideally resend JOIN_ACK if needed.
                 joined = true;
            }
        }

        // Process received packet based on type
        switch(packet.type) {
            case lora::PACKET_TYPE_JOIN_ACK:
                // Handle join confirmation (likely redundant if performJoin handles it, but safe)
                if (!joined && packet.payloadSize >= 1) {
                    uint8_t assignedID = packet.payload[0];
                    myNodeID = assignedID;
                    joined = true;
                    lastMotherHeartbeat = now; // Start heartbeat timer upon joining
                    Serial.print("Received JOIN_ACK. Assigned Node ID: "); Serial.println(myNodeID);
                }
                break;

            case lora::PACKET_TYPE_ACK:
                 // Handle ACK for data we sent
                 if (packet.payloadSize >= 1) {
                    uint8_t ackedSeqNum = packet.payload[0];
                    Serial.print("Received ACK from Node 0x"); Serial.print(packet.nodeID, HEX);
                    Serial.print(" for SeqNum: "); Serial.println(ackedSeqNum);

                    // Check if this ACK is for the image chunk we are waiting for
                    handleImageChunkAck(ackedSeqNum); // Specific handler for image ACKs

                    // Let SendManager also handle it for its internal buffer cleanup
                    // It might log a warning if handleImageChunkAck already handled it, which is okay.
                    lora::SendManager::getInstance().handleAck(packet.nodeID, ackedSeqNum);
                 } else {
                     Serial.println("WARN: Received ACK packet with invalid payload size.");
                 }
                break;

            case lora::PACKET_TYPE_COMMAND:
                 // ACK the command back to the sender if requested BEFORE processing
                 if (packet.flags & 0x02) {
                     sendPacketAck(packet.nodeID, packet.seqNum);
                 }

                 // Process command subtype
                 switch(packet.subtype) {
                    case lora::SUBTYPE_REQUEST_IMAGE: { // <<< Add opening brace
                        Serial.print("Received REQUEST_IMAGE command from Node 0x"); Serial.println(packet.nodeID, HEX);
                        uint32_t requestedImageId = 0; // Initialization is now safe inside its own scope
                        if (packet.payloadSize == sizeof(uint32_t)) {
                            requestedImageId = ((uint32_t)packet.payload[0] << 24) |
                                               ((uint32_t)packet.payload[1] << 16) |
                                               ((uint32_t)packet.payload[2] << 8) |
                                               ((uint32_t)packet.payload[3]);
                        }
                        startImageUpload(requestedImageId, packet.nodeID);
                        break;
                    } // <<< Add closing brace
   
                    case lora::SUBTYPE_REBOOT: { // <<< Add opening brace (good practice even if no variables yet)
                     //   Serial.println("Received REBOOT command. Rebooting...");
                      //  delay(1000);
                        //ESP.restart();
                        // break; // Unreachable after restart, but technically correct
                    } // <<< Add closing brace
   
                    // Add braces for other cases if they declare variables
                    case lora::SUBTYPE_REQUEST_STATUS: {
                       Serial.println("Received REQUEST_STATUS command (Not Implemented)");
                       break;
                    }
                    case lora::SUBTYPE_REQUEST_CONFIG: {
                       Serial.println("Received REQUEST_CONFIG command (Not Implemented)");
                       break;
                    }
   
                    default: { // <<< Add opening brace
                        Serial.print("Received unknown COMMAND Subtype 0x"); Serial.println(packet.subtype, HEX);
                        break;
                    } // <<< Add closing brace
                } // End switch(packet.subtype)
                 break; // Break from COMMAND case

            // Handle other packet types (DATA from Mother, HEARTBEAT from Mother)
            case lora::PACKET_TYPE_DATA:
            case lora::PACKET_TYPE_HEARTBEAT:
                 // Already updated lastMotherHeartbeat timer above
                 // Serial.print("Received packet type "); Serial.print(packet.type); Serial.print(" from Mother."); // Verbose
                 break;

            default:
                 Serial.print("Received unhandled packet type 0x"); Serial.print(packet.type, HEX);
                 Serial.print(" from Node 0x"); Serial.println(packet.nodeID, HEX);
                 break;
        } // End switch(packet.type)
    } // End if receivePacket

    // ----- 3. Check for Mother Node Heartbeat Timeout -----
    if (joined && (now - lastMotherHeartbeat > MOTHER_HEARTBEAT_TIMEOUT)) {
        Serial.println("!!! Lost connection to Mother Node (Heartbeat Timeout) !!!");
        joined = false; // Mark as disconnected
        myNodeID = 0xFF; // Reset assigned Node ID
        imageUploadState = ImageUploadState::IDLE; // Abort any ongoing upload

        // --- Initiate Rejoin Strategy (Example: Call blocking performJoin) ---
        Serial.println("Attempting to rejoin network immediately...");
        if (lora::performJoin(myNodeID, deviceID, 5)) { // Try 5 times
            Serial.println("Rejoin successful!");
            lastMotherHeartbeat = millis(); // Reset timer immediately
            // joined is set true inside performJoin on success
        } else {
            Serial.println("Immediate rejoin attempt failed. Will rely on periodic retries.");
            // joined remains false
        }
        // --- End Rejoin Strategy ---
    }

    // ----- 4. Periodic Battery Check -----
    if (now - lastBatteryCheck > BATTERY_CHECK_INTERVAL) {
        lastBatteryCheck = now;
        int rawValue = analogRead(VBAT_PIN);
        float pinVoltage = (float)rawValue / (ADC_RESOLUTION - 1) * ADC_REF_VOLTAGE;
        float currentVoltage = pinVoltage * VOLTAGE_DIVIDER_FACTOR;
        if (smoothedBatteryVoltage < 0.0) { smoothedBatteryVoltage = currentVoltage; }
        else { smoothedBatteryVoltage = (currentVoltage * EMA_ALPHA) + (smoothedBatteryVoltage * (1.0 - EMA_ALPHA)); }
        // Serial.print("Smoothed Battery Voltage: "); Serial.println(smoothedBatteryVoltage, 2); // Verbose
    }

    // ----- 5. Handle Image Upload State Machine -----
    if (imageUploadState == ImageUploadState::SENDING_CHUNK) {
        // Try to send the next chunk (function checks if sender is idle)
        sendNextImageChunk();
    }
    else if (imageUploadState == ImageUploadState::WAITING_CHUNK_ACK) {
        // Check if we've timed out waiting for the ACK for the last sent chunk
        if (now - chunkSendTimestamp > IMAGE_CHUNK_ACK_TIMEOUT) {
            Serial.print("Timeout waiting for ACK for Image Chunk #"); Serial.print(currentChunkNumber);
            Serial.print(" (SeqNum "); Serial.print(lastSentChunkSeqNum); Serial.println(")");

            if (chunkResendAttempts < MAX_CHUNK_RESEND_ATTEMPTS) {
                // Increment counter and go back to SENDING state to trigger resend logic
                chunkResendAttempts++;
                Serial.print("Attempting resend #"); Serial.println(chunkResendAttempts);
                imageUploadState = ImageUploadState::SENDING_CHUNK;
            } else {
                Serial.println("Max resend attempts reached for chunk. Aborting image upload.");
                imageUploadState = ImageUploadState::IDLE;
                // TODO: Send IMAGE_UPLOAD_STATUS (Failure) packet to Mother Node
                // lora::SendManager::getInstance().send(...DATA, SUBTYPE_IMAGE_UPLOAD_STATUS, ...);
            }
        }
    } // End image upload state handling


    // ----- 6. Perform Periodic Actions Based on Joined Status -----
    if (joined && imageUploadState == ImageUploadState::IDLE) { // Only if joined AND not busy uploading
        // --- Send Periodic Battery Status ---
        if (now - lastBatterySend > BATTERY_SEND_INTERVAL) {
            int currentSmoothedPercent = estimateBatteryPercentage(smoothedBatteryVoltage);
            if (abs(currentSmoothedPercent - (int)lastSentBatteryPercent) >= 2 || lastSentBatteryPercent == 255) {
                 uint8_t percentPayload = (uint8_t)constrain(currentSmoothedPercent, 0, 100);
                 // Serial.print("Attempting to send Battery Status: "); Serial.print(percentPayload); Serial.println("%"); // Verbose
                 bool initiated = lora::SendManager::getInstance().send( MOTHER_NODE_ID, lora::PACKET_TYPE_DATA, lora::SUBTYPE_BATTERY_STATUS, &percentPayload, 1, false );
                 if (initiated) { lastBatterySend = now; lastSentBatteryPercent = percentPayload; }
                 // else { Serial.println("  Battery Status send failed to initiate..."); } // Verbose
            } else { lastBatterySend = now; } // Update timer even if not sent
        }

        // --- Send Periodic Data (Example) ---
        if (now - lastSend > 15000) {
             if (channelIsClear()) {
                 const char msg[] = "Edge node data payload!";
                 bool initiated = lora::SendManager::getInstance().send( MOTHER_NODE_ID, lora::PACKET_TYPE_DATA, 1, (uint8_t*)msg, strlen(msg), true);
                 if (initiated) { lastSend = now; }
                 // else { Serial.println("  Data send failed to initiate..."); } // Verbose
             }
             // else { Serial.println("Channel busy, delaying data send."); } // Verbose
        }

        // --- Send Periodic Heartbeat (From Edge Node to Mother) ---
        if (now - lastHeartbeat > 30000) {
             bool initiated = lora::SendManager::getInstance().send( MOTHER_NODE_ID, lora::PACKET_TYPE_HEARTBEAT, 0, nullptr, 0, false);
             if (initiated) { lastHeartbeat = now; }
             // else { Serial.println("  Heartbeat send failed to initiate..."); } // Verbose
        }

        // --- Perform AI Inference or other tasks ---
        // ...

    } else if (!joined) { // Not currently joined
        // --- Periodic Rejoin Logic ---
        static unsigned long lastJoinAttempt = 0;
        if (now - lastJoinAttempt > 60000) { // Try joining every 60 seconds when disconnected
             lastJoinAttempt = now;
              Serial.println("Attempting periodic rejoin...");
              // Call performJoin with limited retries.
              if (lora::performJoin(myNodeID, deviceID, 2)) { // Try only twice periodically
                    Serial.println("Periodic rejoin successful!");
                    lastMotherHeartbeat = millis(); // Reset timer
                    // joined is set true inside performJoin
              } else {
                   Serial.println("Periodic rejoin attempt failed.");
                   // joined remains false
              }
         }
         // --- End Periodic Rejoin Logic ---

    } // End if(joined && IDLE) / else (!joined)

    // delay(1); // Optional small delay if loop has little work
} // End loop()



// --- Function to Estimate Battery Percentage ---
int estimateBatteryPercentage(float voltage) {
  if (voltage >= 4.20) { return 100; }
  else if (voltage >= 4.10) { return map(voltage * 100, 410, 420, 90, 100); }
  else if (voltage >= 4.00) { return map(voltage * 100, 400, 410, 80, 90); }
  else if (voltage >= 3.90) { return map(voltage * 100, 390, 400, 70, 80); }
  else if (voltage >= 3.80) { return map(voltage * 100, 380, 390, 55, 70); }
  else if (voltage >= 3.70) { return map(voltage * 100, 370, 380, 40, 55); }
  else if (voltage >= 3.60) { return map(voltage * 100, 360, 370, 25, 40); }
  else if (voltage >= 3.50) { return map(voltage * 100, 350, 360, 10, 25); }
  else if (voltage >= 3.30) { return map(voltage * 100, 330, 350, 0, 10); }
  else { return 0; } // Below 3.3V is considered 0%
}
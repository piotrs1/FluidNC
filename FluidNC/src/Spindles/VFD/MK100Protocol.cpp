/*
    MK100Spindle.cpp
*/

#include "MK100Protocol.h"
#include "Spindles/VFDSpindle.h"

namespace Spindles {
    namespace VFD {
        void MK100Protocol::direction_command(SpindleState mode, ModbusCommand& data) {
            log_info("MK100 mode " << (uint8_t)mode);

            data.tx_length = 6;
            data.rx_length = 6;

            data.msg[1] = 0x06;  // WRITE
            data.msg[2] = 0x20;  // Command ID 0x2000
            data.msg[3] = 0x00;
            data.msg[4] = 0x00;
            data.msg[5] = (mode == SpindleState::Ccw) ? 0x02 : (mode == SpindleState::Cw ? 0x01 : 0x06);
        }

        void MK100Protocol::set_speed_command(uint32_t dev_speed, ModbusCommand& data) {
            // NOTE: MK100 inverters are a-symmetrical. You set the speed in 1/100
            // percentages, and you get the speed in RPM. So, we need to convert
            // the RPM using maxRPM to a percentage. See MD document for details.
            //
            // For the MK100 VFD, the speed is read directly units of RPM, unlike many
            // other VFDs where it is given in Hz times some scale factor.
            log_info("MK100 speed: " << dev_speed);

            data.tx_length = 6;
            data.rx_length = 6;
            uint16_t maxRPM = 4000;
            uint16_t speed = (uint32_t(dev_speed) * 10000L) / uint32_t(maxRPM);
            if (speed > 10000) {
                speed = 10000;
            }
            log_info("MK100 dev speed: " << speed);
            data.msg[1] = 0x06;  // WRITE
            data.msg[2] = 0x10;  // Command ID 0x1000
            data.msg[3] = 0x00;
            data.msg[4] = speed >> 8;
            data.msg[5] = speed & 0xFF;
        }

        VFDProtocol::response_parser MK100Protocol::initialization_sequence(int index, ModbusCommand& data, VFDSpindle* vfd) {
            log_info("MK100 spindle initialization");
            //return nullptr;

            if (index == -1) {
                data.tx_length = 6;
                data.rx_length = 6;

                data.msg[1] = 0x06;
                data.msg[2] = 0xF0;
                data.msg[3] = 0x02;
                data.msg[4] = 0x00;
                data.msg[5] = 0x02;

                return [](const uint8_t* response, VFDSpindle* vfd, VFDProtocol* detail) -> bool {
                    uint16_t maxRPM = 24000;
                    if (vfd->_speeds.size() == 0) {
                        vfd->shelfSpeeds(0, maxRPM);
                    }
                    uint16_t _maxFrequency = 4000;
                    //vfd->setupSpeeds(4000);  // The speed is given directly in RPM
                    vfd->setupSpeeds(_maxFrequency);
                    vfd->_slop = std::max(_maxFrequency / 40, 1);

                    log_info("MK100 spindle initialized  done");

                    return true;
                };
            } else {
                return nullptr;
            }

        }

        VFDProtocol::response_parser MK100Protocol::get_current_speed(ModbusCommand& data) {
            data.tx_length = 6;
            data.rx_length = 5;

            // Send: 01 03 1007 0001
            data.msg[1] = 0x03;  // READ
            data.msg[2] = 0x10;  //
            data.msg[3] = 0x07;
            data.msg[4] = 0x00;  // Read 1 values
            data.msg[5] = 0x01;

            return [](const uint8_t* response, VFDSpindle* vfd, VFDProtocol* detail) -> bool {
            	uint32_t dev_speed = (uint16_t(response[3]) << 8) | uint16_t(response[4]);
                log_info("MK100 get speed: " << dev_speed);
                xQueueSend(VFD::VFDProtocol::vfd_speed_queue, &dev_speed, 0);
                return true;
            };
        }

        VFDProtocol::response_parser MK100Protocol::get_current_direction(ModbusCommand& data) {
            data.tx_length = 6;
            data.rx_length = 5;

            // Send: 01 03 30 00 00 01
            data.msg[1] = 0x03;  // READ
            data.msg[2] = 0x30;  // Command group ID
            data.msg[3] = 0x00;
            data.msg[4] = 0x00;  // Message ID
            data.msg[5] = 0x01;

            // Receive: 01 03 00 02 00 02
            //                      ----- status

            // TODO: What are we going to do with this? Update vfd state?
            return [](const uint8_t* response, VFDSpindle* vfd, VFDProtocol* detail) -> bool {
                uint16_t state = (uint16_t(response[3]) << 8) | uint16_t(response[4]);
                log_info("MK100 get direction: " << state);
                return true;
            };
        }

        VFDProtocol::response_parser MK100Protocol::get_status_ok(ModbusCommand& data) {
            data.tx_length = 6;
            data.rx_length = 5;

            data.msg[1] = 0x03;  // READ
            data.msg[2] = 0x80;  // Register address, high byte (current fault number)
            data.msg[3] = 0x00;  // Register address, low byte (current fault number)
            data.msg[4] = 0x00;  // Number of elements, high byte
            data.msg[5] = 0x01;  // Number of elements, low byte (1 element)

        /*
        Contents of register 0x0300
        Bit 0-15: current fault number, 0 = no fault, 1~18 = fault number
        */

            return [](const uint8_t* response, VFDSpindle* vfd, VFDProtocol* detail) -> bool {
                uint16_t currentFaultNumber = 0;

                if (response[1] != 0x03) {
                    return false;
                }

                // We expect a result length of 2 bytes
                if (response[2] != 2) {
                    return false;
                }

                currentFaultNumber = (uint16_t(response[3]) << 8) | uint16_t(response[4]);
                log_info("MK100 fault: " << currentFaultNumber);
                if (currentFaultNumber != 0) {
                    log_info("VFD: Got fault number: " << currentFaultNumber);
                    return false;
                }
                return true;
            };
        }

        // Configuration registration
        namespace {
            SpindleFactory::DependentInstanceBuilder<VFDSpindle, MK100Protocol> registration("MK100");
        }
    }
}

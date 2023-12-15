#include <iostream>
#include <libremidi/reader.hpp>
#include <fstream>
#include "Serial.hpp"
#include <termios.h>

int main() {
    auto devices = sms::Serial::getDevices();

    std::cout << devices.size() << std::endl;
    std:: cout << devices.at(0) << std::endl;

    unsigned char msg[] = {'H', 'e', 'l', 'l', 'o', '\n'};
    serial.write(msg, sizeof(msg));

    long num_received = 0;
    std::vector<unsigned char> buffer;
    while (serial.tryRead(buffer, sizeof(msg)) == -1);

    for (auto c : buffer) {
        std::cout << c;
    }

//    std::ifstream file{"sk-lava-reef.mid", std::ios::binary};
//
//    std::vector<uint8_t> bytes;
//    bytes.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
//
//    libremidi::reader r{true};
//
//    libremidi::reader::parse_result result = r.parse(bytes);
//
//    for (const auto& track : r.tracks) {
//        std:: cout << "New Track\n";
//        for (const libremidi::track_event& event : track)
//        {
//            std::cout << "Event at " << event.tick << " : ";
//            if (event.m.is_meta_event())
//            {
//                std::cout << "Meta event";
//            }
//            else
//            {
//                switch (event.m.get_message_type())
//                {
//                    case libremidi::message_type::NOTE_ON:
//                        std::cout << "Note ON: "
//                                  << "channel " << event.m.get_channel() << ' ' << "note "
//                                  << (int)event.m.bytes[1] << ' ' << "velocity " << (int)event.m.bytes[2]
//                                  << ' ';
//                        break;
//                    case libremidi::message_type::NOTE_OFF:
//                        std::cout << "Note OFF: "
//                                  << "channel " << event.m.get_channel() << ' ' << "note "
//                                  << (int)event.m.bytes[1] << ' ' << "velocity " << (int)event.m.bytes[2]
//                                  << ' ';
//                        break;
//                    case libremidi::message_type::CONTROL_CHANGE:
//                        std::cout << "Control: "
//                                  << "channel " << event.m.get_channel() << ' ' << "control "
//                                  << (int)event.m.bytes[1] << ' ' << "value " << (int)event.m.bytes[2]
//                                  << ' ';
//                        break;
//                    case libremidi::message_type::PROGRAM_CHANGE:
//                        std::cout << "Program: "
//                                  << "channel " << event.m.get_channel() << ' ' << "program "
//                                  << (int)event.m.bytes[1] << ' ';
//                        break;
//                    case libremidi::message_type::AFTERTOUCH:
//                        std::cout << "Aftertouch: "
//                                  << "channel " << event.m.get_channel() << ' ' << "value "
//                                  << (int)event.m.bytes[1] << ' ';
//                        break;
//                    case libremidi::message_type::POLY_PRESSURE:
//                        std::cout << "Poly pressure: "
//                                  << "channel " << event.m.get_channel() << ' ' << "note "
//                                  << (int)event.m.bytes[1] << ' ' << "value " << (int)event.m.bytes[2]
//                                  << ' ';
//                        break;
//                    case libremidi::message_type::PITCH_BEND:
//                        std::cout << "Poly pressure: "
//                                  << "channel " << event.m.get_channel() << ' ' << "bend "
//                                  << (int)((event.m.bytes[1] << 7) + event.m.bytes[2]) << ' ';
//                        break;
//                    default:
//                        std::cout << "Unsupported.";
//                        break;
//                }
//            }
//            std::cout << '\n';
//        }
//    }
    return 0;
}

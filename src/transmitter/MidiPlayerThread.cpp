//
// Created by dylangreen on 12/21/23.
//

#include <iostream>
#include "MidiPlayerThread.hpp"
#include "MidiMessage.h"

namespace sms {
    MidiPlayerThread::MidiPlayerThread(MessageQueue<ControlMessage>* messageQueue, libremidi::reader* midiReader,
                                       Serial* serial, std::mutex* readerMutex, std::mutex* serialMutex) {
        this->messageQueue = messageQueue;
        this->reader = midiReader;
        this->serial = serial;
        this->readerMutex = readerMutex;
        this->serialMutex = serialMutex;

        // Match the number of tracks to that of the MIDI reader
        trackReadIndices.resize(reader->tracks.size(), 0);

        // Calculate how long to wait between MIDI ticks
        // See http://midi.teragonaudio.com/tech/midifile/ppqn.htm
        auto tempo = reader->startingTempo;
        double microsPerBeat = 60000000.0 / tempo;
        double microsPerTick = microsPerBeat / reader->ticksPerBeat;
        tickDuration = std::chrono::duration<double, std::micro>(microsPerTick);

        endTick = static_cast<uint64_t>(reader->get_end_time());
    }

    // The thread's main work loop
    void MidiPlayerThread::operator()() {
        while (true) {
            if (state == PLAYING) {
                bool exit = handlePlaying();
                if (exit) return;
            } else { // Thread is paused
                bool exit = handlePaused();
                if (exit) return;
            }

        }
    }

    bool MidiPlayerThread::handlePlaying() {
        ControlMessage m;
        bool messageValid = messageQueue->tryPop(m);

        // If a message was received, handle it
        if (messageValid) {
            switch (m) {
                case PLAY: // Already playing
                    break;
                case PAUSE:
                    state = PAUSED;
                    break;
                case RESTART:
                    // TODO: Implement restart
                    currentTick = 0;
                    break;
                case EXIT:
                    return true; // Thread should stop execution
            }
        }

        // Process MIDI events at the current tick
        processTick();

        // Sleep until it's time to process the next tick
        auto wakeTime = std::chrono::steady_clock::now() + tickDuration;
        std::this_thread::sleep_until(wakeTime);

        return false;
    }

    bool MidiPlayerThread::handlePaused() {
        // Pause the thread until it receives a new control message
        ControlMessage m;
        messageQueue->waitPop(m);

        switch (m) {
            case PLAY:
                state = PLAYING;
                break;
            case PAUSE: // Already paused
                break;
            case RESTART:
                // TODO: Implement restart
                currentTick = 0;
                break;
            case EXIT:
                return true; // Thread should stop execution
        }

        return false;
    }

    // Processes any events in the MIDI file that occur at the current tick
    void MidiPlayerThread::processTick() {
        // If we've reached the end of the song, exit the playing state
        if (currentTick == endTick) {
            state = PAUSED;
            return;
        }

        // Each track will have its own list of MIDI events. We will process
        // each track's events up to the current tick number.
        int trackIdx = 0;
        for (const auto& track : reader->tracks) {
            auto currentIndex = trackReadIndices.at(trackIdx);
            // Iterate until we run out of events or reach a future tick's event
            while (currentIndex < track.size() &&
                   track.at(currentIndex).tick == currentTick) {
                processEvent(track.at(currentIndex));
                currentIndex++;
            }

            trackReadIndices.at(trackIdx) = currentIndex;
            trackIdx++;
        }

        currentTick++;
    }

    void MidiPlayerThread::processEvent(const libremidi::track_event &event) {
        if (event.m.is_meta_event()) {
            processMetaEvent(event);
            return;
        }

        switch (event.m.get_message_type()) {
            case libremidi::message_type::NOTE_ON:
            case libremidi::message_type::NOTE_OFF:
            case libremidi::message_type::CONTROL_CHANGE: {
                std::lock_guard<std::mutex> lock(*serialMutex);
                MidiMessage message;
                message.status = event.m.bytes[0];
                message.data[0] = event.m.bytes[1];
                message.data[1] = event.m.bytes[2];
                serial->write(&message, sizeof(message));
                break;
            }
            default:
                // Do nothing for PROGRAM_CHANGE, AFTERTOUCH, POLY_PRESSURE
                // and PITCH_BEND
                break;
        }
    }

    void MidiPlayerThread::processMetaEvent(const libremidi::track_event &event) {
        // We only want to handle tempo events right now
        if (event.m.bytes[1] == 0x51) {
            // This event always has 3 data bytes, starting at index 3
            auto bytes = event.m.bytes;
            uint32_t microsPerBeat = 0 | (bytes[3] << 16) | (bytes[4] << 8) | bytes[5];
            double microsPerTick = static_cast<double>(microsPerBeat) / reader->ticksPerBeat;
            tickDuration = std::chrono::duration<double, std::micro>(microsPerTick);
        }
    }
}
//
// Created by dylangreen on 12/21/23.
//

#ifndef STEPPER_MOTOR_SYMPHONY_V2_MIDIPLAYERTHREAD_HPP
#define STEPPER_MOTOR_SYMPHONY_V2_MIDIPLAYERTHREAD_HPP

#include <utility>
#include <libremidi/reader.hpp>
#include "MessageQueue.hpp"
#include "Serial.hpp"

namespace sms {
    class MidiPlayerThread {
    public:
        // The messages the MIDI player thread can process through its message queue
        enum ControlMessage {PLAY, PAUSE, RESTART, EXIT};

        MidiPlayerThread() = delete;
        MidiPlayerThread(MessageQueue<ControlMessage>* messageQueue, libremidi::reader* midiReader, Serial* serial,
                         std::mutex* readerMutex, std::mutex* serialMutex);

        void operator()();
    private:

        /**
         * Controls thread execution in the Playing state
         * @return A boolean value representing whether the thread should stop
         * execution. If true, the thread should exit so that the main thread
         * can call join() on it.
         */
        bool handlePlaying();

        /**
         * Controls thread execution in the Paused state
         * @return A boolean value representing whether the thread should stop
         * execution. If true, the thread should exit so that the main thread
         * can call join() on it.
         */
        bool handlePaused();

        /**
         * Processes any events in the MIDI file that occur at the current tick
         */
        void processTick();

        /**
         * A dispatch method for handling MIDI events supported by the program
         * @param event
         */
        void processEvent(const libremidi::track_event& event);


        // Member variables

        MessageQueue<ControlMessage>* messageQueue;
        libremidi::reader* reader;
        Serial* serial;
        mutable std::mutex* readerMutex;
        mutable std::mutex* serialMutex;

        enum CurrentState {PLAYING, PAUSED};
        CurrentState state = PAUSED;

        uint64_t currentTick = 0;
        std::chrono::duration<double, std::micro> tickDuration;
        uint64_t endTick = 0;

        std::vector<std::size_t> trackReadIndices;
    };

}

#endif //STEPPER_MOTOR_SYMPHONY_V2_MIDIPLAYERTHREAD_HPP

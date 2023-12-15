

#include <catch2/catch_test_macros.hpp>
#include <thread>
#include "MessageQueue.hpp"

TEST_CASE("Test constructor", "[MessageQueue]") {
    sms::MessageQueue<int> queue;

    REQUIRE(queue.size() == 0);
}

TEST_CASE("Test single threaded usage", "[MessageQueue]") {
    sms::MessageQueue<int> queue;
    queue.push(10);
    REQUIRE(queue.size() == 1);
    queue.push(20);
    REQUIRE(queue.size() == 2);

    int number = 0;
    REQUIRE(queue.tryPop(number));
    REQUIRE(number == 10);
    REQUIRE(queue.tryPop(number));
    REQUIRE(number == 20);
    REQUIRE_FALSE(queue.tryPop(number));
}

void waitPopThread(sms::MessageQueue<int>* queue, int& returnValue) {
    queue->waitPop(returnValue);
}

void tryPopThread(sms::MessageQueue<int>* queue, int& returnValue) {
    queue->tryPop(returnValue);
}

void pushThread(sms::MessageQueue<int>* queue) {
    queue->push(1000);
}

TEST_CASE("Test multi threaded usage", "[MessageQueue]") {
    sms::MessageQueue<int> q;
    q.push(123);

    int n1;
    std::thread waitPop = std::thread(waitPopThread, &q, std::ref(n1));
    waitPop.join();
    REQUIRE(n1 == 123);

    int n2 = 500;
    std::thread tryPop = std::thread(tryPopThread, &q, std::ref(n2));
    tryPop.join();
    REQUIRE(n2 == 500);

    std::thread push = std::thread(pushThread, &q);
    push.join();
    int n3;
    std::thread tryPop2 = std::thread(tryPopThread, &q, std::ref(n3));
    tryPop2.join();
    REQUIRE(n3 == 1000);
}
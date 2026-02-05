#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

std::queue<int> buffer;           // Shared buffer
std::mutex mtx;                  // Mutex for synchronizing access to the buffer
std::condition_variable cv;      // Condition variable for synchronization

const int MAX_SIZE = 5;          // Maximum size of the buffer

// Producer function
void producer() {
    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulate work
        std::unique_lock<std::mutex> lock(mtx);
        
        // Wait until there is space in the buffer
        // cv.wait(lock, []{ return buffer.size() < MAX_SIZE; }); // Predicate as lambda
        
        // Produce item (add to the buffer)
        buffer.push(i);
        std::cout << "Produced: " << i << std::endl;
        
        // Notify one consumer that there is an item available
        // cv.notify_one(); // Notify one consumer
    }
}

// Consumer function
void consumer(int id) {
    for (int i = 0; i < 5; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(150)); // Simulate work
        std::unique_lock<std::mutex> lock(mtx);

        // Wait until there is an item to consume
        // cv.wait(lock, []{ return !buffer.empty(); }); // Predicate as lambda
        
        // Consume item (remove from the buffer)
        if (!buffer.empty()) {
        int item = buffer.front();
            buffer.pop();
            std::cout << "Consumer " << id << " consumed: " << item << std::endl;
        }

        // Notify the producer that there is space available
        // cv.notify_one(); // Notify producer that there is space
    }
}

int main() {
    // Start producer thread
    std::thread prod(producer);
    
    // Start consumer threads
    std::vector<std::thread> consumers;
    for (int i = 1; i <= 3; ++i) {
        consumers.push_back(std::thread(consumer, i));
    }

    // Join threads to the main thread
    prod.join();
    for (auto& cons : consumers) {
        cons.join();
    }

    return 0;
}

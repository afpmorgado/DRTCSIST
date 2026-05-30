#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <vector>
#include <cstdint>

class CircularBuffer {
private:
    std::vector<int16_t> buffer;
    int head;
    int tail;
    int capacity;

public:
    explicit CircularBuffer(int capacity);
    void push_back(int value);
    bool pop_front(int& value);
    bool empty() const;
    bool full() const;
    int size() const;
    int getTail() const;
    int getHead() const;
    int getCapacity() const;
    float getValue(int index) const;
};

#endif
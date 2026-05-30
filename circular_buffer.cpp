#include "circular_buffer.h"
#include <stdexcept>

CircularBuffer::CircularBuffer(int capacity) {
    this->capacity = capacity;
    this->head = 0;
    this->tail = 0;
    buffer.resize(capacity);
}

void CircularBuffer::push_back(int value) {
    if (full()) {
        buffer[tail] = value;
        tail = (tail + 1) % capacity;
        head = (head + 1) % capacity;
    } else {
        buffer[tail] = value;
        tail = (tail + 1) % capacity;
        if (tail == head) {
            head = (head + 1) % capacity;
        }
    }
}

bool CircularBuffer::pop_front(int& value) {
    if (empty()) {
        return false;
    }
    value = buffer[head];
    head = (head + 1) % capacity;
    return true;
}

bool CircularBuffer::empty() const {
    return head == tail;
}

bool CircularBuffer::full() const {
    return (head + 1) % capacity == tail;
}

int CircularBuffer::size() const {
    if (head >= tail) return head - tail;
    return capacity - (tail - head);
}

int CircularBuffer::getTail() const {
    return tail;
}

int CircularBuffer::getHead() const {
    return head;
}

int CircularBuffer::getCapacity() const {
    return capacity;
}

float CircularBuffer::getValue(int index) const {
    return buffer[index];
}
#pragma once

#include "LinkedList.hpp"

namespace mtrn3100 {

template <typename T, typename Compare = util::less<T>>
class Queue {
private:
    LinkedList<T, Compare> list;

public:
    Queue() = default;

    void enqueue(const T& element) {
        list.push_back(element);
    }

    T dequeue() {
        T frontElement = list.front();
        list.pop_front();
        return frontElement;
    }

    const T& front() const {
        return list.front();
    }

    bool empty() const {
        return list.empty();
    }

    size_t size() const {
        return list.size();
    }

    void clear() {
        list.clear();
    }
};

}  // namespace mtrn3100

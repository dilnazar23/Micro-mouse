# Lab05: Path Planning

## Learning Outcome

- Understand time complexity and its significance.
- Understand the basic concepts of graph theory such as terminology, nodes, edges, and traversal.
- Understand the completeness and optimality of BFS.
- Can implement BFS and its variations.

---

## Prerequisites

### Related Lectures

- Planning I.

### Prelab Video

Mandatory:
- [Time complexity](https://youtu.be/__vX2sjlpXU)


- [Graph theory](https://unsw.sharepoint.com/:v:/s/CLS-MTRN3100_T2_5236_Combine/EY9voJXNUnhKv-fGW0nFhl4BlArVUSmOac8cOgkreFf1OQ?e=OSDhix)


- [Graph implementation](https://unsw.sharepoint.com/:v:/s/CLS-MTRN3100_T2_5236_Combine/EXS3hr8nfllMhjjOfgILJE0BKUzDDmaQ9y0WspSu9mMK0A?e=KWag37)


- [BFS](https://unsw.sharepoint.com/:v:/s/CLS-MTRN3100_T2_5236_Combine/ET7wRVHtqlNDpZmgjc83ajAB-g0C5cM1rROZantkDOsfZA?e=Iuhq0n)


### Kit

This lab will not require a kit.

---

## Prelab

1. (1 mark) State the time complexity for BFS.


1. (1 mark) Is the BFS complete?


1. (2 marks) Is the BFS optimal?


1. (2 marks) Demonstrate how to use the `LinkedList` class to simulate a queue.


1. (2 marks) Demonstrate how to use the `Graph` class to insert nodes and edges.


---

## Lab

1. (required) Ensure you have a C++ compiler installed.

    Use the following command when compiling:
    ```
    g++ -std=c++11 -Wall -Wextra -Werror -Wuninitialized test_something.cpp -o my_executable
    ```

1. (3 marks) Implement the function `bfs_simple` in `shortest_path.cpp` so it returns true if a shortest path has been found and false otherwise.

    You are not allowed to modify any of the function prototypes in `shortest_path.cpp`.

    Test the function against `test_bfs_simple.cpp`.


1. (3 marks) Implement the function `bfs_single` so that the shortest path is returned instead.

    Use `bfs_simple` as the starter for `bfs_single`.

    Test the function against `test_bfs_single.cpp`.


1. (4 marks) Implement the function `bfs_multiple` so that multiple shortest paths are returned instead.

    Use `bfs_single` as the starter for `bfs_multiple`.

    Test the function against `test_bfs_multiple.cpp`.


1. (required) Individually answer a demo question orally. The mark from this question will scale your lab mark.


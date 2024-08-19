This is a simple graph data structure implemented in C++ leveraging the standard template library's map, vector, queue, stack, and set. 
The library is generic, so it should work for most data types so long as a proper hash function is provided to the STL map.
I implemented common graph algorithms such as depth and breadth first traversals as well as Djikstra's and A* pathfinding algorithms.
In addition, I provided a way to test the comparative speed of Djikstra's and A*, although my heuristic function fo A* is only a placeholder currently, so they are about the same.
To download and run this library, do the following...
  => git clone https://github.com/dgunther2001/Graph
  => enter the project's root directory in your terminal
  => mkdir build
  => cd build
  => cmake ..
  => make
  => ./test
This will run the test file that I currently have built in, but feel free to write your own, or reconfigure the CMake files to run your own C++ files and test it yourself.
I plan to create a full suite of tests that I will run through a Git CI/CD pipeline (Git actions) and have a stable release API for general download and use.

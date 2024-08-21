This is a simple graph data structure implemented in C++ leveraging the standard template library's map, vector, queue, stack, and set. <br />
The entire library has been placed within a Ubuntu 22.04 Docker Container, and future full releases will be self-contained in a Docker Image. <br />
The library is generic, so it should work for most data types so long as a proper hash function is provided to the STL map. <br />
I implemented common graph algorithms such as depth and breadth first traversals as well as Djikstra's and A* pathfinding algorithms. <br />
In addition, I provided a way to test the comparative speed of Djikstra's and A*, although my heuristic function fo A* is only a placeholder currently, so they are about the same. <br />
To download and run this library, do the following... <br />
&emsp;  => git clone https://github.com/dgunther2001/Graph <br />
&emsp;  => cd Graph <br />
&emsp;  => docker build -t graph_api . <br />
&esmp;  => docker run --rm graph_api <br />
This will run the test file that I currently have built in, but feel free to write your own, or reconfigure the CMake files to run your own C++ files and test it yourself. <br />
I plan to create a full suite of tests that I will run through a Git CI/CD pipeline (Git actions) and have a stable release API for general download and use. <br />

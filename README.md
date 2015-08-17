# Laplace-Beltrami Operator for PCL

This is an implementation of the proposed method from [Liu, et al.](https://www.utdallas.edu/~xxg061000/pbmh.htm) 
to compute the Laplace-Beltrami Operator for point clouds. It is adapted to be 
used as part of the [Point Cloud Library](http://pointclouds.org/).

### Requirements

+ Point Cloud Library (>= 1.7)
+ Eigen library 3.0
+ CGAL 3.9

### Compile and run the demo

![Demo](https://raw.githubusercontent.com/alfonsoros88/PCLBO/master/doc/images/eigenfunctions.gif)

There is a small demo with the code that visualizes the Eigenfunctions of the 
Laplace-Beltrami Operator on the Stanford Bunny cloud. To compile and run it 
simply do the following:

```bash
    mkdir build
    cd build
    cmake ..
    make
    ./visualize_eigenfunctions
```

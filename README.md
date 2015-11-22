# Laplace-Beltrami Operator for PCL

This is an implementation of the proposed method from [Liu, et al.](https://www.utdallas.edu/~xxg061000/pbmh.htm) 
to compute the Laplace-Beltrami Operator for point clouds. It is adapted to be 
used as part of the [Point Cloud Library](http://pointclouds.org/).

### Requirements

+ Point Cloud Library (>= 1.7)
+ Eigen library 3.0
+ CGAL 3.9

### Demos

#### Computing the eigenfunctions of the Laplacian

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

#### Heat diffusion

There is also a small example for computing the heat kernel signature over 
time. The executable is called `heat`.

![Demo](https://raw.githubusercontent.com/alfonsoros88/PCLBO/master/doc/images/heat_diffusion.gif)

#### Compute the geodesic functions

This one is pretty cool. This is an implementation of the geodesics in heat 
method from ![Crane](http://www.cs.columbia.edu/~keenan/Projects/GeodesicsInHeat/index.html). This one
has a point picking event so you can click a point in the viewer and display 
the geodesics coming out from that point.

![Demo](https://raw.githubusercontent.com/alfonsoros88/PCLBO/master/doc/images/geodesics.gif)

```bash
    mkdir build
    cd build
    cmake ..
    make
    ./mesh_geoheat -m ../models/bunny.ply
```


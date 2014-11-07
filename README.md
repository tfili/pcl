AGI BUILD INSTRUCTIONS
======================
Enter the following commands to build Flann
```
    cd dependencies/flann-1.8.4
    cmake -G "Visual Studio 12 Win64"
```

Load dependencies/flann-1.8.4/flann.sln in Visual Studio 2013 and build debug and release.

Now that we build the dependency we can build PCL
```
    cd ../..
    cmake -DBOOST_ROOT=<path to boost> -G "Visual Studio 12 Win64"
```

Load PCL.sln in Visual Studio 2013 and build debug and release.

pcl
===

The Point Cloud Library (PCL) is a standalone, large scale, open project for 2D/3D image and point cloud processing.

PCL is released under the terms of the BSD license, and thus free for commercial and research use. We are financially supported by a consortium of commercial companies, with our own non-profit organization, Open Perception. We would also like to thank individual donors and contributors that have been helping the project.

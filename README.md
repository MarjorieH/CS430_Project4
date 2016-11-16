# CS430_Project4
### CS430: Computer Graphics
### Project 4: Ray Tracing
### Marjorie Hahn
### 15 November 2016

This application takes a JSON file containing object data for spheres and
planes within a scene and uses ray tracing to create a P3 PPM image that
corresponds to that object data. It also takes data for spot lights and point
lights contained within the scene and illuminates the objects in the scene
accordingly. Furthermore, it will create reflections and refractions corresponding
to the refractivity and reflectivity of the objects in the scene. An example of
an appropriate JSON file that this program can work on can be found in input.json.

Usage: raytrace width height input.json output.ppm

Where "width" and "height" set the size in pixels of the output.ppm image.

In order to run the program, after you have downloaded the files off of Github,
make sure that you are sitting in the directory that holds all of the files and
run the command "make all". Then you will be able to run the program using the
usage command mentioned above.

There is one JSON test file included that you can use to test the functionality
of the program: input.json. The expected output image for this JSON file is
output.ppm. You can create the corresponding PPM image for this JSON file using
the command "make test".

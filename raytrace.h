#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Hard coded Program Constants
#define maxColor 255
#define format '3' // format of output image data
#define maxObjects 128
#define epsilon 0.0000001 // tolerated error for comparing doubles
#define maxRecursionLevel 7 // number of recursive calls for ray tracing

#define ambientIntensity 1 // ambient lighting contribution (1 = 100%)
#define diffuseIntensity 1 // diffuse lighting contribution (1 = 100%)
#define specularIntensity 1 // specular lighting contribution (1 = 100%)

#define ambience 0 // ambient lighting color
#define specularPower 20 // degree of specular reflection, hard coded to 20
#define backgroundColor 0.1 // background of the scene

// Structure to hold RGB pixel data
typedef struct RGBpixel {
  unsigned char R, G, B;
} RGBpixel;

// Structure to hold an object's data in the scene
typedef struct {
  int kind; // 0 = plane, 1 = sphere, 2 = light, 3 = camera
  double color[3];
  double position[3];
  double diffuseColor[3];
  double specularColor[3];
  double reflectivity;
  double refractivity;
  double ior;
  union {
    struct {
      double normal[3];
    } plane;
    struct {
      double radius;
    } sphere;
    struct {
      double direction[3];
      double radialA2;
      double radialA1;
      double radialA0;
      double angularA0;
      double theta;
    } light;
    struct {
      double width;
      double height;
    } camera;
  };
} Object;

// Global variables to hold image data
RGBpixel* pixmap; // array of pixels to hold the image data
int numPixels; // total number of pixels in image (N * M)
int M; // height of image in pixels
int N; // width of image in pixels

// Global variables to hold general scene data
Object physicalObjects[maxObjects]; // Global array to keep track of objects in the scene
int numPhysicalObjects; // index to keep track of number of objects in the scene
Object lightObjects[maxObjects];
int numLightObjects;
Object cameraObject;

// Miscellaneous Globals
int line = 1; // keep track of the line number inside of the json file

// function prototype declarations
double* calculate_reflection(double* V, double* N);
double* calculate_refraction(double externalIOR, double transmitIOR, double* incomingRay, double* normal);
double diffuse_reflection(double lightColor, double diffuseColor, double diffuseFactor);
double* direct_shade(Object shadeObj, double* hitPoint, double* lightColor, double* lightDirection, double* lightPosition);
unsigned char double_to_color(double color);
void expect_c(FILE* json, int d);
double fang(double angularA0, double theta, double* lightToObj, double* lightDirection);
double frad(double lightDistance, double a0, double a1, double a2);
double* local_illumination(double* hitPoint, Object colorObj);
int next_c(FILE* json);
double next_number(FILE* json);
char* next_string(FILE* json);
void next_vector(FILE* json, double* v);
int obj_compare(Object a, Object b);
double plane_intersection(double* Ro, double* Rd, double* P, double* N);
void printObjs();
void printPixMap();
void raycast();
void read_scene(char* filename);
double* shade(Object objectHit, double* position, double* Ur, int level, double rindex);
void skip_ws(FILE* json);
double specular_reflection(double lightColor, double specularColor, double diffuseFactor, double specularFactor);
double sphere_intersection(double* Ro, double* Rd, double* C, double r);
void writeP3(FILE* fh);

// static inline functions
static inline int equal(double a, double b) { // returns 1 if values are equal, 0 if not
  return fabs(a - b) < epsilon;
}
static inline double sqr(double v) {
  return v*v;
}
static inline void normalize(double* v) {
  double len = sqrt(sqr(v[0]) + sqr(v[1]) + sqr(v[2]));
  if (!equal(len, 0.0)) {
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
  }
}
static inline void v3_scale(double* a, double s, double* c) {
  c[0] = s * a[0];
  c[1] = s * a[1];
  c[2] = s * a[2];
}
static inline void v3_add(double* a, double* b, double* c) {
  c[0] = a[0] + b[0];
  c[1] = a[1] + b[1];
  c[2] = a[2] + b[2];
}
static inline void v3_subtract(double* a, double* b, double* c) {
  c[0] = a[0] - b[0];
  c[1] = a[1] - b[1];
  c[2] = a[2] - b[2];
}
static inline double p3_distance(double* a, double* b) {
  return sqrt(sqr(b[0] - a[0]) + sqr(b[1] - a[1]) + sqr(b[2] - a[2]));
}
static inline double v3_dot(double* a, double* b) {
  return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}
static inline double rad_to_deg(double radians) {
    return radians * (180.0 / M_PI);
}

#include "raytrace.h"

// Writes P3 formatted data to a file
// Takes in the file handler of the file to be written to
void writeP3(FILE* fh) {
  fprintf(fh, "P%c\n%i %i\n%i\n", format, N, M, maxColor); // Write out header
  for (int i = 0; i < numPixels; i++) { // Write out Pixel data
    fprintf(fh, "%i %i %i\n", pixmap[i].R, pixmap[i].G, pixmap[i].B);
  }
  fclose(fh);
}

// Calculate if the ray Ro->Rd will intersect with a sphere of center C and radius R
// Return distance to intersection
double sphere_intersection(double* Ro, double* Rd, double* C, double r) {
  double a = sqr(Rd[0]) + sqr(Rd[1]) + sqr(Rd[2]);
  double b = 2 * (Rd[0] * (Ro[0] - C[0]) + Rd[1] * (Ro[1] - C[1]) + Rd[2] * (Ro[2] - C[2]));
  double c = sqr(Ro[0] - C[0]) + sqr(Ro[1] - C[1]) + sqr(Ro[2] - C[2]) - sqr(r);

  double det = sqr(b) - 4 * a * c;
  if (det < 0) return -1; // no intersection

  det = sqrt(det);

  double t0 = (-b - det) / (2 * a);
  if (t0 > 0) return t0;

  double t1 = (-b + det) / (2 * a);
  if (t1 > 0) return t1;

  return -1;
}

// Calculate if the ray Ro->Rd will intersect with a plane of position P and normal N
// Return distance to intersection
double plane_intersection(double* Ro, double* Rd, double* P, double* N) {
  double D = -(N[0] * P[0] + N[1] * P[1] + N[2] * P[2]); // distance from origin to plane
  double t = -(N[0] * Ro[0] + N[1] * Ro[1] + N[2] * Ro[2] + D) /
  (N[0] * Rd[0] + N[1] * Rd[1] + N[2] * Rd[2]);

  if (t > 0) return t;

  return -1; // no intersection
}

// helper function to convert a percentage double into a valid value for a color channel
unsigned char double_to_color(double color) {
  if (color > 1.0) {
    color = 1.0;
  }
  else if (color < 0) {
    color = 0.0;
  }
  return (unsigned char)(maxColor * color);
}

// Cast the objects in the scene
void raycast() {

  // default camera position
  double cx = cameraObject.position[0];
  double cy = cameraObject.position[1];
  double cz = cameraObject.position[2];

  double ch = cameraObject.camera.height;
  double cw = cameraObject.camera.width;

  double pixheight = ch / M;
  double pixwidth = cw / N;

  int pixIndex = 0; // position in pixmap array

  for (int y = 0; y < M; y++) { // for each row
    double y_coord = -(cy - (ch/2) + pixheight * (y + 0.5)); // y coord of the row

    for (int x = 0; x < N; x++) { // for each column

      double x_coord = cx - (cw/2) + pixwidth * (x + 0.5); // x coord of the column
      double Ro[3] = {cx, cy, cz}; // position of camera
      double Rd[3] = {x_coord, y_coord, 1}; // position of pixel
      normalize(Rd); // normalize (P - Ro)

      double closestT = INFINITY;
      Object closestObject;
      for (int i = 0; i < numPhysicalObjects; i++) { // loop through the array of objects in the scene
        double t = 0;
        if (physicalObjects[i].kind == 0) { // plane
          t = plane_intersection(Ro, Rd, physicalObjects[i].position, physicalObjects[i].plane.normal);
        }
        else if (physicalObjects[i].kind == 1) { // sphere
          t = sphere_intersection(Ro, Rd, physicalObjects[i].position, physicalObjects[i].sphere.radius);
        }
        else { // ???
          fprintf(stderr, "Unrecognized object.\n");
          exit(1);
        }
        if (t > 0 && t < closestT) { // found a closer t value, save the object data
          closestT = t;
          closestObject = physicalObjects[i];
        }
      }
      // place the pixel into the pixmap array, with illumination
      if (closestT > 0 && closestT != INFINITY) {
        illuminate(closestT, closestObject, Rd, Ro, pixIndex);
      }
      else { // make background pixels black
        pixmap[pixIndex].R = 0;
        pixmap[pixIndex].G = 0;
        pixmap[pixIndex].B = 0;
      }
      pixIndex++;
    }
  }
}

void illuminate(double colorObjT, Object colorObj, double* Rd, double* Ro, int pixIndex) {
  // initialize values for color, would be where ambient color goes
  double color[3];

  color[0] = ambientIntensity * ambience;
  color[1] = ambientIntensity * ambience;
  color[2] = ambientIntensity * ambience;

  int kind = colorObj.kind;

  double objOrigin[3]; // where the current object pixel is in space
  v3_scale(Rd, colorObjT, objOrigin);
  v3_add(objOrigin, Ro, objOrigin);

  double* objToCam = malloc(3 * sizeof(double)); // vector from the object to the camera
  v3_subtract(cameraObject.position, objOrigin, objToCam);
  normalize(objToCam);

  double* surfaceNormal = malloc(3 * sizeof(double));; // surface normal of the object
  if (kind == 0) { // plane
    surfaceNormal = colorObj.plane.normal;
  }
  else { // sphere
    v3_subtract(objOrigin, colorObj.position, surfaceNormal);
  }
  normalize(surfaceNormal); // TODO: This should really be moved elsewhere to save CPU...

  // loop through all the lights in the lights array
  for (int i = 0; i < numLightObjects; i++) {

    double lightToObj[3]; // ray from light towards the object
    v3_scale(Rd, colorObjT, lightToObj);
    v3_add(lightToObj, Ro, lightToObj);
    v3_subtract(lightToObj, lightObjects[i].position, lightToObj);
    normalize(lightToObj);

    double objToLight[3]; // ray from object towards the light
    v3_subtract(lightObjects[i].position, objOrigin, objToLight);
    normalize(objToLight);

    double* lightDirection = lightObjects[i].light.direction;
    normalize(lightDirection);

    // reflection of the ray of light hitting the surface, symmetrical across the normal
    double* reflection = malloc(3 * sizeof(double)); // R =  lightToObj - 2 * N * (N dot lightToObj)
    v3_scale(surfaceNormal, 2  * v3_dot(surfaceNormal, lightToObj), reflection);
    v3_subtract(lightToObj, reflection, reflection);
    normalize(reflection);

    double diffuseFactor = v3_dot(surfaceNormal, objToLight);
    double specularFactor = v3_dot(reflection, objToCam);

    double lightDistance = p3_distance(lightObjects[i].position, objOrigin); // distance from the light to the current pixel

    int shadow = 0;
    double currentT = 0.0;
    for (int j = 0; j < numPhysicalObjects; j++) { // loop through all the objects in the array
      Object currentObj = physicalObjects[j];

      if (obj_compare(currentObj, colorObj)) {
        continue; // skip over the object we are coloring
      }

      double* newObjOrigin = malloc(3 * sizeof(double));
      v3_scale(objToLight, 0.0000001, newObjOrigin);
      v3_add(newObjOrigin, objOrigin, newObjOrigin);

      if (currentObj.kind == 0) { // plane
        currentT = plane_intersection(newObjOrigin, objToLight, currentObj.position, currentObj.plane.normal);
      }
      else if (currentObj.kind == 1) { // sphere
        currentT = sphere_intersection(newObjOrigin, objToLight, currentObj.position, currentObj.sphere.radius);
      }
      else { // ???
        fprintf(stderr, "Unrecognized object.\n");
        exit(1);
      }

      if (currentT <= lightDistance && currentT > 0 && currentT < INFINITY) {
        shadow = 1;
        break;
      }
    }
    if (shadow == 0) { // no shadow

      double diffuse[3];
      diffuse[0] = diffuse_reflection(lightObjects[i].color[0], colorObj.diffuseColor[0], diffuseFactor);
      diffuse[1] = diffuse_reflection(lightObjects[i].color[1], colorObj.diffuseColor[1], diffuseFactor);
      diffuse[2] = diffuse_reflection(lightObjects[i].color[2], colorObj.diffuseColor[2], diffuseFactor);

      double specular[3];
      specular[0] = specular_reflection(lightObjects[i].color[0], colorObj.specularColor[0], diffuseFactor, specularFactor);
      specular[1] = specular_reflection(lightObjects[i].color[1], colorObj.specularColor[1], diffuseFactor, specularFactor);
      specular[2] = specular_reflection(lightObjects[i].color[2], colorObj.specularColor[2], diffuseFactor, specularFactor);

      double fRad = frad(lightDistance, lightObjects[i].light.radialA0, lightObjects[i].light.radialA1, lightObjects[i].light.radialA2);
      double fAng = fang(lightObjects[i].light.angularA0, lightObjects[i].light.theta, lightToObj, lightDirection);

      color[0] += fRad * fAng * (diffuse[0] + specular[0]);
      color[1] += fRad * fAng * (diffuse[1] + specular[1]);
      color[2] += fRad * fAng * (diffuse[2] + specular[2]);
    }
  }
  pixmap[pixIndex].R = double_to_color(color[0]);
  pixmap[pixIndex].G = double_to_color(color[1]);
  pixmap[pixIndex].B = double_to_color(color[2]);
}


// returns 1 if the physical objects are equal, 0 if not
int obj_compare(Object a, Object b) {
  if (a.kind == b.kind &&
    equal(a.diffuseColor[0], b.diffuseColor[0]) &&
    equal(a.diffuseColor[1], b.diffuseColor[1]) &&
    equal(a.diffuseColor[2], b.diffuseColor[2]) &&
    equal(a.specularColor[0], b.specularColor[0]) &&
    equal(a.specularColor[1], b.specularColor[1]) &&
    equal(a.specularColor[2], b.specularColor[2]) &&
    equal(a.position[0], b.position[0]) &&
    equal(a.position[1], b.position[1]) &&
    equal(a.position[2], b.position[2])) {
      if (a.kind == 0 &&
        equal(a.plane.normal[0], b.plane.normal[0]) &&
        equal(a.plane.normal[1], b.plane.normal[1]) &&
        equal(a.plane.normal[2], b.plane.normal[2])) {
          return 1; // same plane object
        }
        else if (a.kind == 1 &&
          equal(a.sphere.radius, b.sphere.radius)) {
      return 1; // same sphere object
    }
  }
  return 0;
}

// calculate diffuse reflection of the object
double diffuse_reflection(double lightColor, double diffuseColor, double diffuseFactor) {
  if (diffuseFactor > 0) {
    return diffuseIntensity * lightColor * diffuseColor * diffuseFactor;
  }
  else {
    return 0.0;
  }
}

// calculate specular reflection of the object
double specular_reflection(double lightColor, double specularColor, double diffuseFactor, double specularFactor) {
  if (specularFactor > 0 && diffuseFactor > 0) {
    return specularIntensity * lightColor * specularColor * pow(specularFactor, specularPower);
  }
  else {
    return 0.0;
  }
}

// helper function to calculate radial attinuation
double frad(double lightDistance, double a0, double a1, double a2) {
  if (lightDistance == INFINITY ||
      (equal(a0, 0.0) && equal(a1, 0.0) && equal(a2, 0.0))) {
    return 1.0;
  }
  else {
    return 1/(a2 * sqr(lightDistance) + (a1 * lightDistance) + a0);
  }
}

// helper function to calculate angular attinuation
double fang(double angularA0, double theta, double* lightToObj, double* lightDirection) {
  double alpha = rad_to_deg(acos(v3_dot(lightToObj, lightDirection)));
  if (equal(theta, 0.0)) { // not a spotlight
    return 1.0;
  }
  else if (theta < alpha) { // point isn't within spotlight
     return 0.0;
  }
  else {
    return pow(v3_dot(lightToObj, lightDirection), angularA0);
  }
}

// next_c() wraps the getc() function and provides error checking and line number maintenance
int next_c(FILE* json) {
  int c = fgetc(json);
  #ifdef DEBUG
  printf("next_c: '%c'\n", c);
  #endif
  if (c == '\n') {
    line += 1;
  }
  if (c == EOF) {
    fprintf(stderr, "Error: Unexpected end of file on line number %d.\n", line);
    exit(1);
  }
  return c;
}

// expect_c() checks that the next character is d.  If it is not it emits an error.
void expect_c(FILE* json, int d) {
  int c = next_c(json);
  if (c == d) return;
  fprintf(stderr, "Error: Expected '%c' on line %d.\n", d, line);
  exit(1);
}

// skip_ws() skips white space in the file.
void skip_ws(FILE* json) {
  int c = next_c(json);
  while (isspace(c)) {
    c = next_c(json);
  }
  ungetc(c, json);
}

// next_string() gets the next string from the file handle and emits an error
// if a string can not be obtained.
char* next_string(FILE* json) {
  char buffer[129];
  int c = next_c(json);
  if (c != '"') {
    fprintf(stderr, "Error: Expected string on line %d.\n", line);
    exit(1);
  }
  c = next_c(json);
  int i = 0;
  while (c != '"') {
    if (i >= 128) {
      fprintf(stderr, "Error: Strings longer than 128 characters in length are not supported.\n");
      exit(1);
    }
    if (c == '\\') {
      fprintf(stderr, "Error: Strings with escape codes are not supported.\n");
      exit(1);
    }
    if (c < 32 || c > 126) {
      fprintf(stderr, "Error: Strings may contain only ascii characters.\n");
      exit(1);
    }
    buffer[i] = c;
    i += 1;
    c = next_c(json);
  }
  buffer[i] = 0;
  return strdup(buffer);
}

// parse the next number in the json file
double next_number(FILE* json) {
  double value;
  fscanf(json, "%lf", &value);
  return value;
}

// parse the next vector in the json file (array of 3 doubles)
void next_vector(FILE* json, double* v) {
  expect_c(json, '[');
  skip_ws(json);
  v[0] = next_number(json);
  skip_ws(json);
  expect_c(json, ',');
  skip_ws(json);
  v[1] = next_number(json);
  skip_ws(json);
  expect_c(json, ',');
  skip_ws(json);
  v[2] = next_number(json);
  skip_ws(json);
  expect_c(json, ']');
}

// parse a json file based on filename and place any objects into the object array
void read_scene(char* filename) {

  int c;
  FILE* json = fopen(filename, "r");
  if (json == NULL) {
    fprintf(stderr, "Error: Could not open file \"%s\"\n", filename);
    exit(1);
  }
  int camFlag = 0; // boolean to see if we have a camera obj yet

  skip_ws(json);
  expect_c(json, '['); // Find the beginning of the list
  skip_ws(json);

  // Find the objects
  while (1) {

    c = fgetc(json);
    if (c == ']') {
      fprintf(stderr, "Error: Empty object at line %d.\n", line);
      fclose(json);
      return;
    }
    if (c == '{') {
      skip_ws(json);
      // Parse the object
      char* key = next_string(json);
      if (strcmp(key, "type") != 0) {
        fprintf(stderr, "Error: Expected \"type\" key on line number %d.\n", line);
        exit(1);
      }
      skip_ws(json);
      expect_c(json, ':');
      skip_ws(json);
      char* value = next_string(json);

      int kind;
      if (strcmp(value, "plane") == 0) {
        physicalObjects[numPhysicalObjects].kind = 0;
        kind = 0;
      }
      else if (strcmp(value, "sphere") == 0) {
        physicalObjects[numPhysicalObjects].kind = 1;
        kind = 1;
      }
      else if (strcmp(value, "light") == 0) {
        lightObjects[numLightObjects].kind = 2;
        kind = 2;
      }
      else if (strcmp(value, "camera") == 0) {
        if (camFlag == 1) {
          fprintf(stderr, "Error: Too many camera objects, see line: %d.\n", line);
          exit(1);
        }
        cameraObject.kind = 3;
        cameraObject.position[0] = 0;
        cameraObject.position[1] = 0;
        cameraObject.position[2] = 0;
        camFlag = 1;
        kind = 3;
      }
      else {
        fprintf(stderr, "Error: Unknown type, \"%s\", on line number %d.\n", value, line);
        exit(1);
      }
      skip_ws(json);

      while (1) { // parse the current object

        c = next_c(json);
        if (c == '}') {
          break; // stop parsing this object
        }
        else if (c == ',') {
          // read another field
          skip_ws(json);
          char* key = next_string(json);
          skip_ws(json);
          expect_c(json, ':');
          skip_ws(json);
          if (strcmp(key, "width") == 0) {
            double value = next_number(json);
            if (kind == 3) {
              cameraObject.camera.width = value;
            }
            else {
              fprintf(stderr, "Error: Unexpected 'width' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "height") == 0) {
            double value = next_number(json);
            if (kind == 3) {
              cameraObject.camera.height = value;
            }
            else {
              fprintf(stderr, "Error: Unexpected 'height' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "radius") == 0) {
            double value = next_number(json);
            if (kind == 1) {
              physicalObjects[numPhysicalObjects].sphere.radius = value;
            }
            else {
              fprintf(stderr, "Error: Unexpected 'radius' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "color") == 0) {
            double value[3];
            next_vector(json, value);
            if (kind == 0 || kind == 1) {
              memcpy(physicalObjects[numPhysicalObjects].color, value, sizeof(double) * 3);
            }
            else if (kind == 2) {
              memcpy(lightObjects[numLightObjects].color, value, sizeof(double) * 3);
            }
            else {
              fprintf(stderr, "Error: Unexpected 'color' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "diffuse_color") == 0) {
            double value[3];
            next_vector(json, value);
            if (kind == 0 || kind == 1) {
              memcpy(physicalObjects[numPhysicalObjects].diffuseColor, value, sizeof(double) * 3);
            }
            else {
              fprintf(stderr, "Error: Unexpected 'diffuse_color' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "specular_color") == 0) {
            double value[3];
            next_vector(json, value);
            if (kind == 0 || kind == 1) {
              memcpy(physicalObjects[numPhysicalObjects].specularColor, value, sizeof(double) * 3);
            }
            else {
              fprintf(stderr, "Error: Unexpected 'specular_color' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "position") == 0) {
            double value[3];
            next_vector(json, value);
            if (kind == 0 || kind == 1) {
              memcpy(physicalObjects[numPhysicalObjects].position, value, sizeof(double) * 3);
            }
            else if (kind == 2) {
              memcpy(lightObjects[numLightObjects].position, value, sizeof(double) * 3);
            }
            else {
              fprintf(stderr, "Error: Unexpected 'position' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "normal") == 0) {
            double value[3];
            next_vector(json, value);
            if (kind == 0) {
              memcpy(physicalObjects[numPhysicalObjects].plane.normal, value, sizeof(double) * 3);
            }
            else {
              fprintf(stderr, "Error: Unexpected 'normal' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "direction") == 0) {
            double value[3];
            next_vector(json, value);
            if (kind == 2) {
              memcpy(lightObjects[numLightObjects].light.direction, value, sizeof(double) * 3);
            }
            else {
              fprintf(stderr, "Error: Unexpected 'direction' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "radial-a0") == 0) {
            double value = next_number(json);
            if (kind == 2) {
              lightObjects[numLightObjects].light.radialA0 = value;
            }
            else {
              fprintf(stderr, "Error: Unexpected 'radial-a0' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "radial-a1") == 0) {
            double value = next_number(json);
            if (kind == 2) {
              lightObjects[numLightObjects].light.radialA1 = value;
            }
            else {
              fprintf(stderr, "Error: Unexpected 'radial-a1' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "radial-a2") == 0) {
            double value = next_number(json);
            if (kind == 2) {
              lightObjects[numLightObjects].light.radialA2 = value;
            }
            else {
              fprintf(stderr, "Error: Unexpected 'radial-a2' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "angular-a0") == 0) {
            double value = next_number(json);
            if (kind == 2) {
              lightObjects[numLightObjects].light.angularA0 = value;
            }
            else {
              fprintf(stderr, "Error: Unexpected 'angular-a0' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else if (strcmp(key, "theta") == 0) {
            double value = next_number(json);
            if (kind == 2) {
              lightObjects[numLightObjects].light.theta = value;
            }
            else {
              fprintf(stderr, "Error: Unexpected 'theta' attribute on line %d.\n", line);
              exit(1);
            }
          }
          else {
            fprintf(stderr, "Error: Unknown property, \"%s\", on line %d.\n", key, line);
            exit(1);
          }
          skip_ws(json);
        }
        else {
          fprintf(stderr, "Error: Unexpected value on line %d\n", line);
          exit(1);
        }
      } // end loop through object fields

      if (camFlag == 0) { // ensure that we parsed a camera
        fprintf(stderr, "Error: The JSON file does not contain a camera object.\n");
        exit(1);
      }

      // increment appropriate counter
      if (kind == 0 || kind == 1) {
        numPhysicalObjects++;
      }
      else if (kind == 2) {
        numLightObjects++;
      }

      skip_ws(json);
      c = next_c(json);
      if (c == ',') {
        skip_ws(json);
      }
      else if (c == ']') {
        fclose(json);
        return;
      }
      else {
        fprintf(stderr, "Error: Expecting ',' or ']' on line %d.\n", line);
        exit(1);
      }
    }
  } // end loop through all objects in scene
}

// function to print out all the objects to stdout, for debugging
void printObjs() {
  for (int i = 0; i < numPhysicalObjects; i++) {
    printf("Object %i: type = %i; position = [%lf, %lf, %lf]; color = [%lf, %lf, %lf]\n", i, physicalObjects[i].kind,
    physicalObjects[i].position[0],
    physicalObjects[i].position[1],
    physicalObjects[i].position[2],
    physicalObjects[i].color[0],
    physicalObjects[i].color[1],
    physicalObjects[i].color[2]);
    if (physicalObjects[i].kind == 1) {
      printf("  Radius = %lf\n", physicalObjects[i].sphere.radius);
    }
    else if (physicalObjects[i].kind == 0) {
      printf("  Normal = [%lf, %lf, %lf]\n", physicalObjects[i].plane.normal[0], physicalObjects[i].plane.normal[1], physicalObjects[i].plane.normal[2]);
    }
  }
  for (int i = 0; i < numLightObjects; i++) {
    printf("Light Object %i: type = %i; position = [%lf, %lf, %lf]\n", i, lightObjects[i].kind,
    lightObjects[i].position[0],
    lightObjects[i].position[1],
    lightObjects[i].position[2]);
    printf("   A0: %lf; A1: %lf A2: %lf\n",
    lightObjects[i].light.radialA0,
    lightObjects[i].light.radialA1,
    lightObjects[i].light.radialA2);
  }
  printf("Camera: type = %i\n", cameraObject.kind);
}

// function to print out the contents of pixmap to stdout, for debugging
void printPixMap() {
  int i = 0;
  for (int y = 0; y < M; y++) {
    for (int x = 0; x < N; x++) {
      printf("[%i, %i, %i] ", pixmap[i].R, pixmap[i].G, pixmap[i].B); // print the pixel
      i++;
    }
    printf("\n");
  }
}

int main(int args, char** argv) {
  if (args != 5) {
    fprintf(stderr, "Usage: raycast width height input.json output.ppm\n");
    exit(1);
  }

  M = atoi(argv[2]); // save height
  N = atoi(argv[1]); // save width
  numPixels = M * N; // total pixels for output image

  // initialize pixmap based on the number of pixels
  pixmap = malloc(sizeof(RGBpixel) * numPixels);

  // initialize counters
  numPhysicalObjects = 0;
  numLightObjects = 0;

  //physicalObjects = malloc(maxObjects * sizeof(Object));
  //lightObjects = malloc(maxObjects * sizeof(Object));

  read_scene(argv[3]);
  raycast();

  // finished creating image data, write out
  FILE* fh = fopen(argv[4], "w");
  writeP3(fh);

  free(pixmap);
  return 0; // exit success
}

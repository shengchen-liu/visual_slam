/**
 * Single camera calibration
 *
 **/

#include <vector>
#include <cmath>

#include "SingleCamera.h"
#include "Points.h"

int main() {

  auto calPoints{new CalibrationPoints};
  float signRho{1.0};
  float signR3{1.0};
  bool verbose = true;
  auto singleCamera = new SingleCamera(calPoints->worldCoord, calPoints->pixelCoord, calPoints->numPoints, signRho, signR3, verbose);

  singleCamera->composeMatP();
  singleCamera->performSVDMatP();
  singleCamera->solveIntrinsicAndExtrinsic();

  // test 5 points and verify M
  singleCamera->selfCheck(calPoints->worldCoordTest, calPoints->cameraCoordTest);

  return EXIT_SUCCESS;
}
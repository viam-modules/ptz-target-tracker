package models

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
)

func TestCalculateCameraPosAndPanPlaneIdealCameraWithFullRange(t *testing.T) {
	// Create a minimal tracker struct for testing
	/*
		With this configuration, the camera is perfectly aligned with the XY plane, this means the tilt angle offset is 0.0.
		In turn, this means the the pan motion will pefectly sweep along the XY plane.

		Also, this camera's range is [0, 90] for the tilt angle and [0, 360] for the pan angle, (normally cameras have a reduced pan/tilt range, but this is a test camera
		with a full range).
	*/
	tracker := &componentTracker{
		logger:                             logging.NewTestLogger(t),
		panMinDeg:                          0.0,
		panMaxDeg:                          360.0,
		tiltMinDeg:                         0.0,
		tiltMaxDeg:                         90.0,
		absoluteCalibrationRayMeasurements: make(map[string]AbsoluteCalibrationRayMeasurements),
	}

	targetPos1 := r3.Vector{X: 0, Y: 100, Z: 0} // Target directly along +Y axis
	pan1 := 0.5
	tilt1 := -1.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos1, pan1, tilt1, "ray1") // Target 1

	targetPos2 := r3.Vector{X: 0, Y: 200, Z: 0} // Target directly along +Y axis
	pan2 := 0.5
	tilt2 := -1.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos2, pan2, tilt2, "ray1") // Target 2

	// Now we move the target to the second position and record the pan and tilt angles
	targetPos3 := r3.Vector{X: 100, Y: 0, Z: 0} // Target directly along +X axis
	pan3 := 0.0
	tilt3 := -1.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos3, pan3, tilt3, "ray2") // Target 3

	targetPos4 := r3.Vector{X: 200, Y: 0, Z: 0} // Target directly along +X axis
	pan4 := 0.0
	tilt4 := -1.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos4, pan4, tilt4, "ray2") // Target 4

	// Get the rays from the measurements
	measuremntsRay1, _ := tracker.getAbsoluteCalibrationMeasurements("ray1")
	if len(measuremntsRay1.Measurements) != 2 {
		t.Errorf("Expected 2 target positions for ray1, got %d", len(measuremntsRay1.Measurements))
	}
	ray1, _ := calculateRayFromMeasurements(measuremntsRay1)
	t.Logf("Ray1: %+v", ray1)
	// The origin of the ray is the centroid of the target positions
	expectedRay1Origin := r3.Vector{X: calculateAverage([]float64{targetPos1.X, targetPos2.X}), Y: calculateAverage([]float64{targetPos1.Y, targetPos2.Y}), Z: calculateAverage([]float64{targetPos1.Z, targetPos2.Z})}
	if expectedRay1Origin.Distance(ray1.Ray.Origin) > 0.001 {
		t.Errorf("Expected ray1 origin to be %+v, got %+v", expectedRay1Origin, ray1.Ray.Origin)
	}
	// The direction of the ray is the unit vector of the target positions
	expectedRay1Direction := r3.Vector{X: 0.0, Y: 1.0, Z: 0.0}
	if ray1.Ray.Direction.X != expectedRay1Direction.X || ray1.Ray.Direction.Y != expectedRay1Direction.Y || ray1.Ray.Direction.Z != expectedRay1Direction.Z {
		t.Errorf("Expected ray1 direction to be %+v, got %+v", expectedRay1Direction, ray1.Ray.Direction)
	}

	measuremntsRay2, _ := tracker.getAbsoluteCalibrationMeasurements("ray2")
	ray2, _ := calculateRayFromMeasurements(measuremntsRay2)
	t.Logf("Ray2: %+v", ray2)
	expectedRay2Origin := r3.Vector{X: calculateAverage([]float64{targetPos3.X, targetPos4.X}), Y: calculateAverage([]float64{targetPos3.Y, targetPos4.Y}), Z: calculateAverage([]float64{targetPos3.Z, targetPos4.Z})}
	if ray2.Ray.Origin.X != expectedRay2Origin.X || ray2.Ray.Origin.Y != expectedRay2Origin.Y || ray2.Ray.Origin.Z != expectedRay2Origin.Z {
		t.Errorf("Expected ray2 origin to be %+v, got %+v", expectedRay2Origin, ray2.Ray.Origin)
	}
	expectedRay2Direction := r3.Vector{X: 1.0, Y: 0.0, Z: 0.0}
	if ray2.Ray.Direction.X != expectedRay2Direction.X || ray2.Ray.Direction.Y != expectedRay2Direction.Y || ray2.Ray.Direction.Z != expectedRay2Direction.Z {
		t.Errorf("Expected ray2 direction to be %+v, got %+v", expectedRay2Direction, ray2.Ray.Direction)
	}

	// Calculate the camera's position and pan plane from the measurements, a pan value of 0 should be aligned with the +X axis of the panPlane
	cameraPos, panPlane, pan0Direction, zeroPanTiltDirection := tracker.calculateCameraPositionAndPanPlane(ray1, ray2)
	tracker.absoluteCalibrationPan0Reference = pan0Direction // Store for predictPanTiltAbsolute
	t.Logf("Camera position: %+v", cameraPos)
	t.Logf("Pan plane: %+v", panPlane)
	t.Logf("Pan0 reference: %+v", pan0Direction)
	if cameraPos.X != 0.0 || cameraPos.Y != 0.0 || cameraPos.Z != 0.0 {
		t.Errorf("Expected camera position to be (0, 0, 0), got %.3f, %.3f, %.3f", cameraPos.X, cameraPos.Y, cameraPos.Z)
	}
	if panPlane.X != 0.0 || panPlane.Y != 0.0 || panPlane.Z != 1.0 {
		t.Errorf("Expected pan plane to be (0, 0, 1), got %.3f, %.3f, %.3f", panPlane.X, panPlane.Y, panPlane.Z)
	}
	if pan0Direction.X != 1.0 || pan0Direction.Y != 0.0 || pan0Direction.Z != 0.0 {
		t.Errorf("Expected pan0 reference to be (1, 0, 0), got %+v", pan0Direction)
	}
	// The zero pan tilt direction is the vector that points from the camera position to the point where the camera is pointing at with pan=0 and tilt=0
	expectedZeroPanTiltDirection := r3.Vector{X: 0.7071067811865475, Y: 0.0, Z: 0.7071067811865475}
	if expectedZeroPanTiltDirection.Distance(zeroPanTiltDirection) > 0.001 {
		t.Errorf("Expected zero pan tilt direction to be %+v, got %+v", expectedZeroPanTiltDirection, zeroPanTiltDirection)
	}
	// Pan value of 0 is aligned with the +X axis of the panPlane
	// Tilt value of -1 means the camera is perfectly aligned with the XY plane, so the predicted pan and tilt should be 0 and -1
	targetPos_P0_T0 := r3.Vector{X: 100, Y: 0, Z: 0}
	P0_Predicted, T0_Predicted := tracker.predictPanTiltAbsolute(targetPos_P0_T0, cameraPos, panPlane, pan0Direction)
	Pan_expected := 0.0
	Tilt_expected := -1.0
	if math.Abs(P0_Predicted-Pan_expected) > 0.001 {
		t.Errorf("Expected pan predicted to be %.3f, got %.3f", Pan_expected, P0_Predicted)
	}
	if math.Abs(T0_Predicted-Tilt_expected) > 0.001 {
		t.Errorf("Expected tilt predicted to be %.3f, got %.3f", Tilt_expected, T0_Predicted)
	}

	// Pan value of 90 degrees is aligned with the +Y axis of the panPlane
	targetPos_P90_T0 := r3.Vector{X: 0, Y: 100, Z: 0}
	P90_Predicted, T0_Predicted := tracker.predictPanTiltAbsolute(targetPos_P90_T0, cameraPos, panPlane, pan0Direction)
	Pan_expected = 0.5
	Tilt_expected = -1.0
	if math.Abs(P90_Predicted-Pan_expected) > 0.001 {
		t.Errorf("Expected pan predicted to be %.3f, got %.3f", Pan_expected, P90_Predicted)
	}
	if math.Abs(T0_Predicted-Tilt_expected) > 0.001 {
		t.Errorf("Expected tilt predicted to be %.3f, got %.3f", Tilt_expected, T0_Predicted)
	}

	// Target at 45 degrees from the xy plane and 90 degrees from the x axis, its tilt should be 0.5 and its pan should be 0.5
	targetPos_P90_T45 := r3.Vector{X: 0, Y: 100, Z: 100}
	P90_Predicted, T45_Predicted := tracker.predictPanTiltAbsolute(targetPos_P90_T45, cameraPos, panPlane, pan0Direction)
	if math.Abs(P90_Predicted-0.5) > 0.001 {
		t.Errorf("Expected pan predicted to be 0.5, got %.3f", P90_Predicted)
	}
	if math.Abs(T45_Predicted-0) > 0.001 {
		t.Errorf("Expected tilt predicted to be 0, got %.3f", T45_Predicted)
	}

	// Target at 60 degrees from the xy plane, its tilt should be 0.3333333333333333 and its pan should be 0
	targetPos_P0_T60 := r3.Vector{X: 100, Y: 0, Z: 173.20508075688772}
	P0_Predicted, T60_Predicted := tracker.predictPanTiltAbsolute(targetPos_P0_T60, cameraPos, panPlane, pan0Direction)
	if math.Abs(P0_Predicted-0.0) > 0.001 {
		t.Errorf("Expected pan predicted to be 0.0, got %.3f", P0_Predicted)
	}
	if math.Abs(T60_Predicted-0.3333333333333333) > 0.001 {
		t.Errorf("Expected tilt predicted to be 0.3333333333333333, got %.3f", T60_Predicted)
	}

	// Target at 45 degrees from the xy plane, its tilt should be 0 and its pan should be 0
	targetPos_PMinus90_T45 := r3.Vector{X: 0, Y: -100, Z: 100}
	PMinus90_Predicted, T45_Predicted := tracker.predictPanTiltAbsolute(targetPos_PMinus90_T45, cameraPos, panPlane, pan0Direction)
	if math.Abs(PMinus90_Predicted-(-0.5)) > 0.001 {
		t.Errorf("Expected pan predicted to be -0.5, got %.3f", PMinus90_Predicted)
	}
	if math.Abs(T45_Predicted-0) > 0.001 {
		t.Errorf("Expected tilt predicted to be 0.5, got %.3f", T45_Predicted)
	}
}
func TestCalculateCameraPosAndPanPlaneReducedPanTiltRange(t *testing.T) {
	// This camera has a reduced pan/tilt range, so the pan/tilt values will be normalized to the range [-1, 1] for the pan and tilt values.
	tracker := &componentTracker{
		logger:                             logging.NewTestLogger(t),
		panMinDeg:                          0.0,
		panMaxDeg:                          355.0,
		tiltMinDeg:                         5.0,
		tiltMaxDeg:                         90.0,
		absoluteCalibrationRayMeasurements: make(map[string]AbsoluteCalibrationRayMeasurements),
	}

	targetPos1 := r3.Vector{X: 0, Y: 100, Z: 8.748866352590167} // Target directly along +Y axis, with a tilt of 5 degrees
	pan1 := 0.5
	tilt1 := 0.0                                                               // In this case, given the reduced range, 5 degrees of tilt corresponds to a pan value of 0
	tracker.addAbsoluteCalibrationMeasurement(targetPos1, pan1, tilt1, "ray1") // Target 1

	targetPos2 := r3.Vector{X: 0, Y: 200, Z: 17.4977327052} // Target directly along +Y axis, with a tilt of 5 degrees
	pan2 := 0.5
	tilt2 := 0.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos2, pan2, tilt2, "ray1") // Target 2

	// Now we move the target to the second position and record the pan and tilt angles
	targetPos3 := r3.Vector{X: 100, Y: 0, Z: 8.748866352590167} // Target directly along +X axis, with a tilt of 5 degrees
	pan3 := 0.0
	tilt3 := 0.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos3, pan3, tilt3, "ray2") // Target 3

	targetPos4 := r3.Vector{X: 200, Y: 0, Z: 17.4977327052} // Target directly along +X axis, with a tilt of 5 degrees
	pan4 := 0.0
	tilt4 := 0.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos4, pan4, tilt4, "ray2") // Target 4

	// Get the rays from the measurements
	measuremntsRay1, _ := tracker.getAbsoluteCalibrationMeasurements("ray1")
	ray1, _ := calculateRayFromMeasurements(measuremntsRay1)
	t.Logf("Ray1: %+v", ray1)
	// The origin of the ray is the centroid of the target positions
	expectedRay1Origin := r3.Vector{X: calculateAverage([]float64{targetPos1.X, targetPos2.X}), Y: calculateAverage([]float64{targetPos1.Y, targetPos2.Y}), Z: calculateAverage([]float64{targetPos1.Z, targetPos2.Z})}
	if ray1.Ray.Origin.X != expectedRay1Origin.X || ray1.Ray.Origin.Y != expectedRay1Origin.Y || ray1.Ray.Origin.Z != expectedRay1Origin.Z {
		t.Errorf("Expected ray1 origin to be %+v, got %+v", expectedRay1Origin, ray1.Ray.Origin)
	}
	// The direction of the ray is the unit vector of the target positions
	// Note: Since targets have Z values (5 degree tilt), the direction includes a Z component
	expectedRay1Direction := r3.Vector{X: 0.0, Y: 0.996194698091730557187873, Z: 0.087155742747830514116636}
	if math.Abs(ray1.Ray.Direction.X-expectedRay1Direction.X) > 0.001 || math.Abs(ray1.Ray.Direction.Y-expectedRay1Direction.Y) > 0.001 || math.Abs(ray1.Ray.Direction.Z-expectedRay1Direction.Z) > 0.001 {
		t.Errorf("Expected ray1 direction to be %+v, got %+v", expectedRay1Direction, ray1.Ray.Direction)
	}
	measuremntsRay2, _ := tracker.getAbsoluteCalibrationMeasurements("ray2")
	ray2, _ := calculateRayFromMeasurements(measuremntsRay2)
	t.Logf("Ray2: %+v", ray2)
	expectedRay2Origin := r3.Vector{X: calculateAverage([]float64{targetPos3.X, targetPos4.X}), Y: calculateAverage([]float64{targetPos3.Y, targetPos4.Y}), Z: calculateAverage([]float64{targetPos3.Z, targetPos4.Z})}
	if ray2.Ray.Origin.X != expectedRay2Origin.X || ray2.Ray.Origin.Y != expectedRay2Origin.Y || ray2.Ray.Origin.Z != expectedRay2Origin.Z {
		t.Errorf("Expected ray2 origin to be %+v, got %+v", expectedRay2Origin, ray2.Ray.Origin)
	}
	expectedRay2Direction := r3.Vector{X: 0.996194698091730557187873, Y: 0.0, Z: 0.087155742747830514116636}
	if math.Abs(ray2.Ray.Direction.X-expectedRay2Direction.X) > 0.001 || math.Abs(ray2.Ray.Direction.Y-expectedRay2Direction.Y) > 0.001 || math.Abs(ray2.Ray.Direction.Z-expectedRay2Direction.Z) > 0.001 {
		t.Errorf("Expected ray2 direction to be %+v, got %+v", expectedRay2Direction, ray2.Ray.Direction)
	}

	// Calculate the camera's position and pan plane from the measurements, a pan value of 0 should be aligned with the +X axis of the panPlane
	cameraPos, panPlane, pan0Direction, zeroPanTiltDirection := tracker.calculateCameraPositionAndPanPlane(ray1, ray2)
	tracker.absoluteCalibrationPan0Reference = pan0Direction // Store for predictPanTiltAbsolute
	t.Logf("Camera position: %+v", cameraPos)
	t.Logf("Pan plane: %+v", panPlane)
	t.Logf("Pan0 reference: %+v", pan0Direction)
	// Camera position should be near origin (allowing for small numerical errors)
	if math.Abs(cameraPos.X) > 0.01 || math.Abs(cameraPos.Y) > 0.01 || math.Abs(cameraPos.Z) > 0.01 {
		t.Errorf("Expected camera position to be near (0, 0, 0), got %.3f, %.3f, %.3f", cameraPos.X, cameraPos.Y, cameraPos.Z)
	}
	// Pan plane is always horizontal (0, 0, 1) regardless of measurement angles
	expectedPanPlane := r3.Vector{X: 0.0, Y: 0.0, Z: 1.0}
	if math.Abs(panPlane.X-expectedPanPlane.X) > 0.001 || math.Abs(panPlane.Y-expectedPanPlane.Y) > 0.001 || math.Abs(panPlane.Z-expectedPanPlane.Z) > 0.001 {
		t.Errorf("Expected pan plane to be approximately %+v, got %.3f, %.3f, %.3f", expectedPanPlane, panPlane.X, panPlane.Y, panPlane.Z)
	}
	// Pan0 direction is the vector direction of the pan=0 reference, in the panPlane coordinates
	expectedPan0Direction := r3.Vector{X: 1.0, Y: 0.0, Z: 0.0}
	if math.Abs(pan0Direction.X-expectedPan0Direction.X) > 0.001 || math.Abs(pan0Direction.Y-expectedPan0Direction.Y) > 0.001 || math.Abs(pan0Direction.Z-expectedPan0Direction.Z) > 0.001 {
		t.Errorf("Expected pan0 reference to be approximately %+v, got %+v", expectedPan0Direction, pan0Direction)
	}

	expectedZeroPanTiltDirection := r3.Vector{X: 0.675590207615660354178999, Y: 0.0, Z: 0.737277336810124084287565}
	if expectedZeroPanTiltDirection.Distance(zeroPanTiltDirection) > 0.001 {
		t.Errorf("Expected zero pan tilt direction to be %+v, got %+v", expectedZeroPanTiltDirection, zeroPanTiltDirection)
	}

	// Pan value of 0 is aligned with the +X axis of the panPlane
	targetPos_P0_TMinus1 := r3.Vector{X: 100, Y: 0, Z: 8.748866352590167}
	P0_Predicted, TMinus1_Predicted := tracker.predictPanTiltAbsolute(targetPos_P0_TMinus1, cameraPos, panPlane, pan0Direction)
	Pan_expected := 0.0
	Tilt_expected := -1.0
	if math.Abs(P0_Predicted-Pan_expected) > 0.001 {
		t.Errorf("Expected pan predicted to be %.3f, got %.3f", Pan_expected, P0_Predicted)
	}
	if math.Abs(TMinus1_Predicted-Tilt_expected) > 0.001 {
		t.Errorf("Expected tilt predicted to be %.3f, got %.3f", Tilt_expected, TMinus1_Predicted)
	}

	// Pan value of 90 degrees is aligned with the +Y axis of the panPlane
	targetPos_P90_TMinus1 := r3.Vector{X: 0, Y: 100, Z: 8.748866352590167}
	P90_Predicted, TMinus1_Predicted := tracker.predictPanTiltAbsolute(targetPos_P90_TMinus1, cameraPos, panPlane, pan0Direction)
	Pan_expected = 0.5
	Tilt_expected = -1.0
	// With horizontal pan plane, pan=90° should map to normalized 0.5 (for 355° range)
	if math.Abs(P90_Predicted-Pan_expected) > 0.01 {
		t.Errorf("Expected pan predicted to be approximately %.3f, got %.3f", Pan_expected, P90_Predicted)
	}
	if math.Abs(TMinus1_Predicted-Tilt_expected) > 0.001 {
		t.Errorf("Expected tilt predicted to be %.3f, got %.3f", Tilt_expected, TMinus1_Predicted)
	}

	// Pan value of 180 degrees is aligned with the -X axis of the panPlane
	targetPos_P180_TMinus1 := r3.Vector{X: -100, Y: 0, Z: 8.748866352590167}
	P180_Predicted, TMinus1_Predicted := tracker.predictPanTiltAbsolute(targetPos_P180_TMinus1, cameraPos, panPlane, pan0Direction)
	Pan_expected = -0.986
	Tilt_expected = -1.0
	// With horizontal pan plane and 355° wrap-around range, pan=180° maps to approximately -0.986
	if math.Abs(P180_Predicted-Pan_expected) > 0.01 {
		t.Errorf("Expected pan predicted to be approximately %.3f, got %.3f", Pan_expected, P180_Predicted)
	}
	if math.Abs(TMinus1_Predicted-Tilt_expected) > 0.001 {
		t.Errorf("Expected tilt predicted to be %.3f, got %.3f", Tilt_expected, TMinus1_Predicted)
	}
}

func angleFromXYPlane(v r3.Vector) float64 {
	// Method 1: Using atan2 (more numerically stable)
	horizontalDist := math.Sqrt(v.X*v.X + v.Y*v.Y)
	angleRadians := math.Atan2(v.Z, horizontalDist)
	angleDegrees := angleRadians * 180.0 / math.Pi
	return angleDegrees
}

func angleFromXYPlaneRelativeToCamera(targetPos, cameraPos r3.Vector) float64 {
	// Calculate direction vector from camera to target
	direction := targetPos.Sub(cameraPos)
	return angleFromXYPlane(direction)
}

func TestCalculateCameraPosAndPanPlaneCameraNotAtOrigin(t *testing.T) {
	// This camera is not at the origin, so the camera position and pan plane will be different from the origin.
	tracker := &componentTracker{
		logger:                             logging.NewTestLogger(t),
		panMinDeg:                          0.0,
		panMaxDeg:                          355.0,
		tiltMinDeg:                         5.0,
		tiltMaxDeg:                         90.0,
		absoluteCalibrationRayMeasurements: make(map[string]AbsoluteCalibrationRayMeasurements),
	}

	// The camera is located at (100, 100, 100), let's add measurements and make sure we can calculate the camera position and pan plane correctly
	expectedCameraPos := r3.Vector{X: 100, Y: 100, Z: 100}
	expectedPanPlane := r3.Vector{X: 0, Y: 0, Z: 1}
	expectedPan0Direction := r3.Vector{X: 1, Y: 0, Z: 0}
	expectedZeroPanTiltDirection := r3.Vector{X: 0.675590207615660354178999, Y: 0, Z: 0.737277336810124084287565}

	targetPos1 := r3.Vector{X: 200, Y: 200, Z: 200} // Target directly along +X axis, with a tilt of 5 degrees
	pan1 := 0.5
	tilt1 := 0.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos1, pan1, tilt1, "ray1") // Target 1

	targetPos2 := r3.Vector{X: 300, Y: 300, Z: 300} // Target directly along +Y axis, with a tilt of 5 degrees
	pan2 := 0.5
	tilt2 := 0.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos2, pan2, tilt2, "ray1") // Target 2

	targetPos3 := r3.Vector{X: 0, Y: 200, Z: 300} // Target directly along +X axis, with a tilt of 5 degrees
	pan3 := 0.0
	tilt3 := 0.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos3, pan3, tilt3, "ray2") // Target 3

	targetPos4 := r3.Vector{X: -100, Y: 300, Z: 500} // Target directly along +X axis, with a tilt of 5 degrees
	pan4 := 0.0
	tilt4 := 0.0
	tracker.addAbsoluteCalibrationMeasurement(targetPos4, pan4, tilt4, "ray2") // Target 4

	// Get the rays from the measurements
	measuremntsRay1, _ := tracker.getAbsoluteCalibrationMeasurements("ray1")
	ray1, _ := calculateRayFromMeasurements(measuremntsRay1)
	t.Logf("Ray1: %+v", ray1)
	measuremntsRay2, _ := tracker.getAbsoluteCalibrationMeasurements("ray2")
	ray2, _ := calculateRayFromMeasurements(measuremntsRay2)
	t.Logf("Ray2: %+v", ray2)

	// Calculate the camera's position and pan plane from the measurements
	cameraPos, panPlane, pan0Direction, zeroPanTiltDirection := tracker.calculateCameraPositionAndPanPlane(ray1, ray2)
	t.Logf("Camera position: %+v", cameraPos)
	t.Logf("Pan plane: %+v", panPlane)
	t.Logf("Pan0 direction: %+v", pan0Direction)
	t.Logf("Zero pan tilt direction: %+v", zeroPanTiltDirection)

	if cameraPos.Distance(expectedCameraPos) > 0.001 {
		t.Errorf("Expected camera position to be %+v, got %+v", expectedCameraPos, cameraPos)
	}
	if panPlane.Distance(expectedPanPlane) > 0.001 {
		t.Errorf("Expected pan plane to be %+v, got %+v", expectedPanPlane, panPlane)
	}
	if pan0Direction.Distance(expectedPan0Direction) > 0.001 {
		t.Errorf("Expected pan0 direction to be %+v, got %+v", expectedPan0Direction, pan0Direction)
	}
	if zeroPanTiltDirection.Distance(expectedZeroPanTiltDirection) > 0.001 {
		t.Errorf("Expected zero pan tilt direction to be %+v, got %+v", expectedZeroPanTiltDirection, zeroPanTiltDirection)
	}

	// Now calculate angles relative to the camera position
	t.Logf("Target 1 angle from XY plane (relative to camera): %.3f degrees", angleFromXYPlaneRelativeToCamera(targetPos1, cameraPos))
	t.Logf("Target 2 angle from XY plane (relative to camera): %.3f degrees", angleFromXYPlaneRelativeToCamera(targetPos2, cameraPos))
	t.Logf("Target 3 angle from XY plane (relative to camera): %.3f degrees", angleFromXYPlaneRelativeToCamera(targetPos3, cameraPos))
	t.Logf("Target 4 angle from XY plane (relative to camera): %.3f degrees", angleFromXYPlaneRelativeToCamera(targetPos4, cameraPos))
}

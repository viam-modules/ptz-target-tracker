package ptztargettracker

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	"github.com/erh/vmodutils"
	"github.com/erh/vmodutils/touch"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
)

var (
	PoseTracker      = resource.NewModel("viam", "ptz-target-tracker", "component-tracker")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(genericservice.API, PoseTracker,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newPoseTracker,
		},
	)
}

type Config struct {
	TargetComponentName string  `json:"target_component_name"`
	PTZCameraName       string  `json:"ptz_camera_name"`
	OnvifPTZClientName  string  `json:"onvif_ptz_client_name"`
	UpdateRateHz        float64 `json:"update_rate_hz"`
	EnableOnStart       bool    `json:"enable_on_start"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit required (first return) and optional (second return) dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	// Add config validation code here
	if cfg.TargetComponentName == "" {
		return nil, nil, errors.New("target_component_name is required")
	}
	if cfg.PTZCameraName == "" {
		return nil, nil, errors.New("ptz_camera_name is required")
	}
	if cfg.OnvifPTZClientName == "" {
		return nil, nil, errors.New("onvif_ptz_client_name is required")
	}
	if cfg.UpdateRateHz <= 0 {
		return nil, nil, errors.New("update_rate_hz must be greater than 0")
	}
	return nil, nil, nil
}

type poseTracker struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()

	robotClient         robot.Robot
	targetComponentName string
	onvifPTZClientName  string
}

// Close implements resource.Resource.
func (s *poseTracker) Close(ctx context.Context) error {
	panic("unimplemented")
}

// Reconfigure implements resource.Resource.
// Subtle: this method shadows the method (AlwaysRebuild).Reconfigure of poseTracker.AlwaysRebuild.
func (s *poseTracker) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	panic("unimplemented")
}

func newPoseTracker(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewPoseTracker(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewPoseTracker(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	robotClient, err := vmodutils.ConnectToMachineFromEnv(ctx, logger)
	if err != nil {
		cancelFunc()
		return nil, fmt.Errorf("failed to connect to robot: %w", err)
	}

	s := &poseTracker{
		name:                name,
		logger:              logger,
		cfg:                 conf,
		cancelCtx:           cancelCtx,
		cancelFunc:          cancelFunc,
		robotClient:         robotClient,
		targetComponentName: conf.TargetComponentName,
		onvifPTZClientName:  conf.OnvifPTZClientName,
	}

	if conf.EnableOnStart {
		go s.trackingLoop(s.cancelCtx)
		s.logger.Info("PTZ pose tracker started")
	}

	return s, nil
}

func (s *poseTracker) Name() resource.Name {
	return s.name
}

func (s *poseTracker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.logger.Infof("DoCommand: %+v", cmd)
	switch cmd["command"] {
	case "start":
		go s.trackingLoop(s.cancelCtx)
		return map[string]interface{}{"status": "running"}, nil
	case "stop":
		s.cancelFunc()
		return map[string]interface{}{"status": "stopped"}, nil
	case "save-starting-pose":
		startingPose, err := s.saveStartingPose(s.cancelCtx)
		if err != nil {
			return nil, fmt.Errorf("failed to save starting pose: %v", err)
		}
		// Convert pose to readable format with orientation in degrees
		poseDict := s.poseToDictDegrees(startingPose)
		return map[string]interface{}{"status": "starting pose saved", "starting_pose": poseDict}, nil
	case "get-target-pose-in-camera-frame":
		targetPoseInCameraFrame := s.getTargetPoseInCameraFrame(s.cancelCtx)
		if targetPoseInCameraFrame == nil {
			return nil, fmt.Errorf("failed to get target pose in camera frame")
		}
		s.logger.Infof("Target pose in camera frame: %+v", targetPoseInCameraFrame.Pose())
		targetPoseInCameraFrameDegrees := s.poseToDictDegrees(targetPoseInCameraFrame.Pose())
		return map[string]interface{}{"status": "target pose in camera frame", "target_pose_in_camera_frame": targetPoseInCameraFrameDegrees}, nil
	case "get-camera-pose":
		cameraPose := s.getCameraPose(s.cancelCtx)
		if cameraPose == nil {
			return nil, fmt.Errorf("failed to get camera pose")
		}
		return map[string]interface{}{"status": "camera pose", "camera_pose": cameraPose}, nil
	default:
		return nil, fmt.Errorf("invalid command: %v", cmd["command"])
	}
}

// poseToDictDegrees converts a spatialmath.Pose to a dictionary with orientation in degrees
func (s *poseTracker) poseToDictDegrees(pose spatialmath.Pose) map[string]interface{} {
	point := pose.Point()
	ov := pose.Orientation().OrientationVectorDegrees()

	return map[string]interface{}{
		"translation": map[string]interface{}{
			"x": point.X,
			"y": point.Y,
			"z": point.Z,
		},
		"orientation": map[string]interface{}{
			"type": "ov_degrees",
			"value": map[string]interface{}{
				"th": ov.Theta,
				"x":  ov.OX,
				"y":  ov.OY,
				"z":  ov.OZ,
			},
		},
		"parent": "world",
	}
}

/*
Get PTZ status response:

{
  "move_status": {
    "pan_tilt": "IDLE",
    "zoom": "IDLE"
  },
  "utc_time": "2025-11-19T15:44:53Z",
  "position": {
    "zoom": {
      "x": 0.5,
      "space": "http://www.onvif.org/ver10/tptz/ZoomSpaces/PositionGenericSpace"
    },
    "pan_tilt": {
      "space": "http://www.onvif.org/ver10/tptz/PanTiltSpaces/PositionGenericSpace",
      "x": 0,
      "y": 0
    }
  }
}
*/
/*
The using must point the PTZ camera at the target pose before saving the starting pose.
This is needed to be able to link the PTZ angles to the orientation of the camera.
*/
func (t *poseTracker) saveStartingPose(ctx context.Context) (spatialmath.Pose, error) {
	fsc, err := t.robotClient.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil, errors.New("failed to get frame system config")
	}
	targetPose := t.getTargetPose(ctx)
	if targetPose == nil {
		t.logger.Errorf("Failed to get target pose")
		return nil, errors.New("failed to get target pose")
	}
	t.logger.Infof("Starting target pose: %+v", targetPose)

	cameraPose := t.getCameraPose(ctx)
	if cameraPose == nil {
		t.logger.Errorf("Failed to get camera pose")
		return nil, errors.New("failed to get camera pose")
	}
	t.logger.Infof("Camera pose: %+v", cameraPose)

	ov, err := t.alignCameraToTarget(ctx, targetPose, cameraPose)
	if err != nil {
		t.logger.Errorf("Failed to align camera to target: %v", err)
		return nil, errors.New("failed to align camera to target")
	}
	t.logger.Infof("Required Camera orientation: %+v", ov)

	// Now, we need to set the camera pose to the required orientation
	cameraFramePart := touch.FindPart(fsc, t.cfg.PTZCameraName)
	if cameraFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
		return nil, fmt.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
	}
	newCameraPose := spatialmath.NewPose(cameraPose.Pose().Point(), ov)
	t.logger.Infof("New camera pose: %+v", newCameraPose)

	return newCameraPose, nil
}

func (t *poseTracker) alignCameraToTarget(ctx context.Context, targetPoseInCameraFrame *referenceframe.PoseInFrame, cameraPose *referenceframe.PoseInFrame) (*spatialmath.OrientationVector, error) {
	t.logger.Infof("Aligning camera to target")
	t.logger.Infof("Target pose in camera frame: %+v", targetPoseInCameraFrame)
	t.logger.Infof("Camera pose: %+v", cameraPose)
	// OK, We know we have the target component in the center of the camera frame, and we have the camera pose, now we need to calculat the transformation such that the camera's Z axis is aligned with the target's Z axis points towards the target.
	// This means that I want the target pose in camera frame to be such that its orienation is {ox:0, oy:0, oz:1}

	targetPose := t.getTargetPose(ctx)
	if targetPose == nil {
		t.logger.Errorf("Failed to get target pose")
		return nil, errors.New("failed to get target pose")
	}
	t.logger.Infof("Target pose: %+v", targetPose)

	targetPosition := targetPose.Pose().Point()
	t.logger.Infof("Target position: %+v", targetPosition)

	cameraPosition := cameraPose.Pose().Point()
	t.logger.Infof("Camera position: %+v", cameraPosition)

	// Now, we need to calculate the orientation of the camera such that its Z axis points towards the target position.
	// This means that I want the camera pose to be such that its orientation is {ox:0, oy:0, oz:1}

	// 3. Calculate vector from camera to target in world frame
	direction := targetPosition.Sub(cameraPosition)
	t.logger.Infof("Camera to target vector: %+v", direction)

	// 4. Normalize
	length := direction.Norm()
	if length < 1e-6 {
		t.logger.Errorf("Target too close to camera")
		return nil, errors.New("target too close to camera")
	}
	direction = direction.Mul(1.0 / length)
	t.logger.Infof("Direction vector (normalized): (%f, %f, %f)", direction.X, direction.Y, direction.Z)

	// 5. We want camera's +Z axis to point along this direction
	desiredZ := direction
	defaultZ := r3.Vector{X: 0, Y: 0, Z: 1} // Camera's +Z in default orientation
	t.logger.Infof("Direction to target: (%f, %f, %f)", direction.X, direction.Y, direction.Z)

	// 6. Calculate rotation axis (perpendicular to both)
	axis := defaultZ.Cross(desiredZ)
	t.logger.Infof("Calculated axis: (%f, %f, %f)", axis.X, axis.Y, axis.Z)

	// 7. Calculate rotation angle
	angle := math.Acos(defaultZ.Dot(desiredZ))
	t.logger.Infof("Calculated angle: %f°", angle*180/math.Pi)

	// Handle parallel/antiparallel cases
	if axis.Norm() < 1e-6 {
		if defaultZ.Dot(direction) > 0 {
			// Already aligned
			return spatialmath.NewOrientationVector(), nil
		} else {
			// 180° rotation
			axis = r3.Vector{X: 1, Y: 0, Z: 0}
			angle = math.Pi
		}
	} else {
		axis = axis.Mul(1.0 / axis.Norm())
	}

	ov := &spatialmath.OrientationVector{
		Theta: angle, // Keep in radians for the internal representation
		OX:    axis.X,
		OY:    axis.Y,
		OZ:    axis.Z,
	}

	t.logger.Infof("Required camera orientation: axis=(%f, %f, %f), angle=%f°",
		axis.X, axis.Y, axis.Z, angle*180/math.Pi)

	return ov, nil
}

func (t *poseTracker) getCameraPose(ctx context.Context) *referenceframe.PoseInFrame {
	fsc, err := t.robotClient.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil
	}
	cameraFramePart := touch.FindPart(fsc, t.cfg.PTZCameraName)
	if cameraFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
		return nil
	}
	cameraPose, err := t.robotClient.GetPose(ctx, cameraFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose: %v", err)
		return nil
	}
	t.logger.Infof("Camera pose: %+v", cameraPose)
	return cameraPose
}

func (t *poseTracker) getTargetPose(ctx context.Context) *referenceframe.PoseInFrame {
	fsc, err := t.robotClient.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil
	}
	targetFramePart := touch.FindPart(fsc, t.targetComponentName)
	if targetFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.targetComponentName)
		return nil
	}
	targetPose, err := t.robotClient.GetPose(ctx, targetFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose: %v", err)
		return nil
	}

	return targetPose
}

func (t *poseTracker) getTargetPoseInCameraFrame(ctx context.Context) *referenceframe.PoseInFrame {
	fsc, err := t.robotClient.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil
	}
	targetFramePart := touch.FindPart(fsc, t.targetComponentName)
	if targetFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.targetComponentName)
		return nil
	}
	targetPose, err := t.robotClient.GetPose(ctx, targetFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose: %v", err)
		return nil
	}
	cameraFramePart := touch.FindPart(fsc, t.cfg.PTZCameraName)
	if cameraFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
		return nil
	}
	targetPoseInCameraFrame, err := t.robotClient.TransformPose(ctx, targetPose, cameraFramePart.FrameConfig.Name(), []*referenceframe.LinkInFrame{})
	if err != nil {
		t.logger.Errorf("Failed to transform target pose to camera frame: %v", err)
		return nil
	}
	t.logger.Infof("Target pose in camera frame: %+v", targetPoseInCameraFrame)

	return targetPoseInCameraFrame
}

func (t *poseTracker) getCameraCurrentPTZStatus(ctx context.Context) (float64, float64, float64, error) {
	onvifPTZClientName := resource.NewName(generic.API, t.onvifPTZClientName)
	onvifPTZClient, err := t.robotClient.ResourceByName(onvifPTZClientName)
	if err != nil {
		t.logger.Errorf("Failed to get onvif PTZ client: %v", err)
		return 0, 0, 0, err
	}
	ptzStatusResponse, err := onvifPTZClient.DoCommand(ctx, map[string]interface{}{
		"command": "get-status",
	})
	if err != nil {
		t.logger.Errorf("Failed to get PTZ status: %v", err)
		return 0, 0, 0, err
	}
	moveStatus, ok := ptzStatusResponse["move_status"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ move status is not a map")
		return 0, 0, 0, err
	}
	movePanTilt, ok := moveStatus["pan_tilt"].(string)
	if !ok {
		t.logger.Errorf("PTZ move pan tilt is not a string")
		return 0, 0, 0, err
	}
	if movePanTilt != "IDLE" {
		t.logger.Errorf("PTZ is not idle")
		return 0, 0, 0, err
	}
	moveZoom, ok := moveStatus["zoom"].(string)
	if !ok {
		t.logger.Errorf("PTZ move zoom is not a string")
		return 0, 0, 0, err
	}
	if moveZoom != "IDLE" {
		t.logger.Errorf("PTZ is not idle")
		return 0, 0, 0, err
	}
	position, ok := ptzStatusResponse["position"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ status is not a map")
		return 0, 0, 0, err
	}
	zoom, ok := position["zoom"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ zoom is not a map")
		return 0, 0, 0, err
	}
	zoomX, ok := zoom["x"].(float64)
	if !ok {
		t.logger.Errorf("PTZ zoom x is not a float")
		return 0, 0, 0, err
	}
	panTilt, ok := position["pan_tilt"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ pan tilt is not a map")
		return 0, 0, 0, err
	}
	panTiltX, ok := panTilt["x"].(float64)
	if !ok {
		t.logger.Errorf("PTZ pan tilt x is not a float")
		return 0, 0, 0, err
	}
	panTiltY, ok := panTilt["y"].(float64)
	if !ok {
		t.logger.Errorf("PTZ pan tilt y is not a float")
		return 0, 0, 0, err
	}
	t.logger.Infof("PTZ status: zoom=%.1f, pan=%.1f, tilt=%.1f", zoomX, panTiltX, panTiltY)

	return panTiltX, panTiltY, zoomX, nil
}

func (t *poseTracker) trackingLoop(ctx context.Context) {
	t.logger.Info("Starting tracking loop")
	t.logger.Info("Update rate: %f Hz", t.cfg.UpdateRateHz)
	var updateInterval time.Duration = time.Duration(1.0 / t.cfg.UpdateRateHz * float64(time.Second))
	t.logger.Info("Update interval: %v", updateInterval)
	ticker := time.NewTicker(updateInterval)
	defer ticker.Stop()

	fsc, err := t.robotClient.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return
	}
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			// 1. Get the target pose in the frame system
			targetFramePart := touch.FindPart(fsc, t.targetComponentName)
			if targetFramePart == nil {
				t.logger.Errorf("can't find frame for %v", t.targetComponentName)
				continue
			}
			targetPose, err := t.robotClient.GetPose(ctx, targetFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
			if err != nil {
				t.logger.Errorf("Failed to get pose: %v", err)
				continue
			}
			t.logger.Infof("Target pose: %+v", targetPose)

			cameraFramePart := touch.FindPart(fsc, t.cfg.PTZCameraName)
			if cameraFramePart == nil {
				t.logger.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
				continue
			}
			cameraPose, err := t.robotClient.GetPose(ctx, cameraFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
			if err != nil {
				t.logger.Errorf("Failed to get pose: %v", err)
				continue
			}
			t.logger.Infof("Camera pose: %+v", cameraPose)

			targetPoseInCameraFrame, err := t.robotClient.TransformPose(ctx, targetPose, cameraFramePart.FrameConfig.Name(), []*referenceframe.LinkInFrame{})
			if err != nil {
				t.logger.Errorf("Failed to transform target pose to camera frame: %v", err)
				continue
			}
			t.logger.Infof("Target pose in camera frame: %+v", targetPoseInCameraFrame)

			// Log position details for debugging
			pos := targetPoseInCameraFrame.Pose().Point()
			t.logger.Infof("Target position relative to camera: X=%.1f (right+), Y=%.1f (up+), Z=%.1f (forward+)",
				pos.X, pos.Y, pos.Z)

			// 3. Calculate pan/tilt angles needed to center the target in the PTZ camera frame
			pan, tilt, zoom := t.calculatePanTiltZoom(targetPoseInCameraFrame)
			t.logger.Infof("Pan: %f, Tilt: %f, Zoom: %f", pan, tilt, zoom)

			// 4. Send relative move command to PTZ
			err = t.movePTZ(ctx, pan, tilt, zoom)
			if err != nil {
				t.logger.Errorf("Failed to move PTZ: %v", err)
			}
		}
	}
}

func (t *poseTracker) calculatePanTiltZoom(targetPoseInCameraFrame *referenceframe.PoseInFrame) (float64, float64, float64) {
	t.logger.Infof("Calculating pan and tilt")
	t.logger.Infof("Target pose in camera frame: %+v", targetPoseInCameraFrame)

	// Position relative to camera
	x := targetPoseInCameraFrame.Pose().Point().X // Right/Left (positive = right)
	y := targetPoseInCameraFrame.Pose().Point().Y // Up/Down (positive = up)
	z := targetPoseInCameraFrame.Pose().Point().Z // Forward/Back (positive = forward)

	// Check if target is behind the camera
	if z < 0 {
		t.logger.Warnf("Target is behind camera (Z=%.1f). Cannot track.", z)
		return 0, 0, 0
	}

	// Calculate distance for zoom
	distance := math.Sqrt(x*x + y*y + z*z)

	// Calculate horizontal distance (in XZ plane) for tilt calculation
	horizontalDist := math.Sqrt(x*x + z*z)

	// Calculate angles in degrees
	// Pan: rotation around vertical axis (positive = rotate right)
	pan := math.Atan2(x, z) * 180.0 / math.Pi

	// Tilt: rotation around horizontal axis (positive = rotate up)
	// Use horizontal distance to ensure tilt stays in valid range [-90, 90]
	tilt := math.Atan2(y, horizontalDist) * 180.0 / math.Pi

	t.logger.Infof("Distance: %.1fmm, Pan: %.1f°, Tilt: %.1f°", distance, pan, tilt)

	// For relative-move tracking, we return angles in degrees and normalized zoom
	// Pan and tilt represent how much the camera needs to rotate to point at the target

	// Convert zoom to normalized coordinates based on distance
	// Map distance range to zoom range (0.0 to 1.0)
	var zoomNormalized float64
	zoomNormalized = 1.0

	return pan, tilt, zoomNormalized
}

/*
Move the PTZ camera to the given pan, tilt, and zoom.
From Viam RTSP PTZ documentation:
Notes
Disclaimer: This model was made in order to fully integrate with one specific camera. I tried to generalize it to all PTZ cameras, but your mileage may vary.
Profile Discovery: Use get-profiles command to discover valid profile tokens
Coordinate Spaces:
Normalized: -1.0 to 1.0 (pan/tilt), 0.0-1.0 (zoom)
Degrees: -180° to 180° (pan), -90° to 90° (tilt)
Absolute Moves: Use normalized coordinates (-1.0 to 1.0 for pan/tilt, 0.0 to 1.0 for zoom).
Relative Moves:
Normalized (degrees: false): -1.0 to 1.0 (pan/tilt/zoom).
Degrees (degrees: true): -180° to 180° (pan), -90° to 90° (tilt). Zoom remains normalized.
Movement Speeds:
Continuous: -1.0 (full reverse) to 1.0 (full forward).
Relative/Absolute: Speed parameters (pan_speed, tilt_speed, zoom_speed between 0.0 and 1.0) are optional. If no speed parameters are provided, the camera uses its default speed. If any speed parameter is provided, the Speed element is included in the request (using defaults of 0.5 for Relative or 1.0 for Absolute for any unspecified speed components).
*/

func (t *poseTracker) movePTZ(ctx context.Context, panError float64, tiltError float64, zoomNormalized float64) error {
	// onvifPTZClientName := resource.NewName(generic.API, t.onvifPTZClientName)
	// onvifPTZClient, err := t.robotClient.ResourceByName(onvifPTZClientName)
	// if err != nil {
	// 	return fmt.Errorf("failed to get onvif PTZ client: %w", err)
	// }

	// Check if we're within deadband (target is centered)
	return nil
}

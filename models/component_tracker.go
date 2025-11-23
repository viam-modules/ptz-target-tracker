package models

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
	"gonum.org/v1/gonum/mat"
)

var (
	ComponentTracker = resource.NewModel("viam", "ptz-target-tracker", "component-tracker")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(genericservice.API, ComponentTracker,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newComponentTracker,
		},
	)
}

type Config struct {
	TargetComponentName string  `json:"target_component_name"`
	PTZCameraName       string  `json:"ptz_camera_name"`
	OnvifPTZClientName  string  `json:"onvif_ptz_client_name"`
	UpdateRateHz        float64 `json:"update_rate_hz"`
	EnableOnStart       bool    `json:"enable_on_start"`
	PanSpeed            float64 `json:"pan_speed"`
	TiltSpeed           float64 `json:"tilt_speed"`
	ZoomSpeed           float64 `json:"zoom_speed"`
	PanMinDeg           float64 `json:"pan_min_deg"`
	PanMaxDeg           float64 `json:"pan_max_deg"`
	TiltMinDeg          float64 `json:"tilt_min_deg"`
	TiltMaxDeg          float64 `json:"tilt_max_deg"`
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
	if cfg.PanSpeed <= 0 || cfg.PanSpeed > 1 {
		return nil, nil, errors.New("pan_speed must be greater than 0 and less than or equal to 1")
	}
	if cfg.TiltSpeed <= 0 || cfg.TiltSpeed > 1 {
		return nil, nil, errors.New("tilt_speed must be greater than 0 and less than or equal to 1")
	}
	if cfg.ZoomSpeed <= 0 || cfg.ZoomSpeed > 1 {
		return nil, nil, errors.New("zoom_speed must be greater than 0 and less than or equal to 1")
	}
	if cfg.PanMinDeg == 0 && cfg.PanMaxDeg == 0 {
		cfg.PanMinDeg = 0
		cfg.PanMaxDeg = 355
	}
	if cfg.TiltMinDeg == 0 && cfg.TiltMaxDeg == 0 {
		cfg.TiltMinDeg = 5
		cfg.TiltMaxDeg = 90
	}
	if cfg.PanMaxDeg <= cfg.PanMinDeg {
		return nil, nil, errors.New("pan_max_deg must be greater than pan_min_deg")
	}
	if cfg.TiltMaxDeg <= cfg.TiltMinDeg {
		return nil, nil, errors.New("tilt_max_deg must be greater than tilt_min_deg")
	}
	return nil, nil, nil
}

type TrackingSample struct {
	TargetPos r3.Vector
	Pan       float64
	Tilt      float64
}

type componentTracker struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()
	running    bool

	robotClient         robot.Robot
	targetComponentName string
	onvifPTZClientName  string

	baselinePan            float64
	baselineTilt           float64
	baselineZoomX          float64
	baslineTargetPosition  r3.Vector
	baselineCameraPosition r3.Vector
	panSpeed               float64
	tiltSpeed              float64
	zoomSpeed              float64
	updateRateHz           float64
	panMinDeg              float64
	panMaxDeg              float64
	tiltMinDeg             float64
	tiltMaxDeg             float64
	reversePan             bool
	samples                []TrackingSample
	// Fitted coefficients: pan = Ax + By + Cz + D
	panCoeffs  [4]float64 // [A, B, C, D]
	tiltCoeffs [4]float64 // [A, B, C, D]

	// Or polynomial: pan = Ax² + By² + Cz² + Dxy + Exz + Fyz + Gx + Hy + Iz + J
	panPolyCoeffs  [10]float64
	tiltPolyCoeffs [10]float64

	usePolynomial bool
	isCalibrated  bool
}

// Close implements resource.Resource.
func (s *componentTracker) Close(ctx context.Context) error {
	panic("unimplemented")
}

// Reconfigure implements resource.Resource.
// Subtle: this method shadows the method (AlwaysRebuild).Reconfigure of poseTracker.AlwaysRebuild.
func (s *componentTracker) Reconfigure(ctx context.Context, deps resource.Dependencies, rawConf resource.Config) error {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return err
	}

	s.logger.Infof("Reconfiguring pose tracker with pan speed: %f, tilt speed: %f, zoom speed: %f", conf.PanSpeed, conf.TiltSpeed, conf.ZoomSpeed)
	wasRunning := s.running
	if s.running {
		s.cancelFunc()
		s.running = false
	}
	s.targetComponentName = conf.TargetComponentName
	s.onvifPTZClientName = conf.OnvifPTZClientName
	s.updateRateHz = conf.UpdateRateHz
	s.panSpeed = conf.PanSpeed
	s.tiltSpeed = conf.TiltSpeed
	s.zoomSpeed = conf.ZoomSpeed
	s.panMinDeg = conf.PanMinDeg
	s.panMaxDeg = conf.PanMaxDeg
	s.tiltMinDeg = conf.TiltMinDeg
	s.tiltMaxDeg = conf.TiltMaxDeg
	if wasRunning {
		go s.trackingLoop(s.cancelCtx)
		s.logger.Info("PTZ pose tracker restarted")
		s.running = true
	}
	return nil
}

func newComponentTracker(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewComponentTracker(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewComponentTracker(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	robotClient, err := vmodutils.ConnectToMachineFromEnv(ctx, logger)
	if err != nil {
		cancelFunc()
		return nil, fmt.Errorf("failed to connect to robot: %w", err)
	}

	s := &componentTracker{
		name:                name,
		logger:              logger,
		cfg:                 conf,
		cancelCtx:           cancelCtx,
		cancelFunc:          cancelFunc,
		robotClient:         robotClient,
		targetComponentName: conf.TargetComponentName,
		onvifPTZClientName:  conf.OnvifPTZClientName,
		panSpeed:            conf.PanSpeed,
		tiltSpeed:           conf.TiltSpeed,
		zoomSpeed:           conf.ZoomSpeed,
		updateRateHz:        conf.UpdateRateHz,
		panMinDeg:           conf.PanMinDeg,
		panMaxDeg:           conf.PanMaxDeg,
		tiltMinDeg:          conf.TiltMinDeg,
		tiltMaxDeg:          conf.TiltMaxDeg,
	}

	if conf.EnableOnStart {
		go s.trackingLoop(s.cancelCtx)
		s.logger.Info("PTZ component tracker started")
	}

	return s, nil
}

func (s *componentTracker) Name() resource.Name {
	return s.name
}

func (t *componentTracker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	t.logger.Infof("DoCommand: %+v", cmd)
	switch cmd["command"] {
	case "start":
		if t.running {
			return map[string]interface{}{"status": "already_running"}, nil
		}
		// Cancel old context and create new one for fresh start
		if t.cancelFunc != nil {
			t.cancelFunc()
		}
		t.cancelCtx, t.cancelFunc = context.WithCancel(context.Background())
		t.running = true
		go t.trackingLoop(t.cancelCtx)
		return map[string]interface{}{"status": "running"}, nil
	case "stop":
		if !t.running {
			return map[string]interface{}{"status": "already_stopped"}, nil
		}
		t.running = false
		if t.cancelFunc != nil {
			t.cancelFunc() // Cancel context to exit the loop
		}
		return map[string]interface{}{"status": "stopped"}, nil
	case "set-zoom":
		zoom, ok := cmd["zoom"].(float64)
		if !ok {
			return nil, fmt.Errorf("zoom is not a float")
		}
		t.baselineZoomX = zoom
		return map[string]interface{}{"status": "success", "zoom": t.baselineZoomX}, nil
	case "reverse-pan":
		val, ok := cmd["value"].(bool)
		if !ok {
			return nil, fmt.Errorf("value is not a bool")
		}
		t.reversePan = val
		return map[string]interface{}{"status": "success", "reversePan": t.reversePan}, nil
	case "calibrate":
		err := t.recordBaseline(t.cancelCtx)
		if err != nil {
			return nil, fmt.Errorf("failed to record baseline: %v", err)
		}
		return map[string]interface{}{
			"status":        "success",
			"baselinePan":   t.baselinePan,
			"baselineTilt":  t.baselineTilt,
			"baselineZoomX": t.baselineZoomX,
			"baselineTargetPosition": map[string]interface{}{
				"x": t.baslineTargetPosition.X,
				"y": t.baslineTargetPosition.Y,
				"z": t.baslineTargetPosition.Z,
			},
			"baselineCameraPosition": map[string]interface{}{
				"x": t.baselineCameraPosition.X,
				"y": t.baselineCameraPosition.Y,
				"z": t.baselineCameraPosition.Z,
			},
		}, nil

	case "get-calibration":
		// Return calibration data in a format that can be saved and later loaded
		return map[string]interface{}{
			"status":        "success",
			"baselinePan":   t.baselinePan,
			"baselineTilt":  t.baselineTilt,
			"baselineZoomX": t.baselineZoomX,
			"baselineTargetPosition": map[string]interface{}{
				"x": t.baslineTargetPosition.X,
				"y": t.baslineTargetPosition.Y,
				"z": t.baslineTargetPosition.Z,
			},
			"baselineCameraPosition": map[string]interface{}{
				"x": t.baselineCameraPosition.X,
				"y": t.baselineCameraPosition.Y,
				"z": t.baselineCameraPosition.Z,
			},
		}, nil

	case "load-calibration":
		// Load calibration data from command parameters
		baselinePan, ok := cmd["baselinePan"].(float64)
		if !ok {
			return nil, fmt.Errorf("baselinePan is required and must be a float64")
		}
		baselineTilt, ok := cmd["baselineTilt"].(float64)
		if !ok {
			return nil, fmt.Errorf("baselineTilt is required and must be a float64")
		}
		baselineZoomX, ok := cmd["baselineZoomX"].(float64)
		if !ok {
			return nil, fmt.Errorf("baselineZoomX is required and must be a float64")
		}

		baselineTargetPosition, ok := cmd["baselineTargetPosition"].(map[string]interface{})
		if !ok {
			return nil, fmt.Errorf("baselineDirection is required and must be a map")
		}

		targetX, ok := baselineTargetPosition["x"].(float64)
		if !ok {
			return nil, fmt.Errorf("baselineDirection.x is required and must be a float64")
		}
		targetY, ok := baselineTargetPosition["y"].(float64)
		if !ok {
			return nil, fmt.Errorf("baselineDirection.y is required and must be a float64")
		}
		targetZ, ok := baselineTargetPosition["z"].(float64)
		if !ok {
			return nil, fmt.Errorf("baselineDirection.z is required and must be a float64")
		}

		baselineCameraPosition, ok := cmd["baselineCameraPosition"].(map[string]interface{})
		if !ok {
			return nil, fmt.Errorf("baselineCameraPosition is required and must be a map")
		}

		cameraX, ok := baselineCameraPosition["x"].(float64)
		if !ok {
			return nil, fmt.Errorf("baselineCameraPosition.x is required and must be a float64")
		}
		cameraY, ok := baselineCameraPosition["y"].(float64)
		if !ok {
			return nil, fmt.Errorf("baselineCameraPosition.y is required and must be a float64")
		}
		cameraZ, ok := baselineCameraPosition["z"].(float64)
		if !ok {
			return nil, fmt.Errorf("baselineCameraPosition.z is required and must be a float64")
		}

		cameraPosition := r3.Vector{X: cameraX, Y: cameraY, Z: cameraZ}

		// Validate direction vector is normalized (or normalize it)
		targetPosition := r3.Vector{X: targetX, Y: targetY, Z: targetZ}

		// Load the calibration data
		t.baselinePan = baselinePan
		t.baselineTilt = baselineTilt
		t.baselineZoomX = baselineZoomX
		t.baslineTargetPosition = targetPosition
		t.baselineCameraPosition = cameraPosition

		t.logger.Infof("Loaded calibration: pan=%.3f, tilt=%.3f, zoom=%.3f, direction=(%.3f, %.3f, %.3f)",
			t.baselinePan, t.baselineTilt, t.baselineZoomX,
			t.baslineTargetPosition.X, t.baslineTargetPosition.Y, t.baslineTargetPosition.Z)

		return map[string]interface{}{
			"status":        "success",
			"baselinePan":   t.baselinePan,
			"baselineTilt":  t.baselineTilt,
			"baselineZoomX": t.baselineZoomX,
			"baselineTargetPosition": map[string]interface{}{
				"x": t.baslineTargetPosition.X,
				"y": t.baslineTargetPosition.Y,
				"z": t.baslineTargetPosition.Z,
			},
			"baselineCameraPosition": map[string]interface{}{
				"x": t.baselineCameraPosition.X,
				"y": t.baselineCameraPosition.Y,
				"z": t.baselineCameraPosition.Z,
			},
		}, nil

	case "add-sample":
		// Get current target position from arm
		targetPose := t.getTargetPose(ctx)
		if targetPose == nil {
			return nil, errors.New("failed to get target pose")
		}
		targetPos := targetPose.Pose().Point()

		// Get current pan/tilt (user has manually centered)
		pan, tilt, _, err := t.getCameraCurrentPTZStatus(ctx)
		if err != nil {
			return nil, err
		}

		sample := TrackingSample{
			TargetPos: targetPos,
			Pan:       pan,
			Tilt:      tilt,
		}
		t.samples = append(t.samples, sample)

		t.logger.Infof("Sample %d: (%.1f, %.1f, %.1f) → pan=%.4f, tilt=%.4f",
			len(t.samples), targetPos.X, targetPos.Y, targetPos.Z, pan, tilt)

		return map[string]interface{}{
			"sample_number": len(t.samples),
			"target":        map[string]interface{}{"x": targetPos.X, "y": targetPos.Y, "z": targetPos.Z},
			"pan":           pan,
			"tilt":          tilt,
		}, nil

	case "fit-linear":
		if len(t.samples) < 4 {
			return nil, fmt.Errorf("need at least 4 samples, have %d", len(t.samples))
		}

		panErr, tiltErr := t.fitLinear()
		t.usePolynomial = false
		t.isCalibrated = true

		return map[string]interface{}{
			"status":         "success",
			"pan_coeffs":     []float64{t.panCoeffs[0], t.panCoeffs[1], t.panCoeffs[2], t.panCoeffs[3]},
			"tilt_coeffs":    []float64{t.tiltCoeffs[0], t.tiltCoeffs[1], t.tiltCoeffs[2], t.tiltCoeffs[3]},
			"pan_error_avg":  panErr,
			"tilt_error_avg": tiltErr,
			"samples_used":   len(t.samples),
		}, nil

	case "fit-polynomial":
		if len(t.samples) < 10 {
			return nil, fmt.Errorf("need at least 10 samples for polynomial fit, have %d", len(t.samples))
		}

		panErr, tiltErr := t.fitPolynomial()
		t.usePolynomial = true
		t.isCalibrated = true

		return map[string]interface{}{
			"status":         "success",
			"pan_error_avg":  panErr,
			"tilt_error_avg": tiltErr,
			"samples_used":   len(t.samples),
		}, nil

	case "clear-samples":
		t.samples = nil
		t.isCalibrated = false
		return map[string]interface{}{"status": "cleared"}, nil

	case "test-prediction":
		if !t.isCalibrated {
			return nil, errors.New("not calibrated")
		}

		targetPose := t.getTargetPose(ctx)
		targetPos := targetPose.Pose().Point()

		pan, tilt := t.predictPanTilt(targetPos)

		actualPan, actualTilt, _, _ := t.getCameraCurrentPTZStatus(ctx)

		return map[string]interface{}{
			"target":         map[string]interface{}{"x": targetPos.X, "y": targetPos.Y, "z": targetPos.Z},
			"predicted_pan":  pan,
			"predicted_tilt": tilt,
			"actual_pan":     actualPan,
			"actual_tilt":    actualTilt,
			"pan_error":      pan - actualPan,
			"tilt_error":     tilt - actualTilt,
		}, nil

	default:
		return nil, fmt.Errorf("invalid command: %v", cmd["command"])
	}
}

// Call this after calibration (camera pointing at target)
func (t *componentTracker) recordBaseline(ctx context.Context) error {
	// 1. Get current PTZ position
	panTiltX, panTiltY, zoomX, err := t.getCameraCurrentPTZStatus(ctx)
	if err != nil {
		return fmt.Errorf("failed to get camera current PTZ status: %v", err)
	}

	t.baselinePan = panTiltX
	t.baselineTilt = panTiltY
	t.baselineZoomX = zoomX

	// 2. Get target position in world
	targetPose := t.getTargetPose(ctx)
	targetPos := targetPose.Pose().Point()

	t.baslineTargetPosition = targetPos

	cameraPose := t.getCameraPose(ctx)
	if cameraPose == nil {
		return errors.New("failed to get camera pose")
	}
	cameraPos := cameraPose.Pose().Point()

	t.baselineCameraPosition = cameraPos

	// 3. Calculate baseline direction
	t.logger.Infof("Baseline calibration:")
	t.logger.Infof("  Pan: %f, Tilt: %f", t.baselinePan, t.baselineTilt)
	t.logger.Infof("  Target position: (%f, %f, %f)",
		t.baslineTargetPosition.X, t.baslineTargetPosition.Y, t.baslineTargetPosition.Z)
	t.logger.Infof("  Camera position: (%f, %f, %f)",
		t.baselineCameraPosition.X, t.baselineCameraPosition.Y, t.baselineCameraPosition.Z)

	return nil
}

func (t *componentTracker) getCameraPose(ctx context.Context) *referenceframe.PoseInFrame {

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

	return cameraPose
}

func (t *componentTracker) getTargetPose(ctx context.Context) *referenceframe.PoseInFrame {
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

func (t *componentTracker) getCameraCurrentPTZStatus(ctx context.Context) (float64, float64, float64, error) {
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

func (t *componentTracker) trackingLoop(ctx context.Context) {
	t.logger.Info("Starting tracking loop")
	t.logger.Info("Update rate: %f Hz", t.cfg.UpdateRateHz)
	var updateInterval time.Duration = time.Duration(1.0 / t.cfg.UpdateRateHz * float64(time.Second))
	t.logger.Info("Update interval: %v", updateInterval)
	ticker := time.NewTicker(updateInterval)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
			if !t.running {
				continue
			}
			err := t.trackTarget(ctx)
			if err != nil {
				t.logger.Errorf("Failed to track target: %v", err)
			}
		}
	}
}

/*

// Main tracking loop
func (t *componentTracker) trackTarget(ctx context.Context) error {
	// 1. Get target position in world frame
	targetPose := t.getTargetPose(ctx)
	targetPos := targetPose.Pose().Point()

	t.logger.Debugf("Target position: %+v", targetPos)

	cameraPose := t.getCameraPose(ctx)
	if cameraPose == nil {
		return errors.New("failed to get camera pose")
	}
	cameraPos := cameraPose.Pose().Point()
	t.logger.Debugf("Camera position: %+v", cameraPos)

	// 2. Calculate current direction from camera to target
	currentDirection := r3.Vector{
		X: targetPos.X - cameraPos.X,
		Y: targetPos.Y - cameraPos.Y,
		Z: targetPos.Z - cameraPos.Z,
	}

	distance := currentDirection.Norm()
	if distance < 1e-6 {
		return errors.New("target too close to camera")
	}
	currentDirection = currentDirection.Mul(1.0 / distance)

	// 3. Calculate pan/tilt to point at this direction
	// Pass current camera position so baseline direction can be recalculated dynamically
	pan, tilt := t.directionToPanTilt(currentDirection, cameraPos)
	zoom := t.baselineZoomX

	t.logger.Debugf("Tracking: distance=%f, pan=%f, tilt=%f, zoom=%f", distance, pan, tilt, zoom)

	// 4. Send absolute move command
	return t.sendAbsoluteMove(ctx, pan, tilt, zoom)
}
*/

// Convert world direction to PTZ pan/tilt values
func (t *componentTracker) directionToPanTilt(direction r3.Vector, currentCameraPos r3.Vector) (pan, tilt float64) {
	// Calculate the angular offset from baseline direction
	// Recalculate baseline direction each iteration using current camera position
	// This allows experimenting with moving the camera position dynamically

	// Compute baseline direction vector from current camera position to baseline target position
	baselineDirectionVec := r3.Vector{
		X: t.baslineTargetPosition.X - currentCameraPos.X,
		Y: t.baslineTargetPosition.Y - currentCameraPos.Y,
		Z: t.baslineTargetPosition.Z - currentCameraPos.Z,
	}
	baselineDirNorm := baselineDirectionVec.Norm()
	if baselineDirNorm < 1e-6 {
		t.logger.Errorf("Baseline direction vector has zero length")
		return t.baselinePan, t.baselineTilt
	}
	baselineDirectionVec = baselineDirectionVec.Mul(1.0 / baselineDirNorm)

	// Decompose into horizontal (XY plane) and vertical components
	// Horizontal angle (pan): project onto XY plane
	baselineHorizontal := r3.Vector{
		X: baselineDirectionVec.X,
		Y: baselineDirectionVec.Y,
		Z: 0,
	}
	currentHorizontal := r3.Vector{
		X: direction.X,
		Y: direction.Y,
		Z: 0,
	}

	// Normalize horizontal vectors
	baselineHorizNorm := baselineHorizontal.Norm()
	currentHorizNorm := currentHorizontal.Norm()

	var panOffset float64
	if baselineHorizNorm > 1e-6 && currentHorizNorm > 1e-6 {
		// Normalize the vectors, now we are operating in the unit circle
		baselineHorizontal = baselineHorizontal.Mul(1.0 / baselineHorizNorm)
		currentHorizontal = currentHorizontal.Mul(1.0 / currentHorizNorm)

		// crossZ gives the sign of the angle between the two vectors (positive if current is counter-clockwise of baseline)
		crossZ := baselineHorizontal.X*currentHorizontal.Y - baselineHorizontal.Y*currentHorizontal.X
		// dot gives the cosine of the angle between the two vectors
		dot := baselineHorizontal.Dot(currentHorizontal)
		// Atan2 returns the angle between the two vectors in the range [-π, π]
		panOffset = math.Atan2(crossZ, dot)
	}

	// Vertical angle (tilt): elevation difference
	baselineElevation := math.Asin(clampMinusOneToOne(baselineDirectionVec.Z))
	currentElevation := math.Asin(clampMinusOneToOne(direction.Z))
	tiltOffset := currentElevation - baselineElevation

	panMinDeg := t.panMinDeg
	panMaxDeg := t.panMaxDeg
	if panMaxDeg <= panMinDeg {
		panMinDeg = 0
		panMaxDeg = 355
	}

	tiltMinDeg := t.tiltMinDeg
	tiltMaxDeg := t.tiltMaxDeg
	if tiltMaxDeg <= tiltMinDeg {
		tiltMinDeg = 5
		tiltMaxDeg = 90
	}

	panOffsetDeg := radToDeg(panOffset)
	tiltOffsetDeg := radToDeg(tiltOffset)

	baselinePanDeg := normalizedToDegrees(t.baselinePan, panMinDeg, panMaxDeg)
	baselineTiltDeg := normalizedToDegrees(t.baselineTilt, tiltMinDeg, tiltMaxDeg)

	t.logger.Debugf("Pan calculation: baselinePanNorm=%.3f, baselinePanDeg=%.1f, panOffsetRad=%.3f, panOffsetDeg=%.1f",
		t.baselinePan, baselinePanDeg, panOffset, panOffsetDeg)
	t.logger.Debugf("Baseline direction: (%.3f, %.3f, %.3f), Current direction: (%.3f, %.3f, %.3f)",
		direction.X, direction.Y, direction.Z,
		direction.X, direction.Y, direction.Z)

	// Apply offsets in degree space so we respect the asymmetric limits.
	var panDeg float64
	if t.reversePan {
		panDeg = baselinePanDeg + panOffsetDeg
	} else {
		panDeg = baselinePanDeg - panOffsetDeg
	}
	tiltDeg := baselineTiltDeg - tiltOffsetDeg

	panDeg = clampFloat(panDeg, panMinDeg, panMaxDeg)
	tiltDeg = clampFloat(tiltDeg, tiltMinDeg, tiltMaxDeg)

	// Convert back to the normalized [-1, 1] range that ONVIF expects.
	pan = degreesToNormalized(panDeg, panMinDeg, panMaxDeg)
	tilt = degreesToNormalized(tiltDeg, tiltMinDeg, tiltMaxDeg)

	// Final safety clamp to ONVIF normalized limits.
	pan = clampFloat(pan, -1.0, 1.0)
	tilt = clampFloat(tilt, -1.0, 1.0)

	if math.IsNaN(pan) || math.IsNaN(tilt) {
		t.logger.Errorf("directionToPanTilt produced NaN (pan=%v, tilt=%v) baselinePan=%f baselineTilt=%f panDeg=%f tiltDeg=%f offsets=(%f,%f) ranges pan[%f,%f] tilt[%f,%f]",
			pan, tilt, t.baselinePan, t.baselineTilt, panDeg, tiltDeg, panOffsetDeg, tiltOffsetDeg, panMinDeg, panMaxDeg, tiltMinDeg, tiltMaxDeg)
		return t.baselinePan, t.baselineTilt
	}

	return pan, tilt
}

func normalizedToDegrees(norm, minDeg, maxDeg float64) float64 {
	return minDeg + ((norm+1)/2)*(maxDeg-minDeg)
}

func degreesToNormalized(deg, minDeg, maxDeg float64) float64 {
	return ((deg - minDeg) / (maxDeg - minDeg) * 2) - 1
}

func radToDeg(rad float64) float64 {
	return rad * 180 / math.Pi
}

func clampFloat(val, minVal, maxVal float64) float64 {
	return math.Max(minVal, math.Min(maxVal, val))
}

func clampMinusOneToOne(val float64) float64 {
	if val > 1 {
		return 1
	}
	if val < -1 {
		return -1
	}
	return val
}

func (t *componentTracker) sendAbsoluteMove(ctx context.Context, pan float64, tilt float64, zoom float64) error {
	onvifPTZClientName := resource.NewName(generic.API, t.onvifPTZClientName)
	onvifPTZClient, err := t.robotClient.ResourceByName(onvifPTZClientName)
	if err != nil {
		return fmt.Errorf("failed to get onvif PTZ client: %w", err)
	}
	_, err = onvifPTZClient.DoCommand(ctx, map[string]interface{}{
		"command":       "absolute-move",
		"pan_position":  pan,
		"tilt_position": tilt,
		"zoom_position": zoom,
		"pan_speed":     t.panSpeed,
		"tilt_speed":    t.tiltSpeed,
		"zoom_speed":    t.zoomSpeed,
	})
	if err != nil {
		return fmt.Errorf("failed to send absolute move: %w", err)
	}
	return nil
}

// Fit: pan = Ax + By + Cz + D
func (t *componentTracker) fitLinear() (panError, tiltError float64) {
	n := len(t.samples)

	// Build matrices for least squares: X * coeffs = Y
	// X is n x 4 matrix: [x, y, z, 1]
	// Y is n x 1 vector: [pan] or [tilt]

	t.panCoeffs = t.fitLinearSingle(func(s TrackingSample) float64 { return s.Pan })
	t.tiltCoeffs = t.fitLinearSingle(func(s TrackingSample) float64 { return s.Tilt })

	// Calculate errors
	var panErrSum, tiltErrSum float64
	for _, s := range t.samples {
		predPan, predTilt := t.predictPanTiltLinear(s.TargetPos)
		panErrSum += math.Abs(predPan - s.Pan)
		tiltErrSum += math.Abs(predTilt - s.Tilt)
	}

	panError = panErrSum / float64(n)
	tiltError = tiltErrSum / float64(n)

	t.logger.Infof("Linear fit complete:")
	t.logger.Infof("  Pan:  %.6f*x + %.6f*y + %.6f*z + %.6f",
		t.panCoeffs[0], t.panCoeffs[1], t.panCoeffs[2], t.panCoeffs[3])
	t.logger.Infof("  Tilt: %.6f*x + %.6f*y + %.6f*z + %.6f",
		t.tiltCoeffs[0], t.tiltCoeffs[1], t.tiltCoeffs[2], t.tiltCoeffs[3])
	t.logger.Infof("  Avg error: pan=%.5f, tilt=%.5f", panError, tiltError)

	return panError, tiltError
}

func (t *componentTracker) fitLinearSingle(getValue func(TrackingSample) float64) [4]float64 {
	// Normal equations: (X^T * X) * coeffs = X^T * Y
	// Build X^T * X (4x4) and X^T * Y (4x1)
	var XtX [4][4]float64
	var XtY [4]float64

	for _, s := range t.samples {
		x := [4]float64{s.TargetPos.X, s.TargetPos.Y, s.TargetPos.Z, 1}
		y := getValue(s)

		for i := 0; i < 4; i++ {
			XtY[i] += x[i] * y
			for j := 0; j < 4; j++ {
				XtX[i][j] += x[i] * x[j]
			}
		}
	}

	// Solve 4x4 system using Gaussian elimination
	return solveLinearSystem4x4(XtX, XtY)
}

func solveLinearSystem4x4(A [4][4]float64, b [4]float64) [4]float64 {
	// Gaussian elimination with partial pivoting
	var aug [4][5]float64
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			aug[i][j] = A[i][j]
		}
		aug[i][4] = b[i]
	}

	// Forward elimination
	for col := 0; col < 4; col++ {
		// Find pivot
		maxRow := col
		for row := col + 1; row < 4; row++ {
			if math.Abs(aug[row][col]) > math.Abs(aug[maxRow][col]) {
				maxRow = row
			}
		}
		aug[col], aug[maxRow] = aug[maxRow], aug[col]

		if math.Abs(aug[col][col]) < 1e-10 {
			continue // Singular
		}

		// Eliminate below
		for row := col + 1; row < 4; row++ {
			factor := aug[row][col] / aug[col][col]
			for j := col; j < 5; j++ {
				aug[row][j] -= factor * aug[col][j]
			}
		}
	}

	// Back substitution
	var result [4]float64
	for i := 3; i >= 0; i-- {
		result[i] = aug[i][4]
		for j := i + 1; j < 4; j++ {
			result[i] -= aug[i][j] * result[j]
		}
		if math.Abs(aug[i][i]) > 1e-10 {
			result[i] /= aug[i][i]
		}
	}

	return result
}

// Fit: pan = Ax² + By² + Cz² + Dxy + Exz + Fyz + Gx + Hy + Iz + J
func (t *componentTracker) fitPolynomial() (panError, tiltError float64) {
	t.panPolyCoeffs = t.fitPolynomialSingle(func(s TrackingSample) float64 { return s.Pan })
	t.tiltPolyCoeffs = t.fitPolynomialSingle(func(s TrackingSample) float64 { return s.Tilt })

	// Calculate errors
	var panErrSum, tiltErrSum float64
	for _, s := range t.samples {
		predPan, predTilt := t.predictPanTiltPolynomial(s.TargetPos)
		panErrSum += math.Abs(predPan - s.Pan)
		tiltErrSum += math.Abs(predTilt - s.Tilt)
	}

	n := float64(len(t.samples))
	panError = panErrSum / n
	tiltError = tiltErrSum / n

	t.logger.Infof("Polynomial fit complete: avg error pan=%.5f, tilt=%.5f", panError, tiltError)

	return panError, tiltError
}

func solveLinearSystem10x10(A [10][10]float64, b [10]float64) [10]float64 {
	// Convert to gonum matrices
	aData := make([]float64, 100)
	for i := 0; i < 10; i++ {
		for j := 0; j < 10; j++ {
			aData[i*10+j] = A[i][j]
		}
	}

	bData := make([]float64, 10)
	copy(bData, b[:])

	aMat := mat.NewDense(10, 10, aData)
	bMat := mat.NewDense(10, 1, bData)

	// Solve using QR decomposition
	var qr mat.QR
	qr.Factorize(aMat)

	var result mat.Dense
	err := qr.SolveTo(&result, false, bMat)
	if err != nil {
		// Return zeros if singular
		return [10]float64{}
	}

	// Extract coefficients
	var coeffs [10]float64
	for i := 0; i < 10; i++ {
		coeffs[i] = result.At(i, 0)
	}

	return coeffs
}

func (t *componentTracker) fitPolynomialSingle(getValue func(TrackingSample) float64) [10]float64 {
	// Features: [x², y², z², xy, xz, yz, x, y, z, 1]
	// Build normal equations
	var XtX [10][10]float64
	var XtY [10]float64

	for _, s := range t.samples {
		x, y, z := s.TargetPos.X, s.TargetPos.Y, s.TargetPos.Z
		features := [10]float64{
			x * x, y * y, z * z,
			x * y, x * z, y * z,
			x, y, z, 1,
		}
		val := getValue(s)

		for i := 0; i < 10; i++ {
			XtY[i] += features[i] * val
			for j := 0; j < 10; j++ {
				XtX[i][j] += features[i] * features[j]
			}
		}
	}

	return solveLinearSystem10x10(XtX, XtY)
}

func (t *componentTracker) predictPanTiltPolynomial(pos r3.Vector) (pan, tilt float64) {
	x, y, z := pos.X, pos.Y, pos.Z
	features := [10]float64{
		x * x, y * y, z * z,
		x * y, x * z, y * z,
		x, y, z, 1,
	}

	pan, tilt = 0, 0
	for i := 0; i < 10; i++ {
		pan += t.panPolyCoeffs[i] * features[i]
		tilt += t.tiltPolyCoeffs[i] * features[i]
	}
	return pan, tilt
}

func (t *componentTracker) predictPanTiltLinear(pos r3.Vector) (pan, tilt float64) {
	pan = t.panCoeffs[0]*pos.X + t.panCoeffs[1]*pos.Y + t.panCoeffs[2]*pos.Z + t.panCoeffs[3]
	tilt = t.tiltCoeffs[0]*pos.X + t.tiltCoeffs[1]*pos.Y + t.tiltCoeffs[2]*pos.Z + t.tiltCoeffs[3]
	return pan, tilt
}

func (t *componentTracker) predictPanTilt(pos r3.Vector) (pan, tilt float64) {
	if t.usePolynomial {
		return t.predictPanTiltPolynomial(pos)
	}
	return t.predictPanTiltLinear(pos)
}

func (t *componentTracker) trackTarget(ctx context.Context) error {
	if !t.isCalibrated {
		return errors.New("not calibrated - run fit-linear or fit-polynomial first")
	}

	targetPose := t.getTargetPose(ctx)
	if targetPose == nil {
		return errors.New("failed to get target pose")
	}
	targetPos := targetPose.Pose().Point()

	pan, tilt := t.predictPanTilt(targetPos)

	// Clamp
	pan = math.Max(-1.0, math.Min(1.0, pan))
	tilt = math.Max(-1.0, math.Min(1.0, tilt))

	return t.sendAbsoluteMove(ctx, pan, tilt, t.baselineZoomX)
}

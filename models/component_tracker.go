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
	"go.viam.com/rdk/spatialmath"
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

	baselinePan               float64
	baselineTilt              float64
	baselineZoomX             float64
	baselineCameraOrientation *spatialmath.OrientationVectorDegrees
	baselineDirection         r3.Vector // World direction at calibration
	panSpeed                  float64
	tiltSpeed                 float64
	zoomSpeed                 float64
	updateRateHz              float64
	panMinDeg                 float64
	panMaxDeg                 float64
	tiltMinDeg                float64
	tiltMaxDeg                float64

	// Fixed camera position
	cameraPosition r3.Vector // e.g., (1600, 0, -600)
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
			"baselineDirection": map[string]interface{}{
				"x": t.baselineDirection.X,
				"y": t.baselineDirection.Y,
				"z": t.baselineDirection.Z,
			}}, nil
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

	// 3. Calculate baseline direction
	t.baselineDirection = r3.Vector{
		X: targetPos.X - t.cameraPosition.X,
		Y: targetPos.Y - t.cameraPosition.Y,
		Z: targetPos.Z - t.cameraPosition.Z,
	}
	t.baselineDirection = t.baselineDirection.Mul(1.0 / t.baselineDirection.Norm())

	t.logger.Infof("Baseline calibration:")
	t.logger.Infof("  Pan: %f, Tilt: %f", t.baselinePan, t.baselineTilt)
	t.logger.Infof("  Direction: (%f, %f, %f)",
		t.baselineDirection.X, t.baselineDirection.Y, t.baselineDirection.Z)

	return nil
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

// Main tracking loop
func (t *componentTracker) trackTarget(ctx context.Context) error {
	// 1. Get target position in world frame
	targetPose := t.getTargetPose(ctx)
	targetPos := targetPose.Pose().Point()

	// 2. Calculate current direction from camera to target
	currentDirection := r3.Vector{
		X: targetPos.X - t.cameraPosition.X,
		Y: targetPos.Y - t.cameraPosition.Y,
		Z: targetPos.Z - t.cameraPosition.Z,
	}

	distance := currentDirection.Norm()
	if distance < 1e-6 {
		return errors.New("target too close to camera")
	}
	currentDirection = currentDirection.Mul(1.0 / distance)

	// 3. Calculate pan/tilt to point at this direction
	pan, tilt := t.directionToPanTilt(currentDirection)
	zoom := t.baselineZoomX

	t.logger.Debugf("Tracking: distance=%f, pan=%f, tilt=%f, zoom=%f", distance, pan, tilt, zoom)

	// 4. Send absolute move command
	return t.sendAbsoluteMove(ctx, pan, tilt, zoom)
}

// Convert world direction to PTZ pan/tilt values
func (t *componentTracker) directionToPanTilt(direction r3.Vector) (pan, tilt float64) {
	// Calculate the angular offset from baseline direction

	// Decompose into horizontal (XY plane) and vertical components
	// Horizontal angle (pan): project onto XY plane
	baselineHorizontal := r3.Vector{
		X: t.baselineDirection.X,
		Y: t.baselineDirection.Y,
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
	baselineElevation := math.Asin(clampMinusOneToOne(t.baselineDirection.Z))
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

	// Apply offsets in degree space so we respect the asymmetric limits.
	panDeg := baselinePanDeg - panOffsetDeg
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

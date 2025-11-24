package models

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"os"
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
	Tag       string
}

type Calibration struct {
	PanCoeffs      [4]float64
	TiltCoeffs     [4]float64
	PanPolyCoeffs  [10]float64
	TiltPolyCoeffs [10]float64
	UsePolynomial  bool
	IsCalibrated   bool
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

	fixedZoomValue float64
	panSpeed       float64
	tiltSpeed      float64
	zoomSpeed      float64
	updateRateHz   float64
	panMinDeg      float64
	panMaxDeg      float64
	tiltMinDeg     float64
	tiltMaxDeg     float64
	samples        []TrackingSample
	calibration    Calibration
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
		t.fixedZoomValue = zoom
		return map[string]interface{}{"status": "success", "zoom": t.fixedZoomValue}, nil

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

		tag := "manual"
		if cmd["tag"] != nil {
			tag = cmd["tag"].(string)
		}

		sample := TrackingSample{
			TargetPos: targetPos,
			Pan:       pan,
			Tilt:      tilt,
			Tag:       tag,
		}
		t.samples = append(t.samples, sample)

		t.logger.Infof("Sample %d: (%.1f, %.1f, %.1f) → pan=%.4f, tilt=%.4f",
			len(t.samples), targetPos.X, targetPos.Y, targetPos.Z, pan, tilt)
		lastSample := t.samples[len(t.samples)-1]
		return map[string]interface{}{
			"sample_number": len(t.samples),
			"target":        map[string]interface{}{"x": lastSample.TargetPos.X, "y": lastSample.TargetPos.Y, "z": lastSample.TargetPos.Z},
			"pan":           lastSample.Pan,
			"tilt":          lastSample.Tilt,
			"tag":           lastSample.Tag,
		}, nil

	case "get-samples":
		return map[string]interface{}{
			"samples": t.samples,
		}, nil
	case "load-samples":
		samples, ok := cmd["samples"].([]TrackingSample)
		if !ok {
			return nil, fmt.Errorf("samples is not a []TrackingSample")
		}
		t.samples = samples
		return map[string]interface{}{
			"status":  "success",
			"samples": len(t.samples),
		}, nil

	case "save-samples-to-file":
		filename := "samples.json"
		if cmd["filename"] != nil {
			filename = cmd["filename"].(string)
		}
		err := t.saveSamplesToJSONFile(filename)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{
			"status":   "success",
			"filename": filename,
		}, nil
	case "load-samples-from-file":
		filename := "samples.json"
		if cmd["filename"] != nil {
			filename = cmd["filename"].(string)
		}
		samples, err := t.loadSamplesFromJSONFile(filename)
		if err != nil {
			return nil, err
		}
		t.samples = samples
		return map[string]interface{}{
			"status":   "success",
			"filename": filename,
			"samples":  len(t.samples),
		}, nil

	case "clear-samples":
		t.samples = nil
		t.calibration.IsCalibrated = false
		return map[string]interface{}{"status": "cleared"}, nil

	case "fit-linear":
		if len(t.samples) < 4 {
			return nil, fmt.Errorf("need at least 4 samples, have %d", len(t.samples))
		}

		panErr, tiltErr := t.fitLinear()
		t.calibration.UsePolynomial = false
		t.calibration.IsCalibrated = true

		return map[string]interface{}{
			"status":         "success",
			"pan_coeffs":     []float64{t.calibration.PanCoeffs[0], t.calibration.PanCoeffs[1], t.calibration.PanCoeffs[2], t.calibration.PanCoeffs[3]},
			"tilt_coeffs":    []float64{t.calibration.TiltCoeffs[0], t.calibration.TiltCoeffs[1], t.calibration.TiltCoeffs[2], t.calibration.TiltCoeffs[3]},
			"pan_error_avg":  panErr,
			"tilt_error_avg": tiltErr,
			"samples_used":   len(t.samples),
		}, nil

	case "fit-polynomial":
		if len(t.samples) < 10 {
			return nil, fmt.Errorf("need at least 10 samples for polynomial fit, have %d", len(t.samples))
		}

		panErr, tiltErr := t.fitPolynomial()
		t.calibration.UsePolynomial = true
		t.calibration.IsCalibrated = true

		return map[string]interface{}{
			"status":         "success",
			"pan_error_avg":  panErr,
			"tilt_error_avg": tiltErr,
			"samples_used":   len(t.samples),
		}, nil
	case "get-calibration":
		return map[string]interface{}{
			"status":           "success",
			"is_calibrated":    t.calibration.IsCalibrated,
			"use_polynomial":   t.calibration.UsePolynomial,
			"pan_coeffs":       []float64{t.calibration.PanCoeffs[0], t.calibration.PanCoeffs[1], t.calibration.PanCoeffs[2], t.calibration.PanCoeffs[3]},
			"tilt_coeffs":      []float64{t.calibration.TiltCoeffs[0], t.calibration.TiltCoeffs[1], t.calibration.TiltCoeffs[2], t.calibration.TiltCoeffs[3]},
			"pan_poly_coeffs":  []float64{t.calibration.PanPolyCoeffs[0], t.calibration.PanPolyCoeffs[1], t.calibration.PanPolyCoeffs[2], t.calibration.PanPolyCoeffs[3], t.calibration.PanPolyCoeffs[4], t.calibration.PanPolyCoeffs[5], t.calibration.PanPolyCoeffs[6], t.calibration.PanPolyCoeffs[7], t.calibration.PanPolyCoeffs[8], t.calibration.PanPolyCoeffs[9]},
			"tilt_poly_coeffs": []float64{t.calibration.TiltPolyCoeffs[0], t.calibration.TiltPolyCoeffs[1], t.calibration.TiltPolyCoeffs[2], t.calibration.TiltPolyCoeffs[3], t.calibration.TiltPolyCoeffs[4], t.calibration.TiltPolyCoeffs[5], t.calibration.TiltPolyCoeffs[6], t.calibration.TiltPolyCoeffs[7], t.calibration.TiltPolyCoeffs[8], t.calibration.TiltPolyCoeffs[9]},
		}, nil
	case "load-calibration":
		panCoeffs, ok := cmd["pan_coeffs"].([]float64)
		if !ok {
			return nil, fmt.Errorf("pan_coeffs is not a []float64")
		}
		tiltCoeffs, ok := cmd["tilt_coeffs"].([]float64)
		if !ok {
			return nil, fmt.Errorf("tilt_coeffs is not a []float64")
		}
		panPolyCoeffs, ok := cmd["pan_poly_coeffs"].([]float64)
		if !ok {
			return nil, fmt.Errorf("pan_poly_coeffs is not a []float64")
		}
		tiltPolyCoeffs, ok := cmd["tilt_poly_coeffs"].([]float64)
		if !ok {
			return nil, fmt.Errorf("tilt_poly_coeffs is not a []float64")
		}
		if len(panCoeffs) != 4 || len(tiltCoeffs) != 4 || len(panPolyCoeffs) != 10 || len(tiltPolyCoeffs) != 10 {
			return nil, fmt.Errorf("pan_coeffs, tilt_coeffs, pan_poly_coeffs, and tilt_poly_coeffs must be 4 and 10 respectively")
		}
		t.calibration.UsePolynomial = cmd["use_polynomial"].(bool)
		t.calibration.PanCoeffs = [4]float64{panCoeffs[0], panCoeffs[1], panCoeffs[2], panCoeffs[3]}
		t.calibration.TiltCoeffs = [4]float64{tiltCoeffs[0], tiltCoeffs[1], tiltCoeffs[2], tiltCoeffs[3]}
		t.calibration.PanPolyCoeffs = [10]float64{panPolyCoeffs[0], panPolyCoeffs[1], panPolyCoeffs[2], panPolyCoeffs[3], panPolyCoeffs[4], panPolyCoeffs[5], panPolyCoeffs[6], panPolyCoeffs[7], panPolyCoeffs[8], panPolyCoeffs[9]}
		t.calibration.TiltPolyCoeffs = [10]float64{tiltPolyCoeffs[0], tiltPolyCoeffs[1], tiltPolyCoeffs[2], tiltPolyCoeffs[3], tiltPolyCoeffs[4], tiltPolyCoeffs[5], tiltPolyCoeffs[6], tiltPolyCoeffs[7], tiltPolyCoeffs[8], tiltPolyCoeffs[9]}
		t.calibration.IsCalibrated = true
		return map[string]interface{}{
			"status": "success",
		}, nil
	case "save-calibration-to-file":
		filename := "calibration.json"
		if cmd["filename"] != nil {
			filename = cmd["filename"].(string)
		}
		err := t.saveCalibrationToJSONFile(filename)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{
			"status":           "success",
			"filename":         filename,
			"use_polynomial":   t.calibration.UsePolynomial,
			"is_calibrated":    t.calibration.IsCalibrated,
			"pan_coeffs":       []float64{t.calibration.PanCoeffs[0], t.calibration.PanCoeffs[1], t.calibration.PanCoeffs[2], t.calibration.PanCoeffs[3]},
			"tilt_coeffs":      []float64{t.calibration.TiltCoeffs[0], t.calibration.TiltCoeffs[1], t.calibration.TiltCoeffs[2], t.calibration.TiltCoeffs[3]},
			"pan_poly_coeffs":  []float64{t.calibration.PanPolyCoeffs[0], t.calibration.PanPolyCoeffs[1], t.calibration.PanPolyCoeffs[2], t.calibration.PanPolyCoeffs[3], t.calibration.PanPolyCoeffs[4], t.calibration.PanPolyCoeffs[5], t.calibration.PanPolyCoeffs[6], t.calibration.PanPolyCoeffs[7], t.calibration.PanPolyCoeffs[8], t.calibration.PanPolyCoeffs[9]},
			"tilt_poly_coeffs": []float64{t.calibration.TiltPolyCoeffs[0], t.calibration.TiltPolyCoeffs[1], t.calibration.TiltPolyCoeffs[2], t.calibration.TiltPolyCoeffs[3], t.calibration.TiltPolyCoeffs[4], t.calibration.TiltPolyCoeffs[5], t.calibration.TiltPolyCoeffs[6], t.calibration.TiltPolyCoeffs[7], t.calibration.TiltPolyCoeffs[8], t.calibration.TiltPolyCoeffs[9]},
		}, nil
	case "load-calibration-from-file":
		filename := "calibration.json"
		if cmd["filename"] != nil {
			filename = cmd["filename"].(string)
		}
		err := t.loadCalibrationFromJSONFile(filename)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{
			"status":           "success",
			"filename":         filename,
			"pan_coeffs":       []float64{t.calibration.PanCoeffs[0], t.calibration.PanCoeffs[1], t.calibration.PanCoeffs[2], t.calibration.PanCoeffs[3]},
			"tilt_coeffs":      []float64{t.calibration.TiltCoeffs[0], t.calibration.TiltCoeffs[1], t.calibration.TiltCoeffs[2], t.calibration.TiltCoeffs[3]},
			"pan_poly_coeffs":  []float64{t.calibration.PanPolyCoeffs[0], t.calibration.PanPolyCoeffs[1], t.calibration.PanPolyCoeffs[2], t.calibration.PanPolyCoeffs[3], t.calibration.PanPolyCoeffs[4], t.calibration.PanPolyCoeffs[5], t.calibration.PanPolyCoeffs[6], t.calibration.PanPolyCoeffs[7], t.calibration.PanPolyCoeffs[8], t.calibration.PanPolyCoeffs[9]},
			"tilt_poly_coeffs": []float64{t.calibration.TiltPolyCoeffs[0], t.calibration.TiltPolyCoeffs[1], t.calibration.TiltPolyCoeffs[2], t.calibration.TiltPolyCoeffs[3], t.calibration.TiltPolyCoeffs[4], t.calibration.TiltPolyCoeffs[5], t.calibration.TiltPolyCoeffs[6], t.calibration.TiltPolyCoeffs[7], t.calibration.TiltPolyCoeffs[8], t.calibration.TiltPolyCoeffs[9]},
			"use_polynomial":   t.calibration.UsePolynomial,
			"is_calibrated":    t.calibration.IsCalibrated,
		}, nil

	default:
		return nil, fmt.Errorf("invalid command: %v", cmd["command"])
	}
}

func (t *componentTracker) saveCalibrationToJSONFile(filename string) error {
	jsonData, err := json.MarshalIndent(t, "", "  ")
	if err != nil {
		return fmt.Errorf("failed to marshal calibration: %v", err)
	}
	err = os.WriteFile(filename, jsonData, 0644)
	if err != nil {
		return fmt.Errorf("failed to write calibration to file: %v", err)
	}
	return nil
}

func (t *componentTracker) loadCalibrationFromJSONFile(filename string) error {
	jsonData, err := os.ReadFile(filename)
	if err != nil {
		return fmt.Errorf("failed to read calibration from file: %v", err)
	}
	return json.Unmarshal(jsonData, &t.calibration)
}
func (t *componentTracker) saveSamplesToJSONFile(filename string) error {
	jsonData, err := json.MarshalIndent(t.samples, "", "  ")
	if err != nil {
		return fmt.Errorf("failed to marshal samples: %v", err)
	}
	err = os.WriteFile(filename, jsonData, 0644)
	if err != nil {
		return fmt.Errorf("failed to write samples to file: %v", err)
	}
	return nil
}
func (t *componentTracker) loadSamplesFromJSONFile(filename string) ([]TrackingSample, error) {
	jsonData, err := os.ReadFile(filename)
	if err != nil {
		return nil, fmt.Errorf("failed to read samples from file: %v", err)
	}
	var samples []TrackingSample
	err = json.Unmarshal(jsonData, &samples)
	if err != nil {
		return nil, fmt.Errorf("failed to unmarshal samples: %v", err)
	}
	return samples, nil
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

	t.calibration.PanCoeffs = t.fitLinearSingle(func(s TrackingSample) float64 { return s.Pan })
	t.calibration.TiltCoeffs = t.fitLinearSingle(func(s TrackingSample) float64 { return s.Tilt })

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
		t.calibration.PanCoeffs[0], t.calibration.PanCoeffs[1], t.calibration.PanCoeffs[2], t.calibration.PanCoeffs[3])
	t.logger.Infof("  Tilt: %.6f*x + %.6f*y + %.6f*z + %.6f",
		t.calibration.TiltCoeffs[0], t.calibration.TiltCoeffs[1], t.calibration.TiltCoeffs[2], t.calibration.TiltCoeffs[3])
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
	t.calibration.PanPolyCoeffs = t.fitPolynomialSingle(func(s TrackingSample) float64 { return s.Pan })
	t.calibration.TiltPolyCoeffs = t.fitPolynomialSingle(func(s TrackingSample) float64 { return s.Tilt })

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
		pan += t.calibration.PanPolyCoeffs[i] * features[i]
		tilt += t.calibration.TiltPolyCoeffs[i] * features[i]
	}
	return pan, tilt
}

func (t *componentTracker) predictPanTiltLinear(pos r3.Vector) (pan, tilt float64) {
	pan = t.calibration.PanCoeffs[0]*pos.X + t.calibration.PanCoeffs[1]*pos.Y + t.calibration.PanCoeffs[2]*pos.Z + t.calibration.PanCoeffs[3]
	tilt = t.calibration.TiltCoeffs[0]*pos.X + t.calibration.TiltCoeffs[1]*pos.Y + t.calibration.TiltCoeffs[2]*pos.Z + t.calibration.TiltCoeffs[3]
	return pan, tilt
}

func (t *componentTracker) predictPanTilt(pos r3.Vector) (pan, tilt float64) {
	if t.calibration.UsePolynomial {
		return t.predictPanTiltPolynomial(pos)
	}
	return t.predictPanTiltLinear(pos)
}

func (t *componentTracker) trackTarget(ctx context.Context) error {
	if !t.calibration.IsCalibrated {
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

	return t.sendAbsoluteMove(ctx, pan, tilt, t.fixedZoomValue)
}

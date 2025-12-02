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
	"go.viam.com/rdk/robot/framesystem"
	genericservice "go.viam.com/rdk/services/generic"
	"gonum.org/v1/gonum/mat"
)

type Ray struct {
	Direction r3.Vector
	Origin    r3.Vector
}

type RayPanTilt struct {
	Pan  float64
	Tilt float64
	Ray  Ray
}

type AbsoluteCalibration struct {
	CameraPos              r3.Vector
	PanPlane               r3.Vector
	Pan0ReferenceDirection r3.Vector // Direction in panPlane that corresponds to pan=0
}

type RayMeasurement struct {
	TargetPos r3.Vector
	Pan       float64
	Tilt      float64
}

type AbsoluteCalibrationRayMeasurements struct {
	RayId        string
	Measurements []RayMeasurement
}

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
	TargetComponentName          string    `json:"target_component_name"`
	PTZCameraName                string    `json:"ptz_camera_name"`
	OnvifPTZClientName           string    `json:"onvif_ptz_client_name"`
	UpdateRateHz                 float64   `json:"update_rate_hz"`
	EnableOnStart                bool      `json:"enable_on_start"`
	ZoomValue                    float64   `json:"zoom_value"`
	PanSpeed                     float64   `json:"pan_speed"`
	TiltSpeed                    float64   `json:"tilt_speed"`
	ZoomSpeed                    float64   `json:"zoom_speed"`
	PanMinSpeedDegreesPerSecond  float64   `json:"pan_min_speed_degrees_per_second"`
	PanMaxSpeedDegreesPerSecond  float64   `json:"pan_max_speed_degrees_per_second"`
	TiltMinSpeedDegreesPerSecond float64   `json:"tilt_min_speed_degrees_per_second"`
	TiltMaxSpeedDegreesPerSecond float64   `json:"tilt_max_speed_degrees_per_second"`
	PanMinDeg                    float64   `json:"pan_min_deg"`
	PanMaxDeg                    float64   `json:"pan_max_deg"`
	TiltMinDeg                   float64   `json:"tilt_min_deg"`
	TiltMaxDeg                   float64   `json:"tilt_max_deg"`
	MinZoomDistanceMM            float64   `json:"min_zoom_distance_mm"`
	MaxZoomDistanceMM            float64   `json:"max_zoom_distance_mm"`
	MinZoomValue                 float64   `json:"min_zoom_value_normalized"`
	MaxZoomValue                 float64   `json:"max_zoom_value_normalized"`
	Deadzone                     float64   `json:"deadzone"`
	TrackingMode                 string    `json:"tracking_mode"`
	AbsoluteCalibrationPanPlane  r3.Vector `json:"absolute_calibration_pan_plane"`
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
	if cfg.ZoomValue < 0 || cfg.ZoomValue > 1 {
		return nil, nil, errors.New("zoom_value must be greater than or equal to 0 and less than or equal to 1")
	}
	if cfg.PanSpeed < 0 || cfg.PanSpeed > 1 {
		return nil, nil, errors.New("pan_speed must be greater than or equal to 0 and less than or equal to 1")
	}
	if cfg.TiltSpeed < 0 || cfg.TiltSpeed > 1 {
		return nil, nil, errors.New("tilt_speed must be greater than or equal to 0 and less than or equal to 1")
	}
	if cfg.ZoomSpeed < 0 || cfg.ZoomSpeed > 1 {
		return nil, nil, errors.New("zoom_speed must be greater than or equal to 0 and less than or equal to 1")
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
	if cfg.PanMinSpeedDegreesPerSecond < 0 {
		return nil, nil, errors.New("pan_min_speed_degrees_per_second must be greater than or equal to 0")
	}
	if cfg.TiltMinSpeedDegreesPerSecond < 0 {
		return nil, nil, errors.New("tilt_min_speed_degrees_per_second must be greater than or equal to 0")
	}
	if cfg.PanMaxSpeedDegreesPerSecond < 0 {
		return nil, nil, errors.New("pan_max_speed_degrees_per_second must be greater than or equal to 0")
	}
	if cfg.TiltMaxSpeedDegreesPerSecond < 0 {
		return nil, nil, errors.New("tilt_max_speed_degrees_per_second must be greater than or equal to 0")
	}
	if cfg.PanMaxSpeedDegreesPerSecond < cfg.PanMinSpeedDegreesPerSecond {
		return nil, nil, errors.New("pan_max_speed_degrees_per_second must be greater than pan_min_speed_degrees_per_second")
	}
	if cfg.TiltMaxSpeedDegreesPerSecond < cfg.TiltMinSpeedDegreesPerSecond {
		return nil, nil, errors.New("tilt_max_speed_degrees_per_second must be greater than tilt_min_speed_degrees_per_second")
	}
	if cfg.Deadzone < 0 || cfg.Deadzone > 1 {
		return nil, nil, errors.New("deadzone must be greater than or equal to 0 and less than or equal to 1 (normalized range)")
	}
	if cfg.MinZoomDistanceMM < 0 {
		return nil, nil, errors.New("min_zoom_distance_mm must be greater than or equal to 0")
	}
	if cfg.MaxZoomDistanceMM < 0 {
		return nil, nil, errors.New("max_zoom_distance_mm must be greater than or equal to 0")
	}
	if cfg.MinZoomValue < 0 || cfg.MaxZoomValue < 0 {
		return nil, nil, errors.New("min_zoom_value_normalized and max_zoom_value_normalized must be greater than or equal to 0")
	}
	if cfg.MinZoomValue >= cfg.MaxZoomValue {
		return nil, nil, errors.New("min_zoom_value_normalized must be less than or equal to max_zoom_value_normalized")
	}
	// possible values: "polynomial-fit", "absolute-position", default: "polynomial-fit"
	if cfg.TrackingMode == "" {
		cfg.TrackingMode = "polynomial-fit"
	}
	if cfg.TrackingMode != "polynomial-fit" && cfg.TrackingMode != "absolute-position" {
		return nil, nil, errors.New("tracking_mode must be either 'polynomial-fit' or 'absolute-position'")
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
	PanPolyCoeffs  [10]float64
	TiltPolyCoeffs [10]float64
	IsCalibrated   bool
}
type componentTracker struct {
	resource.AlwaysRebuild
	resource.TriviallyReconfigurable

	name resource.Name

	logger logging.Logger
	cfg    *Config

	cancelCtx  context.Context
	cancelFunc func()
	running    bool

	robotClient         robot.Robot
	frameSystemService  framesystem.Service
	targetComponentName string
	onvifPTZClientName  string

	zoomValue                          float64
	panSpeed                           float64
	tiltSpeed                          float64
	zoomSpeed                          float64
	updateRateHz                       float64
	panMinDeg                          float64
	panMaxDeg                          float64
	tiltMinDeg                         float64
	tiltMaxDeg                         float64
	panMinSpeedDegreesPerSecond        float64
	panMaxSpeedDegreesPerSecond        float64
	tiltMinSpeedDegreesPerSecond       float64
	tiltMaxSpeedDegreesPerSecond       float64
	samples                            []TrackingSample
	calibration                        Calibration
	lastSentPan                        float64
	lastSentTilt                       float64
	lastSentZoom                       float64
	deadzone                           float64
	minZoomDistance                    float64
	maxZoomDistance                    float64
	minZoomValue                       float64
	maxZoomValue                       float64
	trackingMode                       string
	absoluteCalibrationPanPlane        r3.Vector
	absoluteCalibrationPan0Reference   r3.Vector                                     // Direction in panPlane that corresponds to pan=0
	absoluteCalibrationRayMeasurements map[string]AbsoluteCalibrationRayMeasurements // rayId -> measurements
}

// Close implements resource.Resource.
func (s *componentTracker) Close(ctx context.Context) error {
	s.logger.Debug("Closing component tracker")
	// Stop tracking loop if running
	if s.running {
		s.running = false
		if s.cancelFunc != nil {
			s.cancelFunc()
		}
	}
	return nil
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
	// Update the config struct so TrackingMode and other fields are available
	s.cfg = conf
	s.targetComponentName = conf.TargetComponentName
	s.onvifPTZClientName = conf.OnvifPTZClientName
	s.updateRateHz = conf.UpdateRateHz
	s.zoomValue = conf.ZoomValue
	s.panSpeed = conf.PanSpeed
	s.tiltSpeed = conf.TiltSpeed
	s.zoomSpeed = conf.ZoomSpeed
	s.panMinSpeedDegreesPerSecond = conf.PanMinSpeedDegreesPerSecond
	s.panMaxSpeedDegreesPerSecond = conf.PanMaxSpeedDegreesPerSecond
	s.tiltMinSpeedDegreesPerSecond = conf.TiltMinSpeedDegreesPerSecond
	s.tiltMaxSpeedDegreesPerSecond = conf.TiltMaxSpeedDegreesPerSecond
	s.panMinDeg = conf.PanMinDeg
	s.panMaxDeg = conf.PanMaxDeg
	s.tiltMinDeg = conf.TiltMinDeg
	s.tiltMaxDeg = conf.TiltMaxDeg
	s.deadzone = conf.Deadzone
	s.minZoomDistance = conf.MinZoomDistanceMM
	s.maxZoomDistance = conf.MaxZoomDistanceMM
	s.minZoomValue = conf.MinZoomValue
	s.maxZoomValue = conf.MaxZoomValue
	s.trackingMode = conf.TrackingMode
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

	frameSystemService, err := framesystem.FromDependencies(deps)
	if err != nil {
		cancelFunc()
		return nil, fmt.Errorf("failed to get frame system service: %w", err)
	}

	s := &componentTracker{
		name:                         name,
		logger:                       logger,
		cfg:                          conf,
		cancelCtx:                    cancelCtx,
		cancelFunc:                   cancelFunc,
		robotClient:                  robotClient,
		frameSystemService:           frameSystemService,
		targetComponentName:          conf.TargetComponentName,
		onvifPTZClientName:           conf.OnvifPTZClientName,
		panSpeed:                     conf.PanSpeed,
		tiltSpeed:                    conf.TiltSpeed,
		zoomSpeed:                    conf.ZoomSpeed,
		panMinSpeedDegreesPerSecond:  conf.PanMinSpeedDegreesPerSecond,
		panMaxSpeedDegreesPerSecond:  conf.PanMaxSpeedDegreesPerSecond,
		tiltMinSpeedDegreesPerSecond: conf.TiltMinSpeedDegreesPerSecond,
		tiltMaxSpeedDegreesPerSecond: conf.TiltMaxSpeedDegreesPerSecond,
		updateRateHz:                 conf.UpdateRateHz,
		panMinDeg:                    conf.PanMinDeg,
		panMaxDeg:                    conf.PanMaxDeg,
		tiltMinDeg:                   conf.TiltMinDeg,
		tiltMaxDeg:                   conf.TiltMaxDeg,
		deadzone:                     conf.Deadzone,
		minZoomDistance:              conf.MinZoomDistanceMM,
		maxZoomDistance:              conf.MaxZoomDistanceMM,
		minZoomValue:                 conf.MinZoomValue,
		maxZoomValue:                 conf.MaxZoomValue,
		samples:                      nil,
		calibration: Calibration{
			PanPolyCoeffs:  [10]float64{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			TiltPolyCoeffs: [10]float64{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			IsCalibrated:   false,
		},
		trackingMode:                       conf.TrackingMode,
		absoluteCalibrationPanPlane:        conf.AbsoluteCalibrationPanPlane,
		absoluteCalibrationPan0Reference:   r3.Vector{X: 1, Y: 0, Z: 0}, // Default to world +X
		absoluteCalibrationRayMeasurements: make(map[string]AbsoluteCalibrationRayMeasurements),
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
		t.zoomValue = zoom
		return map[string]interface{}{"status": "success", "zoom": t.zoomValue}, nil

	case "set-deadzone":
		deadzone, ok := cmd["deadzone"].(float64)
		if !ok {
			return nil, fmt.Errorf("deadzone is not a float")
		}
		t.deadzone = deadzone
		return map[string]interface{}{"status": "success", "deadzone": t.deadzone}, nil

	case "add-calibration-sample":
		// Get current target position from arm
		targetPose, err := t.getTargetPose(ctx)
		if err != nil {
			return nil, err
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
		t.fitPolynomial()
		return map[string]interface{}{
			"sample_number": len(t.samples),
			"target":        map[string]interface{}{"x": lastSample.TargetPos.X, "y": lastSample.TargetPos.Y, "z": lastSample.TargetPos.Z},
			"pan":           lastSample.Pan,
			"tilt":          lastSample.Tilt,
			"tag":           lastSample.Tag,
		}, nil

	case "get-calibration-samples":
		return map[string]interface{}{
			"calibration-samples": t.samples,
		}, nil
	case "load-calibration-samples":
		samplesRaw, ok := cmd["calibration-samples"]
		if !ok {
			return nil, fmt.Errorf("samples field is required")
		}

		samplesArray, ok := samplesRaw.([]interface{})
		if !ok {
			return nil, fmt.Errorf("samples must be an array")
		}

		var loadedSamples []TrackingSample
		for i, sampleRaw := range samplesArray {
			sampleMap, ok := sampleRaw.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("sample %d is not a map", i)
			}

			// Parse TargetPos
			targetPosRaw, ok := sampleMap["TargetPos"]
			if !ok {
				return nil, fmt.Errorf("sample %d missing TargetPos", i)
			}
			targetPosMap, ok := targetPosRaw.(map[string]interface{})
			if !ok {
				return nil, fmt.Errorf("sample %d TargetPos is not a map", i)
			}

			targetX, ok := targetPosMap["X"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d TargetPos.X is not a float64", i)
			}
			targetY, ok := targetPosMap["Y"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d TargetPos.Y is not a float64", i)
			}
			targetZ, ok := targetPosMap["Z"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d TargetPos.Z is not a float64", i)
			}

			// Parse Pan
			pan, ok := sampleMap["Pan"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d Pan is not a float64", i)
			}

			// Parse Tilt
			tilt, ok := sampleMap["Tilt"].(float64)
			if !ok {
				return nil, fmt.Errorf("sample %d Tilt is not a float64", i)
			}

			// Parse Tag (optional)
			tag := "manual"
			if tagRaw, ok := sampleMap["Tag"]; ok {
				if tagStr, ok := tagRaw.(string); ok {
					tag = tagStr
				}
			}

			sample := TrackingSample{
				TargetPos: r3.Vector{X: targetX, Y: targetY, Z: targetZ},
				Pan:       pan,
				Tilt:      tilt,
				Tag:       tag,
			}
			loadedSamples = append(loadedSamples, sample)
		}

		t.samples = loadedSamples
		t.logger.Infof("Loaded %d samples", len(t.samples))

		t.fitPolynomial()

		return map[string]interface{}{
			"status":  "success",
			"samples": len(t.samples),
		}, nil

	case "save-calibration-samples-to-file":
		filename := "calibration-samples.json"
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
	case "load-calibration-samples-from-file":
		filename := "calibration-samples.json"
		if cmd["filename"] != nil {
			filename = cmd["filename"].(string)
		}
		samples, err := t.loadSamplesFromJSONFile(filename)
		if err != nil {
			return nil, err
		}
		t.samples = samples
		t.fitPolynomial()
		return map[string]interface{}{
			"status":   "success",
			"filename": filename,
			"samples":  len(t.samples),
		}, nil

	case "clear-calibration-samples":
		t.samples = nil
		t.calibration.IsCalibrated = false
		return map[string]interface{}{"status": "cleared"}, nil

	case "remove-calibration-sample":
		index, ok := cmd["index"].(int)
		if !ok {
			return nil, fmt.Errorf("index is not an int")
		}
		if index < 0 || index >= len(t.samples) {
			return nil, fmt.Errorf("index out of range")
		}
		t.samples = append(t.samples[:index], t.samples[index+1:]...)
		t.fitPolynomial()
		return map[string]interface{}{"status": "removed", "index": index}, nil

	case "remove-last-calibration-sample":
		if len(t.samples) == 0 {
			return nil, fmt.Errorf("no samples to remove")
		}
		t.samples = t.samples[:len(t.samples)-1]
		t.fitPolynomial()
		return map[string]interface{}{"status": "removed", "index": len(t.samples)}, nil

	case "clear-calibration":
		t.calibration.IsCalibrated = false
		t.fitPolynomial()
		return map[string]interface{}{"status": "cleared"}, nil

	case "fit-polynomial-calibration":
		if len(t.samples) < 10 {
			return nil, fmt.Errorf("need at least 10 samples for polynomial fit, have %d", len(t.samples))
		}

		panErr, tiltErr := t.fitPolynomial()

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
			"pan_poly_coeffs":  []float64{t.calibration.PanPolyCoeffs[0], t.calibration.PanPolyCoeffs[1], t.calibration.PanPolyCoeffs[2], t.calibration.PanPolyCoeffs[3], t.calibration.PanPolyCoeffs[4], t.calibration.PanPolyCoeffs[5], t.calibration.PanPolyCoeffs[6], t.calibration.PanPolyCoeffs[7], t.calibration.PanPolyCoeffs[8], t.calibration.PanPolyCoeffs[9]},
			"tilt_poly_coeffs": []float64{t.calibration.TiltPolyCoeffs[0], t.calibration.TiltPolyCoeffs[1], t.calibration.TiltPolyCoeffs[2], t.calibration.TiltPolyCoeffs[3], t.calibration.TiltPolyCoeffs[4], t.calibration.TiltPolyCoeffs[5], t.calibration.TiltPolyCoeffs[6], t.calibration.TiltPolyCoeffs[7], t.calibration.TiltPolyCoeffs[8], t.calibration.TiltPolyCoeffs[9]},
		}, nil
	case "load-calibration":
		panPolyCoeffs, ok := cmd["pan_poly_coeffs"].([]float64)
		if !ok {
			return nil, fmt.Errorf("pan_poly_coeffs is not a []float64")
		}
		tiltPolyCoeffs, ok := cmd["tilt_poly_coeffs"].([]float64)
		if !ok {
			return nil, fmt.Errorf("tilt_poly_coeffs is not a []float64")
		}
		if len(panPolyCoeffs) != 10 || len(tiltPolyCoeffs) != 10 {
			return nil, fmt.Errorf("pan_poly_coeffs and tilt_poly_coeffs must be 10")
		}
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
			"is_calibrated":    t.calibration.IsCalibrated,
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
			"pan_poly_coeffs":  []float64{t.calibration.PanPolyCoeffs[0], t.calibration.PanPolyCoeffs[1], t.calibration.PanPolyCoeffs[2], t.calibration.PanPolyCoeffs[3], t.calibration.PanPolyCoeffs[4], t.calibration.PanPolyCoeffs[5], t.calibration.PanPolyCoeffs[6], t.calibration.PanPolyCoeffs[7], t.calibration.PanPolyCoeffs[8], t.calibration.PanPolyCoeffs[9]},
			"tilt_poly_coeffs": []float64{t.calibration.TiltPolyCoeffs[0], t.calibration.TiltPolyCoeffs[1], t.calibration.TiltPolyCoeffs[2], t.calibration.TiltPolyCoeffs[3], t.calibration.TiltPolyCoeffs[4], t.calibration.TiltPolyCoeffs[5], t.calibration.TiltPolyCoeffs[6], t.calibration.TiltPolyCoeffs[7], t.calibration.TiltPolyCoeffs[8], t.calibration.TiltPolyCoeffs[9]},
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

func (t *componentTracker) getTargetPose(ctx context.Context) (*referenceframe.PoseInFrame, error) {
	fsc, err := t.frameSystemService.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil, err
	}
	targetFramePart := touch.FindPart(fsc, t.targetComponentName)
	if targetFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.targetComponentName)
		return nil, fmt.Errorf("can't find frame for %v", t.targetComponentName)
	}
	targetPose, err := t.frameSystemService.GetPose(ctx, targetFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose: %v", err)
		return nil, err
	}

	return targetPose, nil
}

func (t *componentTracker) getCameraPose(ctx context.Context) (*referenceframe.PoseInFrame, error) {
	fsc, err := t.frameSystemService.FrameSystemConfig(ctx)
	if err != nil {
		t.logger.Error("Failed to get frame system config: %v", err)
		return nil, err
	}
	cameraFramePart := touch.FindPart(fsc, t.cfg.PTZCameraName)
	if cameraFramePart == nil {
		t.logger.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
		return nil, fmt.Errorf("can't find frame for %v", t.cfg.PTZCameraName)
	}
	cameraPose, err := t.frameSystemService.GetPose(ctx, cameraFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose: %v", err)
		return nil, err
	}
	return cameraPose, nil
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
		return 0, 0, 0, fmt.Errorf("PTZ move pan tilt is not a string")
	}
	if movePanTilt != "IDLE" {
		t.logger.Debugf("PTZ pan/tilt is moving (status: %s), reading position anyway", movePanTilt)
	}
	moveZoom, ok := moveStatus["zoom"].(string)
	if !ok {
		t.logger.Errorf("PTZ move zoom is not a string")
		return 0, 0, 0, fmt.Errorf("PTZ move zoom is not a string")
	}
	if moveZoom != "IDLE" {
		t.logger.Debugf("PTZ zoom is moving (status: %s), reading position anyway", moveZoom)
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

	panSpeed := t.panSpeed
	tiltSpeed := t.tiltSpeed
	zoomSpeed := t.zoomSpeed

	currentPan, currentTilt, currentZoom, err := t.getCameraCurrentPTZStatus(ctx)
	if err != nil {
		return fmt.Errorf("failed to get current PTZ status: %w", err)
	}

	// Check if we're sending the same absolute position as last time (idempotent check)
	panDeltaFromLastSent := math.Abs(pan - t.lastSentPan)
	tiltDeltaFromLastSent := math.Abs(tilt - t.lastSentTilt)
	zoomDeltaFromLastSent := math.Abs(zoom - t.lastSentZoom)

	// Threshold: skip move if we're sending the same absolute position (< 0.001 normalized)
	const samePositionThresholdNormalized = 0.001

	if panDeltaFromLastSent < samePositionThresholdNormalized &&
		tiltDeltaFromLastSent < samePositionThresholdNormalized &&
		zoomDeltaFromLastSent < samePositionThresholdNormalized {
		t.logger.Debugf("Skipping move - same absolute position as last sent: pan=%.3f, tilt=%.3f, zoom=%.3f", pan, tilt, zoom)
		return nil
	}

	// Calculate deltas to current position for speed calculation
	panDeltaNormalized := math.Abs(pan - currentPan)
	tiltDeltaNormalized := math.Abs(tilt - currentTilt)
	zoomDeltaNormalized := math.Abs(zoom - currentZoom)
	// Deadzone check: skip move if target is within deadzone of current position
	// Deadzone is in normalized [0, 1] range (e.g., 0.01 = 1% of the full range)
	if t.deadzone > 0 {
		if panDeltaNormalized < t.deadzone && tiltDeltaNormalized < t.deadzone {
			t.logger.Debugf("Skipping move - within deadzone: pan_delta=%.4f (deadzone=%.4f), tilt_delta=%.4f (deadzone=%.4f)", panDeltaNormalized, t.deadzone, tiltDeltaNormalized, t.deadzone)
			// Update last sent position even though we skipped
			t.lastSentPan = pan
			t.lastSentTilt = tilt
			t.lastSentZoom = zoom
			return nil
		}
	}

	// Threshold: skip move if delta to current position is very small (< 0.001 normalized)
	const moveThresholdNormalized = 0.001

	if panDeltaNormalized < moveThresholdNormalized && tiltDeltaNormalized < moveThresholdNormalized {
		t.logger.Debugf("Skipping move - already at target: pan=%.3f (delta=%.4f), tilt=%.3f (delta=%.4f)", pan, panDeltaNormalized, tilt, tiltDeltaNormalized)
		// Update last sent position even though we skipped
		t.lastSentPan = pan
		t.lastSentTilt = tilt
		t.lastSentZoom = zoom
		return nil
	}

	// Variables to store speed calculation details for logging
	var panDeltaDegrees, panRequiredSpeedDegreesPerSecond float64
	var tiltDeltaDegrees, tiltRequiredSpeedDegreesPerSecond float64

	// If the user has not set a fixed pan speed, calculate it based on the required angle change and available time
	if panSpeed == 0 {
		// Calculate speed based on required angle change and available time
		// panDeltaNormalized already calculated above
		// 2. Convert to degrees: normalized [-1, 1] maps to [panMinDeg, panMaxDeg]
		panRangeDegrees := t.panMaxDeg - t.panMinDeg
		panDeltaDegrees = panDeltaNormalized * panRangeDegrees / 2.0
		// 3. Calculate time available (period between updates)
		timeAvailableSeconds := 1.0 / t.updateRateHz
		// 4. Calculate required speed in degrees/second
		panRequiredSpeedDegreesPerSecond = panDeltaDegrees / timeAvailableSeconds
		// 5. Map to normalized speed [0, 1] based on min/max speed configuration
		// If required speed is less than min, use min. If greater than max, use max (clamped to 1.0)
		if panRequiredSpeedDegreesPerSecond <= t.panMinSpeedDegreesPerSecond {
			panSpeed = 0.0
		} else if panRequiredSpeedDegreesPerSecond >= t.panMaxSpeedDegreesPerSecond {
			panSpeed = 1.0
		} else {
			// Linear interpolation between min and max speeds
			speedRange := t.panMaxSpeedDegreesPerSecond - t.panMinSpeedDegreesPerSecond
			if speedRange > 0 {
				panSpeed = (panRequiredSpeedDegreesPerSecond - t.panMinSpeedDegreesPerSecond) / speedRange
			} else {
				panSpeed = 0.0
			}
		}
		// Clamp to [0.0, 1.0] as ONVIF expects normalized speeds
		panSpeed = math.Max(0.0, math.Min(1.0, panSpeed))
	}
	// If the user has not set a fixed tilt speed, calculate it based on the required angle change and available time
	if tiltSpeed == 0 {
		// Calculate speed based on required angle change and available time
		// tiltDeltaNormalized already calculated above
		// 2. Convert to degrees: normalized [-1, 1] maps to [tiltMinDeg, tiltMaxDeg]
		tiltRangeDegrees := t.tiltMaxDeg - t.tiltMinDeg
		tiltDeltaDegrees = tiltDeltaNormalized * tiltRangeDegrees / 2.0
		// 3. Calculate time available (period between updates)
		timeAvailableSeconds := 1.0 / t.updateRateHz
		// 4. Calculate required speed in degrees/second
		tiltRequiredSpeedDegreesPerSecond = tiltDeltaDegrees / timeAvailableSeconds
		// 5. Map to normalized speed [0, 1] based on min/max speed configuration
		if tiltRequiredSpeedDegreesPerSecond <= t.tiltMinSpeedDegreesPerSecond {
			tiltSpeed = 0.0
		} else if tiltRequiredSpeedDegreesPerSecond >= t.tiltMaxSpeedDegreesPerSecond {
			tiltSpeed = 1.0
		} else {
			// Linear interpolation between min and max speeds
			speedRange := t.tiltMaxSpeedDegreesPerSecond - t.tiltMinSpeedDegreesPerSecond
			if speedRange > 0 {
				tiltSpeed = (tiltRequiredSpeedDegreesPerSecond - t.tiltMinSpeedDegreesPerSecond) / speedRange
			} else {
				tiltSpeed = 0.0
			}
		}
		// Clamp to [0.0, 1.0] as ONVIF expects normalized speeds
		tiltSpeed = math.Max(0.0, math.Min(1.0, tiltSpeed))
	}
	// If the user has not set a fixed zoom speed, we will simply use the max speed
	if t.zoomSpeed == 0 {
		if zoomDeltaNormalized < t.deadzone {
			zoomSpeed = 0.0
		} else {
			zoomSpeed = 1
		}
	}
	// Ensure zoom speed is also clamped
	zoomSpeed = math.Max(0.0, math.Min(1.0, zoomSpeed))

	// Skip move if both pan and tilt speeds are effectively zero (very small movements)
	// Threshold of 0.01 means we skip movements requiring less than 1% of max speed
	// Also skip if the maximum of pan/tilt speeds is very small (< 0.02)
	const minEffectiveSpeed = 0.01
	const minMaxSpeed = 0.02
	maxSpeed := math.Max(panSpeed, tiltSpeed)
	if (panSpeed < minEffectiveSpeed && tiltSpeed < minEffectiveSpeed || maxSpeed < minMaxSpeed) &&
		math.Abs(zoom-t.lastSentZoom) < samePositionThresholdNormalized {
		t.logger.Debugf("Sending absolute move: Skipping move - speeds too small: pan_speed=%.4f, tilt_speed=%.4f, max_speed=%.4f", panSpeed, tiltSpeed, maxSpeed)
		// Update last sent position even though we skipped
		t.lastSentPan = pan
		t.lastSentTilt = tilt
		t.lastSentZoom = zoom
		return nil
	}

	// Comprehensive debug log with all information
	t.logger.Debugf("Sending absolute move: target pan=%.3f (current=%.3f, delta=%.4f norm/%.2f deg, req_speed=%.2f deg/s, speed_range=[%.2f,%.2f] deg/s, norm_speed=%.3f), target tilt=%.3f (current=%.3f, delta=%.4f norm/%.2f deg, req_speed=%.2f deg/s, speed_range=[%.2f,%.2f] deg/s, norm_speed=%.3f), zoom=%.3f, speeds: pan=%.3f tilt=%.3f zoom=%.3f",
		pan, currentPan, panDeltaNormalized, panDeltaDegrees, panRequiredSpeedDegreesPerSecond, t.panMinSpeedDegreesPerSecond, t.panMaxSpeedDegreesPerSecond, panSpeed,
		tilt, currentTilt, tiltDeltaNormalized, tiltDeltaDegrees, tiltRequiredSpeedDegreesPerSecond, t.tiltMinSpeedDegreesPerSecond, t.tiltMaxSpeedDegreesPerSecond, tiltSpeed,
		zoom, panSpeed, tiltSpeed, zoomSpeed)
	_, err = onvifPTZClient.DoCommand(ctx, map[string]interface{}{
		"command":       "absolute-move",
		"pan_position":  pan,
		"tilt_position": tilt,
		"zoom_position": zoom,
		"pan_speed":     panSpeed,
		"tilt_speed":    tiltSpeed,
		"zoom_speed":    zoomSpeed,
	})
	if err != nil {
		return fmt.Errorf("failed to send absolute move: %w", err)
	}

	// Update last sent positions (absolute positions are idempotent)
	t.lastSentPan = pan
	t.lastSentTilt = tilt
	t.lastSentZoom = zoom

	return nil
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

	t.calibration.IsCalibrated = true
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

func (t *componentTracker) predictPanTiltZoom(ctx context.Context, pos r3.Vector) (pan, tilt, zoom float64, err error) {
	cameraPose, err := t.getCameraPose(ctx)
	if err != nil {
		return 0, 0, 0, fmt.Errorf("failed to get camera pose: %w", err)
	}
	cameraPos := cameraPose.Pose().Point()
	switch t.cfg.TrackingMode {
	case "polynomial-fit":
		pan, tilt = t.predictPanTiltPolynomial(pos)
	case "absolute-position":
		pan, tilt = t.predictPanTiltAbsolute(pos, cameraPos, t.absoluteCalibrationPanPlane, t.absoluteCalibrationPan0Reference)
		return 0, 0, 0, errors.New("invalid tracking mode: " + t.cfg.TrackingMode)
	}
	zoom = t.calculateZoom(ctx, pos, cameraPos)
	return pan, tilt, zoom, nil
}
func (t *componentTracker) calculateZoom(ctx context.Context, pos r3.Vector, cameraPos r3.Vector) float64 {
	distance := pos.Distance(cameraPos)
	t.logger.Debugf("calculateZoom: calculating zoom for distance: %.2f mm, minZoomDistance: %.2f mm, maxZoomDistance: %.2f mm, minZoomValue: %.2f, maxZoomValue: %.2f\n", distance, t.minZoomDistance, t.maxZoomDistance, t.minZoomValue, t.maxZoomValue)

	// Clamp distance to [minZoomDistance, maxZoomDistance]
	// Closer = zoomed out (minZoomValue), farther = zoomed in (maxZoomValue)
	if distance <= t.minZoomDistance {
		t.logger.Debugf("calculateZoom: closest: zoomed out, zoom: %.2f", t.minZoomValue)
		return t.minZoomValue // Closest: zoomed out
	}
	if distance >= t.maxZoomDistance {
		t.logger.Debugf("calculateZoom: farthest: zoomed in, zoom: %.2f", t.maxZoomValue)
		return t.maxZoomValue // Farthest: zoomed in
	}

	// Linear interpolation between minZoomDistance and maxZoomDistance
	// Normalized distance: 0 at minZoomDistance, 1 at maxZoomDistance
	// Zoom: minZoomValue at minZoomDistance (closest), maxZoomValue at maxZoomDistance (farthest)
	if (t.maxZoomDistance - t.minZoomDistance) > 0 {
		normalizedDistance := (distance - t.minZoomDistance) / (t.maxZoomDistance - t.minZoomDistance)
		zoom := t.minZoomValue + (t.maxZoomValue-t.minZoomValue)*normalizedDistance
		t.logger.Debugf("calculateZoom: zoom: %.2f normalizedDistance: %.2f, distance: %.2f mm", zoom, normalizedDistance, distance)
		return zoom
	} else {
		t.logger.Debugf("calculateZoom: minZoomDistance == maxZoomDistance, zoom: %.2f", t.minZoomValue)
		return t.minZoomValue
	}
}

func (t *componentTracker) trackTarget(ctx context.Context) error {
	// Only require calibration for polynomial-fit mode
	if t.trackingMode == "polynomial-fit" && !t.calibration.IsCalibrated {
		return errors.New("not calibrated - run fit-polynomial-calibration first")
	}

	targetPose, err := t.getTargetPose(ctx)
	if err != nil {
		return err
	}
	targetPos := targetPose.Pose().Point()

	pan, tilt, zoom, err := t.predictPanTiltZoom(ctx, targetPos)
	if err != nil {
		t.logger.Errorf("Failed to predict pan/tilt/zoom: %v", err)
		return err
	}
	t.logger.Debugf("Predicted pan: %.1f, tilt: %.1f, zoom: %.1f", pan, tilt, zoom)

	// Clamp
	pan = math.Max(-1.0, math.Min(1.0, pan))
	tilt = math.Max(-1.0, math.Min(1.0, tilt))

	return t.sendAbsoluteMove(ctx, pan, tilt, zoom)
}

func (t *componentTracker) addAbsoluteCalibrationMeasurement(targetPos r3.Vector, pan float64, tilt float64, rayId string) error {
	measurements, ok := t.absoluteCalibrationRayMeasurements[rayId]
	if !ok {
		measurements = AbsoluteCalibrationRayMeasurements{
			RayId:        rayId,
			Measurements: []RayMeasurement{},
		}
	}
	measurements.Measurements = append(measurements.Measurements, RayMeasurement{
		TargetPos: targetPos,
		Pan:       pan,
		Tilt:      tilt,
	})
	t.absoluteCalibrationRayMeasurements[rayId] = measurements
	return nil
}

func (t *componentTracker) getAbsoluteCalibrationMeasurements(rayId string) (AbsoluteCalibrationRayMeasurements, error) {
	measurements, ok := t.absoluteCalibrationRayMeasurements[rayId]
	if !ok {
		return AbsoluteCalibrationRayMeasurements{}, errors.New("no measurements found for rayId: " + rayId)
	}
	return measurements, nil
}

func calculateAverage(values []float64) float64 {
	sum := 0.0
	for _, value := range values {
		sum += value
	}
	return sum / float64(len(values))
}
func calculateCentroid(points []r3.Vector) r3.Vector {

	XValues := make([]float64, 0, len(points))
	YValues := make([]float64, 0, len(points))
	ZValues := make([]float64, 0, len(points))

	for _, p := range points {
		XValues = append(XValues, p.X)
		YValues = append(YValues, p.Y)
		ZValues = append(ZValues, p.Z)
	}
	centroid := r3.Vector{
		X: calculateAverage(XValues),
		Y: calculateAverage(YValues),
		Z: calculateAverage(ZValues),
	}
	return centroid
}
func calculateRayFromMeasurements(measurements AbsoluteCalibrationRayMeasurements) (rayPanTilt RayPanTilt, err error) {
	rayMeasurements := measurements.Measurements
	targetPositions := make([]r3.Vector, len(rayMeasurements))
	for i, measurement := range rayMeasurements {
		targetPositions[i] = measurement.TargetPos
	}
	// The Pan and Tilt values of all the measurements should be the same as the camera should be pointing at the target positions with the same pan and tilt values
	// We allow for a small error in the Pan and Tilt values as the camera may return slightly different values due to noise or other factors
	// We will use the average of the Pan and Tilt values as the camera position
	panValues := make([]float64, len(rayMeasurements))
	tiltValues := make([]float64, len(rayMeasurements))
	for i, measurement := range rayMeasurements {
		panValues[i] = measurement.Pan
		tiltValues[i] = measurement.Tilt
	}
	averagePan := calculateAverage(panValues)
	averageTilt := calculateAverage(tiltValues)
	// No tilt/pan value should be more than 0.001 from the average
	for _, measurement := range rayMeasurements {
		if math.Abs(measurement.Pan-averagePan) > 0.001 {
			err = errors.New("pan value is not within 0.001 of the average")
			return RayPanTilt{}, err
		}
		if math.Abs(measurement.Tilt-averageTilt) > 0.001 {
			err = errors.New("tilt value is not within 0.001 of the average")
			return RayPanTilt{}, err
		}
	}
	ray, _ := FitLine3D(targetPositions)
	return RayPanTilt{Pan: averagePan, Tilt: averageTilt, Ray: ray}, nil
}

func (t *componentTracker) predictPanTiltAbsolute(pos r3.Vector, cameraPos r3.Vector, panPlane r3.Vector, panZeroDirection r3.Vector) (pan, tilt float64) {
	// Normalize panPlane normal (this is in camera/panPlane coordinate frame)
	panPlaneLen := math.Sqrt(panPlane.X*panPlane.X + panPlane.Y*panPlane.Y + panPlane.Z*panPlane.Z)
	if panPlaneLen < 1e-10 {
		t.logger.Errorf("panPlane normal is zero vector")
		return 0, 0
	}
	panPlaneNormal := r3.Vector{
		X: panPlane.X / panPlaneLen,
		Y: panPlane.Y / panPlaneLen,
		Z: panPlane.Z / panPlaneLen,
	}

	// Calculate direction from camera to target (in world coordinates)
	directionWorld := pos.Sub(cameraPos)
	dirLen := math.Sqrt(directionWorld.X*directionWorld.X + directionWorld.Y*directionWorld.Y + directionWorld.Z*directionWorld.Z)
	if dirLen < 1e-10 {
		// Target is at camera position, return zero pan/tilt
		return 0, 0
	}
	directionWorld = r3.Vector{X: directionWorld.X / dirLen, Y: directionWorld.Y / dirLen, Z: directionWorld.Z / dirLen}

	// Transform direction from world coordinates to panPlane coordinate system
	// Establish orthonormal basis: panPlaneNormal = Z-axis, pan0Reference = X-axis
	// pan0Reference is already projected onto panPlane and normalized, so use it directly as panPlaneX
	panPlaneX := panZeroDirection

	// Ensure panPlaneX is normalized (should already be, but check anyway)
	panPlaneXLen := math.Sqrt(panPlaneX.X*panPlaneX.X + panPlaneX.Y*panPlaneX.Y + panPlaneX.Z*panPlaneX.Z)
	if panPlaneXLen < 1e-10 {
		// Fallback: use world +X projected onto panPlane
		worldX := r3.Vector{X: 1, Y: 0, Z: 0}
		worldXProjLen := worldX.Dot(panPlaneNormal)
		panPlaneX = r3.Vector{
			X: worldX.X - panPlaneNormal.X*worldXProjLen,
			Y: worldX.Y - panPlaneNormal.Y*worldXProjLen,
			Z: worldX.Z - panPlaneNormal.Z*worldXProjLen,
		}
		panPlaneXLen = math.Sqrt(panPlaneX.X*panPlaneX.X + panPlaneX.Y*panPlaneX.Y + panPlaneX.Z*panPlaneX.Z)
		if panPlaneXLen > 1e-10 {
			panPlaneX = r3.Vector{
				X: panPlaneX.X / panPlaneXLen,
				Y: panPlaneX.Y / panPlaneXLen,
				Z: panPlaneX.Z / panPlaneXLen,
			}
		} else {
			panPlaneX = r3.Vector{X: 1, Y: 0, Z: 0}
		}
	} else if math.Abs(panPlaneXLen-1.0) > 1e-6 {
		// Normalize if not already normalized
		panPlaneX = r3.Vector{
			X: panPlaneX.X / panPlaneXLen,
			Y: panPlaneX.Y / panPlaneXLen,
			Z: panPlaneX.Z / panPlaneXLen,
		}
	}

	// Y-axis = Z cross X (right-handed coordinate system)
	panPlaneY := panPlaneNormal.Cross(panPlaneX)
	panPlaneYLen := math.Sqrt(panPlaneY.X*panPlaneY.X + panPlaneY.Y*panPlaneY.Y + panPlaneY.Z*panPlaneY.Z)
	if panPlaneYLen > 1e-10 {
		panPlaneY = r3.Vector{
			X: panPlaneY.X / panPlaneYLen,
			Y: panPlaneY.Y / panPlaneYLen,
			Z: panPlaneY.Z / panPlaneYLen,
		}
	}

	// Transform direction from world coordinates to panPlane coordinates
	// direction_panPlane = [directionWorld · panPlaneX, directionWorld · panPlaneY, directionWorld · panPlaneNormal]
	directionPanPlane := r3.Vector{
		X: directionWorld.Dot(panPlaneX),
		Y: directionWorld.Dot(panPlaneY),
		Z: directionWorld.Dot(panPlaneNormal),
	}

	// Now both direction and panPlaneNormal are in the same coordinate system (panPlane coordinates)
	// panPlaneNormal in panPlane coords = (0, 0, 1)
	// Calculate tilt: angle between direction and panPlane
	// In panPlane coords, panPlaneNormal = (0, 0, 1), so:
	// cos(angle_from_normal) = directionPanPlane.Z
	// angle_from_normal = acos(directionPanPlane.Z)
	// angle_from_plane = π/2 - angle_from_normal
	dotWithNormal := directionPanPlane.Z
	dotWithNormal = math.Max(-1.0, math.Min(1.0, dotWithNormal)) // Clamp to avoid NaN from acos
	tiltFromNormalRad := math.Acos(dotWithNormal)
	// Convert from angle from normal to angle from plane
	tiltFromPlaneRad := math.Pi/2.0 - tiltFromNormalRad
	// Tilt is the angle from the panPlane, where tilt=-1 corresponds to tiltMinDeg (in plane) and tilt=+1 corresponds to tiltMaxDeg
	tiltRangeDeg := t.tiltMaxDeg - t.tiltMinDeg
	if tiltRangeDeg > 0 {
		// Convert tiltFromPlaneRad (in radians) to degrees, then normalize to [-1, 1]
		// Linear mapping: tiltMinDeg → -1, tiltMaxDeg → +1
		tiltFromPlaneDeg := tiltFromPlaneRad * 180.0 / math.Pi
		tilt = (tiltFromPlaneDeg-t.tiltMinDeg)/tiltRangeDeg*2.0 - 1.0
		tilt = math.Max(-1.0, math.Min(1.0, tilt))
	} else {
		tilt = 0
	}
	// In panPlane coordinates, directionInPlane is just the XY components
	// directionInPlane = (directionPanPlane.X, directionPanPlane.Y, 0)
	planeDirLen := math.Sqrt(directionPanPlane.X*directionPanPlane.X + directionPanPlane.Y*directionPanPlane.Y)
	if planeDirLen < 1e-10 {
		// Direction is parallel to panPlane normal (straight up/down), pan is undefined
		// Return pan=0, tilt calculated from angle from plane
		pan = 0
		// Convert tilt to normalized [-1, 1]
		tiltRangeDeg := t.tiltMaxDeg - t.tiltMinDeg
		if tiltRangeDeg > 0 {
			tiltFromPlaneRad := math.Pi/2.0 - tiltFromNormalRad
			tiltFromPlaneDeg := tiltFromPlaneRad * 180.0 / math.Pi
			tilt = (tiltFromPlaneDeg-t.tiltMinDeg)/tiltRangeDeg*2.0 - 1.0
			tilt = math.Max(-1.0, math.Min(1.0, tilt))
		} else {
			tilt = 0
		}
		return pan, tilt
	}

	// In panPlane coordinates, pan=0 reference is along the +X axis (panPlaneX direction)
	// pan0Ref in panPlane coords = (1, 0, 0)
	// Calculate pan angle using atan2 for full 360° range
	// pan = atan2(directionPanPlane.Y, directionPanPlane.X)
	panRad := math.Atan2(directionPanPlane.Y, directionPanPlane.X)
	panDeg := panRad * 180.0 / math.Pi

	// Normalize panDeg to [0, 360) range
	// Handle negative angles by adding 360
	if panDeg < 0 {
		panDeg += 360
	}
	// Handle angles >= 360 by subtracting 360
	if panDeg >= 360 {
		panDeg -= 360
	}

	// Convert pan from degrees to normalized [-1, 1] range
	panRangeDeg := t.panMaxDeg - t.panMinDeg
	if panRangeDeg > 0 {
		// Check if this is a wrap-around range (>= 355 degrees, starting at 0)
		// This handles both full 360° ranges and ranges close to 360° (like [0, 355])
		if panRangeDeg >= 355 && t.panMinDeg == 0 {
			// Wrap-around mapping for full/near-full circle: [0, halfRange] → [0, 1], [halfRange, maxRange] → [1, -1, 0]
			halfRange := panRangeDeg / 2.0
			if panDeg <= halfRange {
				pan = panDeg / halfRange
			} else {
				pan = panDeg/halfRange - 2.0
			}
			pan = math.Max(-1.0, math.Min(1.0, pan))
		} else {
			// Standard mapping: [panMinDeg, panMaxDeg] → [-1, 1]
			// Maps panMinDeg → -1, (panMinDeg+panMaxDeg)/2 → 0, panMaxDeg → +1
			pan = (panDeg-t.panMinDeg)/panRangeDeg*2.0 - 1.0
			pan = math.Max(-1.0, math.Min(1.0, pan))
		}
	} else {
		pan = 0
	}

	// Recalculate tilt using angle from plane (already calculated above, but recalculate for consistency)
	tiltRangeDeg = t.tiltMaxDeg - t.tiltMinDeg
	if tiltRangeDeg > 0 {
		tiltFromPlaneRad := math.Pi/2.0 - tiltFromNormalRad
		tiltFromPlaneDeg := tiltFromPlaneRad * 180.0 / math.Pi
		tilt = (tiltFromPlaneDeg-t.tiltMinDeg)/tiltRangeDeg*2.0 - 1.0
		tilt = math.Max(-1.0, math.Min(1.0, tilt))
	} else {
		tilt = 0
	}
	return pan, tilt
}

/*
This function is used to get the camera position and pan plane from the two rays
The camera position is the point where the two rays intersect
The pan plane is the plane that is perpendicular to the two rays and contains the camera position at its origin such that the pan value of 0 is aligned with the +X axis of the pan plane
*/

func (t *componentTracker) calculateCameraPositionAndPanPlane(rayPanTilt1 RayPanTilt, rayPanTilt2 RayPanTilt) (cameraPos r3.Vector, panPlane r3.Vector, panZeroDirection r3.Vector, tiltZeroDirection r3.Vector) {
	ray1 := rayPanTilt1.Ray
	ray2 := rayPanTilt2.Ray
	// Calculate the closest points on the two rays (not assuming intersection)
	// Ray: P = O + t*D, minimize |(O1 + t1*D1) - (O2 + t2*D2)|
	// Analytical solution for t1 and t2:
	d1 := ray1.Direction
	d2 := ray2.Direction
	o1 := ray1.Origin
	o2 := ray2.Origin

	// w0 is the vector from the origin of ray2 to the origin of ray1
	w0 := o1.Sub(o2)
	a := d1.Dot(d1)
	b := d1.Dot(d2)
	c := d2.Dot(d2)
	d := d1.Dot(w0)
	e := d2.Dot(w0)

	denom := a*c - b*b
	var t1, t2 float64
	if math.Abs(denom) > 1e-8 {
		t1 = (b*e - c*d) / denom
		t2 = (a*e - b*d) / denom
	} else {
		t1 = 0
		t2 = 0
	}

	// pt1 and pt2 are the closest points on the two rays
	pt1 := o1.Add(d1.Mul(t1))
	pt2 := o2.Add(d2.Mul(t2))
	// Use the midpoint between the two closest points as the camera position estimate
	cameraPos = r3.Vector{
		X: (pt1.X + pt2.X) / 2,
		Y: (pt1.Y + pt2.Y) / 2,
		Z: (pt1.Z + pt2.Z) / 2,
	}
	// The pan plane is always horizontal (normal = (0, 0, 1))
	// This simplifies calculations and makes tilt independent of the measurement angles
	panPlane = r3.Vector{X: 0, Y: 0, Z: 1}

	// The zero pan reference is panPlane projection ofthe vector that points from the camera position to the point where the camera is pointing at with pan=0
	panZeroDirection = r3.Vector{X: 1, Y: 0, Z: 0}

	// Zero tilt reference is the vector that points from the camera position to the point where the camera is pointing at with tilt=0, this depends on the minDeg and maxDeg values.
	// We assume that a pan value of 0 is the midpoint of the pan range, so we can calculate the zero tilt reference as the vector that points from the camera position to the point where the camera is pointing at with pan=0
	tiltZeroDeg := (t.tiltMaxDeg + t.tiltMinDeg) / 2.0
	// Convert tiltZeroDeg to radians
	tiltZeroRad := tiltZeroDeg * math.Pi / 180.0
	// Now orient this vector to point to the zero tilt reference
	tiltZeroDirection = r3.Vector{
		X: math.Cos(tiltZeroRad),
		Y: 0,
		Z: math.Sin(tiltZeroRad),
	}
	// Normalize the tilt zero reference
	tiltZeroDirection = tiltZeroDirection.Normalize()
	return cameraPos, panPlane, panZeroDirection, tiltZeroDirection
}

// FitLine3D fits a 3D line to a set of points using Principal Component Analysis (PCA).
// Returns the best-fit line as a Ray (origin point and direction vector) and the residual error.
func FitLine3D(points []r3.Vector) (Ray, float64) {
	if len(points) < 2 {
		return Ray{}, 0
	}

	// Step 1: Calculate centroid (mean point)
	xValues := make([]float64, len(points))
	yValues := make([]float64, len(points))
	zValues := make([]float64, len(points))
	for i, p := range points {
		xValues[i] = p.X
		yValues[i] = p.Y
		zValues[i] = p.Z
	}

	centroid := calculateCentroid(points)

	// Step 2: Build mean-centered data matrix (n x 3)
	n := len(points)
	data := mat.NewDense(n, 3, nil)
	for i, p := range points {
		data.Set(i, 0, p.X-centroid.X)
		data.Set(i, 1, p.Y-centroid.Y)
		data.Set(i, 2, p.Z-centroid.Z)
	}

	// Step 3: Compute covariance matrix manually
	// Covariance = (1/(n-1)) * X^T * X where X is the mean-centered data matrix (n x 3)
	cov := mat.NewSymDense(3, nil)
	scale := 1.0 / float64(n-1)
	for i := 0; i < 3; i++ {
		for j := i; j < 3; j++ {
			var sum float64
			for k := 0; k < n; k++ {
				sum += data.At(k, i) * data.At(k, j)
			}
			cov.SetSym(i, j, scale*sum)
		}
	}

	// Step 4: Eigen decomposition (symmetric matrix -> use EigenSym)
	var eig mat.EigenSym
	if !eig.Factorize(cov, true) {
		return Ray{}, math.NaN() // Failed to factorize
	}

	// Get eigenvalues and eigenvectors
	eigenvals := eig.Values(nil)
	eigenvectors := mat.NewDense(3, 3, nil)
	eig.VectorsTo(eigenvectors)

	// Find index of largest eigenvalue (principal component)
	maxIdx := 0
	for i := 1; i < 3; i++ {
		if eigenvals[i] > eigenvals[maxIdx] {
			maxIdx = i
		}
	}

	// Extract direction vector (principal eigenvector)
	direction := r3.Vector{
		X: eigenvectors.At(0, maxIdx),
		Y: eigenvectors.At(1, maxIdx),
		Z: eigenvectors.At(2, maxIdx),
	}

	// Normalize direction
	dirLen := math.Sqrt(direction.X*direction.X + direction.Y*direction.Y + direction.Z*direction.Z)
	if dirLen > 1e-10 {
		direction.X /= dirLen
		direction.Y /= dirLen
		direction.Z /= dirLen
	}

	// Ensure direction points from centroid toward the points (not away from them)
	// Use the point furthest from the origin to determine correct direction
	// (assuming camera is near origin and targets are further away)
	if len(points) > 0 {
		maxDistFromOrigin := 0.0
		var referencePoint r3.Vector
		for _, p := range points {
			distFromOrigin := math.Sqrt(p.X*p.X + p.Y*p.Y + p.Z*p.Z)
			if distFromOrigin > maxDistFromOrigin {
				maxDistFromOrigin = distFromOrigin
				referencePoint = p
			}
		}
		// Compute vector from centroid to reference point
		toReference := r3.Vector{
			X: referencePoint.X - centroid.X,
			Y: referencePoint.Y - centroid.Y,
			Z: referencePoint.Z - centroid.Z,
		}
		// Normalize toReference vector
		toRefLen := math.Sqrt(toReference.X*toReference.X + toReference.Y*toReference.Y + toReference.Z*toReference.Z)
		if toRefLen > 1e-10 {
			toReference = r3.Vector{
				X: toReference.X / toRefLen,
				Y: toReference.Y / toRefLen,
				Z: toReference.Z / toRefLen,
			}
			// If direction doesn't align with vector from centroid to reference point, flip it
			if direction.Dot(toReference) < 0 {
				direction = r3.Vector{X: -direction.X, Y: -direction.Y, Z: -direction.Z}
			}
		}
	}

	// Step 5: Calculate residual error
	var totalError float64
	for _, p := range points {
		// Vector from centroid to point
		toPoint := r3.Vector{X: p.X - centroid.X, Y: p.Y - centroid.Y, Z: p.Z - centroid.Z}

		// Project onto direction
		projLen := toPoint.Dot(direction)
		projection := r3.Vector{
			X: direction.X * projLen,
			Y: direction.Y * projLen,
			Z: direction.Z * projLen,
		}

		// Perpendicular component (error)
		perpendicular := r3.Vector{
			X: toPoint.X - projection.X,
			Y: toPoint.Y - projection.Y,
			Z: toPoint.Z - projection.Z,
		}

		totalError += perpendicular.Norm2()
	}

	residual := math.Sqrt(totalError / float64(n))

	return Ray{
		Origin:    centroid,
		Direction: direction,
	}, residual
}

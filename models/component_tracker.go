package models

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/utils"
	"gonum.org/v1/gonum/mat"
)

type PTZValues struct {
	Pan  float64
	Tilt float64
	Zoom float64
}

var (
	ModelComponentTracker = resource.NewModel("viam", "ptz-target-tracker", "component-tracker")
	errUnimplemented      = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(genericservice.API, ModelComponentTracker,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newComponentTracker,
		},
	)
}

type Config struct {
	TargetComponentName string      `json:"target_component_name"`
	PTZCameraName       string      `json:"ptz_camera_name"`
	OnvifPTZClientName  string      `json:"onvif_ptz_client_name"`
	UpdateRateHz        float64     `json:"update_rate_hz"`
	EnableOnStart       bool        `json:"enable_on_start"`
	PanMinDeg           float64     `json:"pan_min_deg"`
	PanMaxDeg           float64     `json:"pan_max_deg"`
	TiltMinDeg          float64     `json:"tilt_min_deg"`
	TiltMaxDeg          float64     `json:"tilt_max_deg"`
	MinZoomDistanceMM   float64     `json:"min_zoom_distance_mm"`
	MaxZoomDistanceMM   float64     `json:"max_zoom_distance_mm"`
	Deadzone            float64     `json:"deadzone"`
	Calibration         Calibration `json:"calibration"`
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
	if cfg.Deadzone < 0 || cfg.Deadzone > 1 {
		return nil, nil, errors.New("deadzone must be greater than or equal to 0 and less than or equal to 1 (normalized range)")
	}
	if cfg.MinZoomDistanceMM < 0 {
		return nil, nil, errors.New("min_zoom_distance_mm must be greater than or equal to 0")
	}
	if cfg.MaxZoomDistanceMM < 0 {
		return nil, nil, errors.New("max_zoom_distance_mm must be greater than or equal to 0")
	}
	if cfg.Calibration.IsCalibrated {
		if len(cfg.Calibration.PanPolyCoeffs) != 10 {
			return nil, nil, errors.New("pan_poly_coeffs must have exactly 10 coefficients")
		}
		if len(cfg.Calibration.TiltPolyCoeffs) != 10 {
			return nil, nil, errors.New("tilt_poly_coeffs must have exactly 10 coefficients")
		}
		// Check if all coefficients are valid numbers
		for _, coeff := range cfg.Calibration.PanPolyCoeffs {
			if math.IsNaN(coeff) || math.IsInf(coeff, 0) {
				return nil, nil, errors.New("pan_poly_coeffs must contain valid numbers")
			}
		}
		for _, coeff := range cfg.Calibration.TiltPolyCoeffs {
			if math.IsNaN(coeff) || math.IsInf(coeff, 0) {
				return nil, nil, errors.New("tilt_poly_coeffs must contain valid numbers")
			}
		}
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
	PanPolyCoeffs  []float64 `json:"pan_poly_coeffs"`
	TiltPolyCoeffs []float64 `json:"tilt_poly_coeffs"`
	IsCalibrated   bool      `json:"is_calibrated"`
}

const PAN_SPEED = 1.0
const TILT_SPEED = 1.0
const ZOOM_SPEED = 1.0

type componentTracker struct {
	resource.AlwaysRebuild
	name resource.Name

	logger logging.Logger
	cfg    *Config

	frameSystemService  framesystem.Service
	targetComponentName string
	onvifPTZClientName  string

	onvifPTZClient generic.Resource

	updateRateHz                     float64
	panMinDeg                        float64
	panMaxDeg                        float64
	tiltMinDeg                       float64
	tiltMaxDeg                       float64
	samples                          []TrackingSample
	calibration                      Calibration
	lastSentTZValues                 PTZValues
	deadzone                         float64
	minZoomDistance                  float64
	maxZoomDistance                  float64
	minZoomValue                     float64
	maxZoomValue                     float64
	absoluteCalibrationPanPlane      r3.Vector
	absoluteCalibrationPan0Reference r3.Vector // Direction in panPlane that corresponds to pan=0
	worker                           *utils.StoppableWorkers
}

// Close implements resource.Resource.
func (s *componentTracker) Close(ctx context.Context) error {
	s.worker.Stop()
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
	configJSON, _ := json.MarshalIndent(conf, "", "  ")
	logger.Debugf("Creating Component with the following config:\n%s", configJSON)

	onvifPTZClientName := resource.NewName(generic.API, conf.OnvifPTZClientName)
	onvifPTZClient, err := deps.GetResource(onvifPTZClientName)
	if err != nil {
		return nil, fmt.Errorf("failed to get ONVIF PTZ client resource: %w", err)
	}

	frameSystemService, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, fmt.Errorf("failed to get frame system service: %w", err)
	}

	s := &componentTracker{
		name:                name,
		logger:              logger,
		cfg:                 conf,
		frameSystemService:  frameSystemService,
		targetComponentName: conf.TargetComponentName,
		onvifPTZClient:      onvifPTZClient,
		updateRateHz:        conf.UpdateRateHz,
		panMinDeg:           conf.PanMinDeg,
		panMaxDeg:           conf.PanMaxDeg,
		tiltMinDeg:          conf.TiltMinDeg,
		tiltMaxDeg:          conf.TiltMaxDeg,
		deadzone:            conf.Deadzone,
		minZoomDistance:     conf.MinZoomDistanceMM,
		maxZoomDistance:     conf.MaxZoomDistanceMM,
		calibration: Calibration{
			PanPolyCoeffs:  []float64{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			TiltPolyCoeffs: []float64{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			IsCalibrated:   false,
		},
		worker: utils.NewBackgroundStoppableWorkers(),
	}

	// Copy calibration if provided
	s.logger.Debugf("Creating component tracker with config: %+v", conf)
	if conf.Calibration.IsCalibrated && len(conf.Calibration.PanPolyCoeffs) > 0 {
		s.calibration = conf.Calibration
		s.logger.Infof("Created component with calibration: %+v", s.calibration)
	}

	if conf.EnableOnStart {
		s.logger.Info("Starting PTZ component tracker on start")
		s.worker.Add(s.trackingLoop)
		s.logger.Info("PTZ component tracker started")
	}

	return s, nil
}

func (s *componentTracker) Name() resource.Name {
	return s.name
}

func (t *componentTracker) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	t.logger.Debugf("DoCommand: %+v", cmd)
	if t.cfg.EnableOnStart {
		return nil, errors.New("cannot execute commands if enable_on_start is set to true")
	}
	switch cmd["command"] {
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

		return map[string]interface{}{
			"status":  "success",
			"samples": len(t.samples),
		}, nil

	case "push-sample":
		// Get current target position from arm
		targetPose, err := t.getPose(ctx, t.targetComponentName)
		if err != nil {
			return nil, err
		}
		targetPos := targetPose.Pose().Point()

		// Get current pan/tilt (user has manually centered)
		ptzValues, err := t.getCameraCurrentPTZStatus(ctx)
		if err != nil {
			return nil, err
		}

		tag := "manual"
		if cmd["tag"] != nil {
			tag = cmd["tag"].(string)
		}

		sample := TrackingSample{
			TargetPos: targetPos,
			Pan:       ptzValues.Pan,
			Tilt:      ptzValues.Tilt,
			Tag:       tag,
		}
		t.samples = append(t.samples, sample)

		t.logger.Infof("Sample %d: (%.1f, %.1f, %.1f) → pan=%.4f, tilt=%.4f",
			len(t.samples), targetPos.X, targetPos.Y, targetPos.Z, ptzValues.Pan, ptzValues.Tilt)
		lastSample := t.samples[len(t.samples)-1]
		return map[string]interface{}{
			"sample_number": len(t.samples),
			"target":        map[string]interface{}{"x": lastSample.TargetPos.X, "y": lastSample.TargetPos.Y, "z": lastSample.TargetPos.Z},
			"pan":           lastSample.Pan,
			"tilt":          lastSample.Tilt,
			"tag":           lastSample.Tag,
		}, nil

	case "pop-sample":
		if len(t.samples) == 0 {
			return nil, fmt.Errorf("no samples to remove")
		}
		t.samples = t.samples[:len(t.samples)-1]
		return map[string]interface{}{"status": "removed", "index": len(t.samples)}, nil

	case "clear-calibration":
		t.calibration = Calibration{}
		t.samples = []TrackingSample{}
		return map[string]interface{}{"status": "cleared"}, nil

	case "compute-polynomial":
		if len(t.samples) < 10 {
			return nil, fmt.Errorf("need at least 10 samples for polynomial fit, have %d", len(t.samples))
		}

		panErr, tiltErr, err := t.fitPolynomial()
		if err != nil {
			return map[string]interface{}{
				"status": "error",
				"error":  err.Error(),
			}, nil
		}

		return map[string]interface{}{
			"status":           "success",
			"pan_error_avg":    panErr,
			"tilt_error_avg":   tiltErr,
			"samples_used":     len(t.samples),
			"is_calibrated":    t.calibration.IsCalibrated,
			"pan_poly_coeffs":  []float64{t.calibration.PanPolyCoeffs[0], t.calibration.PanPolyCoeffs[1], t.calibration.PanPolyCoeffs[2], t.calibration.PanPolyCoeffs[3], t.calibration.PanPolyCoeffs[4], t.calibration.PanPolyCoeffs[5], t.calibration.PanPolyCoeffs[6], t.calibration.PanPolyCoeffs[7], t.calibration.PanPolyCoeffs[8], t.calibration.PanPolyCoeffs[9]},
			"tilt_poly_coeffs": []float64{t.calibration.TiltPolyCoeffs[0], t.calibration.TiltPolyCoeffs[1], t.calibration.TiltPolyCoeffs[2], t.calibration.TiltPolyCoeffs[3], t.calibration.TiltPolyCoeffs[4], t.calibration.TiltPolyCoeffs[5], t.calibration.TiltPolyCoeffs[6], t.calibration.TiltPolyCoeffs[7], t.calibration.TiltPolyCoeffs[8], t.calibration.TiltPolyCoeffs[9]},
		}, nil

	default:
		return nil, fmt.Errorf("invalid command: %v", cmd["command"])
	}
}

func (t *componentTracker) getPose(ctx context.Context, componentName string) (*referenceframe.PoseInFrame, error) {
	pose, err := t.frameSystemService.GetPose(ctx, componentName, "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		t.logger.Errorf("Failed to get pose for component %s: %v", componentName, err)
		return nil, fmt.Errorf("Failed to get pose for component %s: %v", componentName, err)
	}
	return pose, nil
}

func (t *componentTracker) getCameraCurrentPTZStatus(ctx context.Context) (PTZValues, error) {
	ptzStatusResponse, err := t.onvifPTZClient.DoCommand(ctx, map[string]interface{}{
		"command": "get-status",
	})
	if err != nil {
		t.logger.Errorf("Failed to get PTZ status: %v", err)
		return PTZValues{}, err
	}
	moveStatus, ok := ptzStatusResponse["move_status"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ move status is not a map")
		return PTZValues{}, fmt.Errorf("PTZ move status is not a map")
	}
	movePanTilt, ok := moveStatus["pan_tilt"].(string)
	if !ok {
		t.logger.Errorf("PTZ move pan tilt is not a string")
		return PTZValues{}, fmt.Errorf("PTZ move pan tilt is not a string")
	}
	if movePanTilt != "IDLE" {
		t.logger.Debugf("PTZ pan/tilt is moving (status: %s), reading position anyway", movePanTilt)
	}
	moveZoom, ok := moveStatus["zoom"].(string)
	if !ok {
		t.logger.Errorf("PTZ move zoom is not a string")
		return PTZValues{}, fmt.Errorf("PTZ move zoom is not a string")
	}
	if moveZoom != "IDLE" {
		t.logger.Debugf("PTZ zoom is moving (status: %s), reading position anyway", moveZoom)
	}
	position, ok := ptzStatusResponse["position"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ status is not a map")
		return PTZValues{}, fmt.Errorf("PTZ status is not a map")
	}
	zoom, ok := position["zoom"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ zoom is not a map")
		return PTZValues{}, fmt.Errorf("PTZ zoom is not a map")
	}
	zoomX, ok := zoom["x"].(float64)
	if !ok {
		t.logger.Errorf("PTZ zoom x is not a float")
		return PTZValues{}, fmt.Errorf("PTZ zoom x is not a float")
	}
	panTilt, ok := position["pan_tilt"].(map[string]interface{})
	if !ok {
		t.logger.Errorf("PTZ pan tilt is not a map")
		return PTZValues{}, fmt.Errorf("PTZ pan tilt is not a map")
	}
	panTiltX, ok := panTilt["x"].(float64)
	if !ok {
		t.logger.Errorf("PTZ pan tilt x is not a float")
		return PTZValues{}, fmt.Errorf("PTZ pan tilt x is not a float")
	}
	panTiltY, ok := panTilt["y"].(float64)
	if !ok {
		t.logger.Errorf("PTZ pan tilt y is not a float")
		return PTZValues{}, fmt.Errorf("PTZ pan tilt y is not a float")
	}
	t.logger.Debugf("PTZ status: zoom=%.1f, pan=%.1f, tilt=%.1f", zoomX, panTiltX, panTiltY)

	return PTZValues{
		Pan:  panTiltX,
		Tilt: panTiltY,
		Zoom: zoomX,
	}, nil
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
			err := t.trackTarget(ctx)
			if err != nil {
				t.logger.Errorf("Failed to track target: %v", err)
			}
		}
	}
}

func (t *componentTracker) sendAbsoluteMove(ctx context.Context, ptzValues PTZValues) error {
	currentPTZValues, err := t.getCameraCurrentPTZStatus(ctx)
	if err != nil {
		return fmt.Errorf("failed to get current PTZ status: %w", err)
	}

	// Calculate deltas to from currentPTZValues to the future PTZValues, if they are within deadzone, skip move
	panDeltaNormalized := math.Abs(ptzValues.Pan - currentPTZValues.Pan)
	tiltDeltaNormalized := math.Abs(ptzValues.Tilt - currentPTZValues.Tilt)
	// Deadzone check: skip move if target is within deadzone of current position
	// Deadzone is in normalized [0, 1] range (e.g., 0.01 = 1% of the full range)
	if t.deadzone > 0 {
		if panDeltaNormalized < t.deadzone && tiltDeltaNormalized < t.deadzone {
			t.logger.Debugf("Skipping move - within deadzone: pan_delta=%.4f (deadzone=%.4f), tilt_delta=%.4f (deadzone=%.4f)", panDeltaNormalized, t.deadzone, tiltDeltaNormalized, t.deadzone)
			return nil
		}
	}

	// Comprehensive debug log with all information
	t.logger.Debugf("Sending absolute move: target pan=%.3f (current=%.3f, delta=%.4f norm, speed=%.3f), target tilt=%.3f (current=%.3f, delta=%.4f norm, speed=%.3f), zoom=%.3f, speeds: pan=%.3f tilt=%.3f zoom=%.3f",
		ptzValues.Pan, t.lastSentTZValues.Pan, panDeltaNormalized, PAN_SPEED,
		ptzValues.Tilt, t.lastSentTZValues.Tilt, tiltDeltaNormalized, TILT_SPEED,
		ptzValues.Zoom, PAN_SPEED, TILT_SPEED, ZOOM_SPEED)
	_, err = t.onvifPTZClient.DoCommand(ctx, map[string]interface{}{
		"command":       "absolute-move",
		"pan_position":  ptzValues.Pan,
		"tilt_position": ptzValues.Tilt,
		"zoom_position": ptzValues.Zoom,
		"pan_speed":     PAN_SPEED,
		"tilt_speed":    TILT_SPEED,
		"zoom_speed":    ZOOM_SPEED,
	})
	if err != nil {
		return fmt.Errorf("failed to send absolute move: %w", err)
	}

	t.lastSentTZValues = ptzValues

	return nil
}

// Fit: pan = Ax² + By² + Cz² + Dxy + Exz + Fyz + Gx + Hy + Iz + J
func (t *componentTracker) fitPolynomial() (panError, tiltError float64, err error) {
	t.calibration.PanPolyCoeffs = t.fitPolynomialSingle(func(s TrackingSample) float64 { return s.Pan })
	t.calibration.TiltPolyCoeffs = t.fitPolynomialSingle(func(s TrackingSample) float64 { return s.Tilt })

	// Calculate errors
	var panErrSum, tiltErrSum float64
	for _, s := range t.samples {
		predPan, predTilt, err := t.calculatePanTiltPolynomial(s.TargetPos)
		if err != nil {
			return 0, 0, fmt.Errorf("failed to predict pan/tilt for error calculation: %w", err)
		}
		panErrSum += math.Abs(predPan - s.Pan)
		tiltErrSum += math.Abs(predTilt - s.Tilt)
	}

	n := float64(len(t.samples))
	panError = panErrSum / n
	tiltError = tiltErrSum / n

	t.logger.Infof("Polynomial fit complete: avg error pan=%.5f, tilt=%.5f", panError, tiltError)

	t.calibration.IsCalibrated = true
	return panError, tiltError, nil
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

func (t *componentTracker) fitPolynomialSingle(getValue func(TrackingSample) float64) []float64 {
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

	coeffs := solveLinearSystem10x10(XtX, XtY)
	// Convert array to slice
	return coeffs[:]
}

func (t *componentTracker) calculatePanTiltPolynomial(pos r3.Vector) (pan, tilt float64, err error) {
	x, y, z := pos.X, pos.Y, pos.Z
	features := [10]float64{
		x * x, y * y, z * z,
		x * y, x * z, y * z,
		x, y, z, 1,
	}

	pan, tilt = 0, 0
	if len(t.calibration.PanPolyCoeffs) != len(t.calibration.TiltPolyCoeffs) {
		t.logger.Errorf("Calibration polynomial coefficient length mismatch: pan=%d, tilt=%d", len(t.calibration.PanPolyCoeffs), len(t.calibration.TiltPolyCoeffs))
		return 0, 0, fmt.Errorf("Calibration polynomial coefficient length mismatch: pan=%d, tilt=%d", len(t.calibration.PanPolyCoeffs), len(t.calibration.TiltPolyCoeffs))
	}
	if len(features) != len(t.calibration.PanPolyCoeffs) {
		t.logger.Errorf("Feature length mismatch: features=%d, pan_coeffs=%d", len(features), len(t.calibration.PanPolyCoeffs))
		return 0, 0, fmt.Errorf("Feature length mismatch: features=%d, pan_coeffs=%d", len(features), len(t.calibration.PanPolyCoeffs))
	}
	for i := range features {
		pan += t.calibration.PanPolyCoeffs[i] * features[i]
		tilt += t.calibration.TiltPolyCoeffs[i] * features[i]
	}
	return pan, tilt, nil
}

func (t *componentTracker) calculatePanTiltZoom(ctx context.Context, targetPos r3.Vector) (ptzValues PTZValues, err error) {
	cameraPose, err := t.getPose(ctx, t.cfg.PTZCameraName)
	if err != nil {
		return PTZValues{}, fmt.Errorf("failed to get camera pose: %w", err)
	}
	cameraPos := cameraPose.Pose().Point()
	ptzValues.Pan, ptzValues.Tilt, err = t.calculatePanTiltPolynomial(targetPos)
	if err != nil {
		return PTZValues{}, err
	}
	ptzValues.Zoom = t.calculateZoom(targetPos, cameraPos)
	return ptzValues, nil
}
func (t *componentTracker) calculateZoom(pos r3.Vector, cameraPos r3.Vector) float64 {
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
	if !t.calibration.IsCalibrated {
		return errors.New("not calibrated - run compute-polynomial first")
	}

	targetPose, err := t.getPose(ctx, t.targetComponentName)
	if err != nil {
		return err
	}
	targetPosition := targetPose.Pose().Point()

	ptzValues, err := t.calculatePanTiltZoom(ctx, targetPosition)
	if err != nil {
		t.logger.Errorf("Failed to predict pan/tilt/zoom: %v", err)
		return err
	}
	t.logger.Debugf("Predicted pan: %.1f, tilt: %.1f, zoom: %.1f", ptzValues.Pan, ptzValues.Tilt, ptzValues.Zoom)

	// Clamp
	ptzValues.Pan = math.Max(-1.0, math.Min(1.0, ptzValues.Pan))
	ptzValues.Tilt = math.Max(-1.0, math.Min(1.0, ptzValues.Tilt))
	return t.sendAbsoluteMove(ctx, ptzValues)
}

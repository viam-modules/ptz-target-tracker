package models

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
)

// validConfig returns a Config that passes Validate, so each test case can
// change exactly the one field it cares about.
func validConfig() Config {
	return Config{
		TargetComponentName: "target",
		PTZCameraName:       "ptz-cam",
		OnvifPTZClientName:  "onvif",
		UpdateRateHz:        10,
		PanMinDeg:           0,
		PanMaxDeg:           355,
		TiltMinDeg:          5,
		TiltMaxDeg:          90,
	}
}

func TestConfigValidate(t *testing.T) {
	tenCoeffs := make([]float64, 10)

	tests := []struct {
		name    string
		mutate  func(*Config)
		wantErr bool
	}{
		{"valid", func(*Config) {}, false},
		{"missing target_component_name", func(c *Config) { c.TargetComponentName = "" }, true},
		{"missing ptz_camera_name", func(c *Config) { c.PTZCameraName = "" }, true},
		{"missing onvif_ptz_client_name", func(c *Config) { c.OnvifPTZClientName = "" }, true},
		{"zero update_rate_hz", func(c *Config) { c.UpdateRateHz = 0 }, true},
		{"negative update_rate_hz", func(c *Config) { c.UpdateRateHz = -1 }, true},
		{"pan_max equals pan_min", func(c *Config) { c.PanMinDeg, c.PanMaxDeg = 10, 10 }, true},
		{"pan_max below pan_min", func(c *Config) { c.PanMinDeg, c.PanMaxDeg = 20, 10 }, true},
		{"tilt_max equals tilt_min", func(c *Config) { c.TiltMinDeg, c.TiltMaxDeg = 10, 10 }, true},
		{"tilt_max below tilt_min", func(c *Config) { c.TiltMinDeg, c.TiltMaxDeg = 20, 10 }, true},
		{"deadzone zero", func(c *Config) { c.Deadzone = 0 }, false},
		{"deadzone one", func(c *Config) { c.Deadzone = 1 }, false},
		{"deadzone negative", func(c *Config) { c.Deadzone = -0.1 }, true},
		{"deadzone above one", func(c *Config) { c.Deadzone = 1.1 }, true},
		{"negative min_zoom_distance_mm", func(c *Config) { c.MinZoomDistanceMM = -1 }, true},
		{"negative max_zoom_distance_mm", func(c *Config) { c.MaxZoomDistanceMM = -1 }, true},
		{
			"calibrated with correct coefficient counts",
			func(c *Config) {
				c.Calibration = Calibration{IsCalibrated: true, PanPolyCoeffs: tenCoeffs, TiltPolyCoeffs: tenCoeffs}
			},
			false,
		},
		{
			"calibrated with too few pan coefficients",
			func(c *Config) {
				c.Calibration = Calibration{IsCalibrated: true, PanPolyCoeffs: make([]float64, 9), TiltPolyCoeffs: tenCoeffs}
			},
			true,
		},
		{
			"calibrated with too few tilt coefficients",
			func(c *Config) {
				c.Calibration = Calibration{IsCalibrated: true, PanPolyCoeffs: tenCoeffs, TiltPolyCoeffs: make([]float64, 9)}
			},
			true,
		},
		{
			"calibrated with NaN pan coefficient",
			func(c *Config) {
				coeffs := make([]float64, 10)
				coeffs[3] = math.NaN()
				c.Calibration = Calibration{IsCalibrated: true, PanPolyCoeffs: coeffs, TiltPolyCoeffs: tenCoeffs}
			},
			true,
		},
		{
			"calibrated with Inf tilt coefficient",
			func(c *Config) {
				coeffs := make([]float64, 10)
				coeffs[7] = math.Inf(1)
				c.Calibration = Calibration{IsCalibrated: true, PanPolyCoeffs: tenCoeffs, TiltPolyCoeffs: coeffs}
			},
			true,
		},
		{
			// Coefficients are only checked when IsCalibrated is set.
			"uncalibrated ignores bad coefficients",
			func(c *Config) {
				c.Calibration = Calibration{IsCalibrated: false, PanPolyCoeffs: []float64{math.NaN()}}
			},
			false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			cfg := validConfig()
			tt.mutate(&cfg)

			required, optional, err := cfg.Validate("components.0")
			if (err != nil) != tt.wantErr {
				t.Fatalf("Validate() error = %v, wantErr = %v", err, tt.wantErr)
			}
			if err == nil {
				if required != nil {
					t.Errorf("Validate() required deps = %v, want nil", required)
				}
				if optional != nil {
					t.Errorf("Validate() optional deps = %v, want nil", optional)
				}
			}
		})
	}
}

// Validate fills in pan/tilt limits in place when both ends are left at zero.
func TestConfigValidateAppliesDefaultLimits(t *testing.T) {
	t.Run("pan and tilt both defaulted", func(t *testing.T) {
		cfg := validConfig()
		cfg.PanMinDeg, cfg.PanMaxDeg = 0, 0
		cfg.TiltMinDeg, cfg.TiltMaxDeg = 0, 0

		if _, _, err := cfg.Validate("components.0"); err != nil {
			t.Fatalf("Validate() unexpected error: %v", err)
		}
		if cfg.PanMinDeg != 0 || cfg.PanMaxDeg != 355 {
			t.Errorf("pan limits = [%v, %v], want [0, 355]", cfg.PanMinDeg, cfg.PanMaxDeg)
		}
		if cfg.TiltMinDeg != 5 || cfg.TiltMaxDeg != 90 {
			t.Errorf("tilt limits = [%v, %v], want [5, 90]", cfg.TiltMinDeg, cfg.TiltMaxDeg)
		}
	})

	t.Run("explicit limits are preserved", func(t *testing.T) {
		cfg := validConfig()
		cfg.PanMinDeg, cfg.PanMaxDeg = -90, 90
		cfg.TiltMinDeg, cfg.TiltMaxDeg = -10, 45

		if _, _, err := cfg.Validate("components.0"); err != nil {
			t.Fatalf("Validate() unexpected error: %v", err)
		}
		if cfg.PanMinDeg != -90 || cfg.PanMaxDeg != 90 {
			t.Errorf("pan limits = [%v, %v], want [-90, 90]", cfg.PanMinDeg, cfg.PanMaxDeg)
		}
		if cfg.TiltMinDeg != -10 || cfg.TiltMaxDeg != 45 {
			t.Errorf("tilt limits = [%v, %v], want [-10, 45]", cfg.TiltMinDeg, cfg.TiltMaxDeg)
		}
	})
}

func TestCalculateZoom(t *testing.T) {
	tracker := &componentTracker{
		logger:          logging.NewTestLogger(t),
		minZoomDistance: 1000,
		maxZoomDistance: 5000,
	}
	origin := r3.Vector{X: 0, Y: 0, Z: 0}

	tests := []struct {
		name     string
		distance float64
		want     float64
	}{
		{"closer than min clamps to zoomed out", 500, minZoomValue},
		{"exactly min clamps to zoomed out", 1000, minZoomValue},
		{"farther than max clamps to zoomed in", 9000, maxZoomValue},
		{"exactly max clamps to zoomed in", 5000, maxZoomValue},
		{"midpoint interpolates halfway", 3000, 0.5},
		{"quarter point interpolates", 2000, 0.25},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			// Distance from the origin along X is just the X coordinate.
			got := tracker.calculateZoom(r3.Vector{X: tt.distance}, origin)
			if math.Abs(got-tt.want) > 1e-9 {
				t.Errorf("calculateZoom(distance=%v) = %v, want %v", tt.distance, got, tt.want)
			}
		})
	}
}

// A degenerate zoom range must not divide by zero.
func TestCalculateZoomEqualMinMaxDistance(t *testing.T) {
	tracker := &componentTracker{
		logger:          logging.NewTestLogger(t),
		minZoomDistance: 2000,
		maxZoomDistance: 2000,
	}

	got := tracker.calculateZoom(r3.Vector{X: 2500}, r3.Vector{})
	if got != maxZoomValue {
		t.Errorf("calculateZoom() = %v, want %v", got, maxZoomValue)
	}
}

func TestSolveLinearSystem10x10(t *testing.T) {
	t.Run("identity matrix returns b unchanged", func(t *testing.T) {
		var a [10][10]float64
		var b [10]float64
		for i := 0; i < 10; i++ {
			a[i][i] = 1
			b[i] = float64(i) + 1
		}

		got := solveLinearSystem10x10(a, b)
		for i := 0; i < 10; i++ {
			if math.Abs(got[i]-b[i]) > 1e-9 {
				t.Errorf("coefficient %d = %v, want %v", i, got[i], b[i])
			}
		}
	})

	t.Run("diagonal matrix divides through", func(t *testing.T) {
		var a [10][10]float64
		var b [10]float64
		for i := 0; i < 10; i++ {
			a[i][i] = 2
			b[i] = 4
		}

		got := solveLinearSystem10x10(a, b)
		for i := 0; i < 10; i++ {
			if math.Abs(got[i]-2) > 1e-9 {
				t.Errorf("coefficient %d = %v, want 2", i, got[i])
			}
		}
	})
}

func TestCalculatePanTiltPolynomial(t *testing.T) {
	t.Run("evaluates the quadratic feature vector", func(t *testing.T) {
		// Features are [x², y², z², xy, xz, yz, x, y, z, 1]. Selecting only the
		// linear terms and the bias gives pan = x + 2y + 3z + 4.
		panCoeffs := []float64{0, 0, 0, 0, 0, 0, 1, 2, 3, 4}
		// Selecting only x² gives tilt = x².
		tiltCoeffs := []float64{1, 0, 0, 0, 0, 0, 0, 0, 0, 0}

		tracker := &componentTracker{
			logger:      logging.NewTestLogger(t),
			calibration: Calibration{PanPolyCoeffs: panCoeffs, TiltPolyCoeffs: tiltCoeffs, IsCalibrated: true},
		}

		pan, tilt, err := tracker.calculatePanTiltPolynomial(r3.Vector{X: 2, Y: 3, Z: 4})
		if err != nil {
			t.Fatalf("calculatePanTiltPolynomial() unexpected error: %v", err)
		}
		if wantPan := 2.0 + 2*3 + 3*4 + 4; math.Abs(pan-wantPan) > 1e-9 {
			t.Errorf("pan = %v, want %v", pan, wantPan)
		}
		if wantTilt := 4.0; math.Abs(tilt-wantTilt) > 1e-9 {
			t.Errorf("tilt = %v, want %v", tilt, wantTilt)
		}
	})

	t.Run("mismatched pan and tilt lengths error", func(t *testing.T) {
		tracker := &componentTracker{
			logger:      logging.NewTestLogger(t),
			calibration: Calibration{PanPolyCoeffs: make([]float64, 10), TiltPolyCoeffs: make([]float64, 5)},
		}

		if _, _, err := tracker.calculatePanTiltPolynomial(r3.Vector{}); err == nil {
			t.Error("calculatePanTiltPolynomial() error = nil, want an error")
		}
	})

	t.Run("coefficient count not matching feature count errors", func(t *testing.T) {
		tracker := &componentTracker{
			logger:      logging.NewTestLogger(t),
			calibration: Calibration{PanPolyCoeffs: make([]float64, 5), TiltPolyCoeffs: make([]float64, 5)},
		}

		if _, _, err := tracker.calculatePanTiltPolynomial(r3.Vector{}); err == nil {
			t.Error("calculatePanTiltPolynomial() error = nil, want an error")
		}
	})

	t.Run("uncalibrated tracker with no coefficients errors", func(t *testing.T) {
		tracker := &componentTracker{logger: logging.NewTestLogger(t)}

		if _, _, err := tracker.calculatePanTiltPolynomial(r3.Vector{X: 1}); err == nil {
			t.Error("calculatePanTiltPolynomial() error = nil, want an error")
		}
	})
}

// Fitting samples drawn from a known linear function should recover that
// function closely enough to predict the samples back.
func TestFitPolynomialRecoversLinearFunction(t *testing.T) {
	wantPan := func(v r3.Vector) float64 { return 2*v.X + 3*v.Y + 1*v.Z + 5 }
	wantTilt := func(v r3.Vector) float64 { return -1*v.X + 0.5*v.Y + 2 }

	var samples []TrackingSample
	for x := 0; x < 3; x++ {
		for y := 0; y < 3; y++ {
			for z := 0; z < 3; z++ {
				pos := r3.Vector{X: float64(x), Y: float64(y), Z: float64(z)}
				samples = append(samples, TrackingSample{
					TargetPos: pos,
					Pan:       wantPan(pos),
					Tilt:      wantTilt(pos),
				})
			}
		}
	}

	tracker := &componentTracker{
		logger:  logging.NewTestLogger(t),
		samples: samples,
	}

	panError, tiltError, err := tracker.fitPolynomial()
	if err != nil {
		t.Fatalf("fitPolynomial() unexpected error: %v", err)
	}
	if panError > 1e-6 {
		t.Errorf("mean pan error = %v, want <= 1e-6", panError)
	}
	if tiltError > 1e-6 {
		t.Errorf("mean tilt error = %v, want <= 1e-6", tiltError)
	}
	if !tracker.calibration.IsCalibrated {
		t.Error("fitPolynomial() did not mark the tracker as calibrated")
	}
	if got := len(tracker.calibration.PanPolyCoeffs); got != 10 {
		t.Errorf("pan coefficient count = %d, want 10", got)
	}
	if got := len(tracker.calibration.TiltPolyCoeffs); got != 10 {
		t.Errorf("tilt coefficient count = %d, want 10", got)
	}

	// The fit must also generalize to a point that was not in the sample set.
	unseen := r3.Vector{X: 1.5, Y: 0.5, Z: 2.5}
	pan, tilt, err := tracker.calculatePanTiltPolynomial(unseen)
	if err != nil {
		t.Fatalf("calculatePanTiltPolynomial() unexpected error: %v", err)
	}
	if math.Abs(pan-wantPan(unseen)) > 1e-6 {
		t.Errorf("pan at unseen point = %v, want %v", pan, wantPan(unseen))
	}
	if math.Abs(tilt-wantTilt(unseen)) > 1e-6 {
		t.Errorf("tilt at unseen point = %v, want %v", tilt, wantTilt(unseen))
	}
}

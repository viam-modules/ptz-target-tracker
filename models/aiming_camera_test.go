package models

import (
	"image"
	"image/color"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/utils"
)

func TestAimingCameraConfigValidate(t *testing.T) {
	t.Run("missing camera_name", func(t *testing.T) {
		cfg := AimingCameraConfig{}

		required, optional, err := cfg.Validate("components.0")
		if err == nil {
			t.Fatal("Validate() error = nil, want an error")
		}
		if required != nil || optional != nil {
			t.Errorf("Validate() deps = (%v, %v), want (nil, nil)", required, optional)
		}
	})

	t.Run("camera_name is returned as a required dependency", func(t *testing.T) {
		cfg := AimingCameraConfig{CameraName: "my-camera"}

		required, optional, err := cfg.Validate("components.0")
		if err != nil {
			t.Fatalf("Validate() unexpected error: %v", err)
		}
		if len(required) != 1 || required[0] != "my-camera" {
			t.Errorf("Validate() required deps = %v, want [my-camera]", required)
		}
		if optional != nil {
			t.Errorf("Validate() optional deps = %v, want nil", optional)
		}
	})
}

func TestIsImageMimeTypeSupported(t *testing.T) {
	tests := []struct {
		mimeType string
		want     bool
	}{
		{utils.MimeTypeJPEG, true},
		{utils.MimeTypePNG, true},
		{utils.MimeTypeRawRGBA, true},
		{"", true},
		{"image/gif", false},
		{"application/json", false},
		{"image/JPEG", false},
	}

	for _, tt := range tests {
		t.Run("mime type "+tt.mimeType, func(t *testing.T) {
			if got := isImageMimeTypeSupported(tt.mimeType); got != tt.want {
				t.Errorf("isImageMimeTypeSupported(%q) = %v, want %v", tt.mimeType, got, tt.want)
			}
		})
	}
}

func TestDrawCrosshair(t *testing.T) {
	const width, height = 200, 200
	white := color.RGBA{R: 255, G: 255, B: 255, A: 255}
	red := color.RGBA{R: 255, G: 0, B: 0, A: 255}

	src := image.NewRGBA(image.Rect(0, 0, width, height))
	for x := 0; x < width; x++ {
		for y := 0; y < height; y++ {
			src.SetRGBA(x, y, white)
		}
	}

	cam := &aimingCamera{logger: logging.NewTestLogger(t)}
	got := cam.drawCrosshair(src)

	if got.Bounds() != src.Bounds() {
		t.Fatalf("drawCrosshair() bounds = %v, want %v", got.Bounds(), src.Bounds())
	}

	// The crosshair is drawn 100px along each arm and 20px thick at the center.
	tests := []struct {
		name string
		x, y int
		want color.RGBA
	}{
		{"center", 100, 100, red},
		{"along horizontal arm", 40, 100, red},
		{"along vertical arm", 100, 40, red},
		{"just outside horizontal arm thickness", 40, 120, white},
		{"corner is untouched", 5, 5, white},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			r, g, b, a := got.At(tt.x, tt.y).RGBA()
			wr, wg, wb, wa := tt.want.RGBA()
			if r != wr || g != wg || b != wb || a != wa {
				t.Errorf("pixel at (%d, %d) = RGBA(%d, %d, %d, %d), want RGBA(%d, %d, %d, %d)",
					tt.x, tt.y, r>>8, g>>8, b>>8, a>>8, wr>>8, wg>>8, wb>>8, wa>>8)
			}
		})
	}
}

// drawCrosshair must not write through to the image it was handed.
func TestDrawCrosshairDoesNotMutateSource(t *testing.T) {
	src := image.NewRGBA(image.Rect(0, 0, 200, 200))
	white := color.RGBA{R: 255, G: 255, B: 255, A: 255}
	for x := 0; x < 200; x++ {
		for y := 0; y < 200; y++ {
			src.SetRGBA(x, y, white)
		}
	}

	cam := &aimingCamera{logger: logging.NewTestLogger(t)}
	cam.drawCrosshair(src)

	if got := src.RGBAAt(100, 100); got != white {
		t.Errorf("source center pixel = %v, want it left as %v", got, white)
	}
}

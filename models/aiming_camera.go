package models

import (
	"context"
	"errors"
	"image"
	"image/color"
	"image/draw"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
)

var (
	AimingCamera = resource.NewModel("viam", "ptz-target-tracker", "aiming-camera")
)

func init() {
	resource.RegisterComponent(camera.API, AimingCamera,
		resource.Registration[camera.Camera, *AimingCameraConfig]{
			Constructor: newAimingCamera,
		},
	)
}

type AimingCameraConfig struct {
	resource.TriviallyValidateConfig
	CameraName string `json:"camera_name"`
}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *AimingCameraConfig) Validate(path string) ([]string, []string, error) {
	if cfg.CameraName == "" {
		return nil, nil, errors.New("camera_name is required")
	}
	return []string{cfg.CameraName}, nil, nil
}

type aimingCamera struct {
	resource.TriviallyCloseable
	resource.TriviallyReconfigurable
	name           resource.Name
	logger         logging.Logger
	cfg            *AimingCameraConfig
	underlyingCam  camera.Camera
	crosshairColor color.Color
}

func newAimingCamera(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (camera.Camera, error) {
	conf, err := resource.NativeConfig[*AimingCameraConfig](rawConf)
	if err != nil {
		return nil, err
	}

	// Get underlying camera
	cam, err := camera.FromDependencies(deps, conf.CameraName)
	if err != nil {
		return nil, err
	}

	s := &aimingCamera{
		name:          rawConf.ResourceName(),
		logger:        logger,
		cfg:           conf,
		underlyingCam: cam,
	}
	return s, nil
}

func (s *aimingCamera) Name() resource.Name {
	return s.name
}

func (s *aimingCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, errors.New("not implemented")
}

func (s *aimingCamera) GetImage(ctx context.Context) (image.Image, error) {
	return nil, errors.New("get image not implemented")
}

// drawCrosshair draws a crosshair at the center of the image
func (s *aimingCamera) drawCrosshair(img image.Image) image.Image {
	bounds := img.Bounds()
	centerX := bounds.Dx() / 2
	centerY := bounds.Dy() / 2

	// Create a mutable copy of the image
	rgba := image.NewRGBA(bounds)
	draw.Draw(rgba, bounds, img, bounds.Min, draw.Src)

	crosshairColorRed := color.RGBA{R: 255, G: 0, B: 0, A: 255}

	// Draw crosshair lines
	size := 100
	thick := 20

	// Horizontal line
	for x := centerX - size; x <= centerX+size; x++ {
		for dy := -thick / 2; dy <= thick/2; dy++ {
			if x >= 0 && x < bounds.Dx() && centerY+dy >= 0 && centerY+dy < bounds.Dy() {
				rgba.Set(x, centerY+dy, crosshairColorRed)
			}
		}
	}

	// Vertical line
	for y := centerY - size; y <= centerY+size; y++ {
		for dx := -thick / 2; dx <= thick/2; dx++ {
			if y >= 0 && y < bounds.Dy() && centerX+dx >= 0 && centerX+dx < bounds.Dx() {
				rgba.Set(centerX+dx, y, crosshairColorRed)
			}
		}
	}

	return rgba
}

func (s *aimingCamera) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return s.underlyingCam.Geometries(ctx, extra)
}

func (s *aimingCamera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	return nil, camera.ImageMetadata{}, errors.New("single image retrieval not implemented; use Images() instead")
}

func (s *aimingCamera) Images(ctx context.Context, mimeTypes []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	// Get images from underlying camera
	imgs, meta, err := s.underlyingCam.Images(ctx, mimeTypes, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}

	// Create new named images with crosshair overlay
	resultImgs := make([]camera.NamedImage, len(imgs))
	for i, namedImg := range imgs {
		// Get the actual image
		img, err := namedImg.Image(ctx)
		if err != nil {
			return nil, resource.ResponseMetadata{}, err
		}

		// Draw crosshair on it
		imgWithCrosshair := s.drawCrosshair(img)

		// Create new NamedImage
		resultImg, err := camera.NamedImageFromImage(imgWithCrosshair, namedImg.SourceName, namedImg.MimeType())
		if err != nil {
			return nil, resource.ResponseMetadata{}, err
		}
		resultImgs[i] = resultImg
	}

	return resultImgs, meta, nil
}

func (s *aimingCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	return s.underlyingCam.NextPointCloud(ctx, extra)
}

func (s *aimingCamera) Properties(ctx context.Context) (camera.Properties, error) {
	// Return properties from underlying camera
	return s.underlyingCam.Properties(ctx)
}

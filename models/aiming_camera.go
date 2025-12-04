package models

import (
	"bytes"
	"context"
	"errors"
	"fmt"
	"image"
	"image/color"
	"image/draw"
	"image/jpeg"
	"image/png"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

var (
	ModelAimingCamera = resource.NewModel("viam", "ptz-target-tracker", "aiming-camera")
)

func init() {
	resource.RegisterComponent(camera.API, ModelAimingCamera,
		resource.Registration[camera.Camera, *AimingCameraConfig]{
			Constructor: newAimingCamera,
		},
	)
}

type AimingCameraConfig struct {
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
	cam, err := camera.FromProvider(deps, conf.CameraName)
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
	draw.Draw(rgba, image.Rect(centerX-size, centerY-thick/2, centerX+size, centerY+thick/2), &image.Uniform{crosshairColorRed}, image.Point{}, draw.Src)

	// Vertical line
	draw.Draw(rgba, image.Rect(centerX-thick/2, centerY-size, centerX+thick/2, centerY+size), &image.Uniform{crosshairColorRed}, image.Point{}, draw.Src)

	return rgba
}

func (s *aimingCamera) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	geometry, err := s.underlyingCam.Geometries(ctx, extra)
	if err != nil {
		return nil, fmt.Errorf("error requesting geometries from underlying camera: %w", err)
	}
	return geometry, nil
}

func (s *aimingCamera) Image(ctx context.Context, mimeType string, extra map[string]interface{}) ([]byte, camera.ImageMetadata, error) {
	// Get images from underlying camera
	filterSourceNames := []string{}
	imgs, _, err := s.Images(ctx, filterSourceNames, extra)
	if len(imgs) == 0 {
		return nil, camera.ImageMetadata{}, fmt.Errorf("No images returned from underlying camera")
	}
	if len(imgs) > 1 {
		s.logger.Warnf("Multiple images returned from underlying camera, only the first will be processed")
	}

	img, err := imgs[0].Bytes(ctx)
	if err != nil {
		return nil, camera.ImageMetadata{}, fmt.Errorf("error when getting bytes from named image: %w", err)
	}

	// Decode image
	decodedImg, _, err := image.Decode(bytes.NewReader(img))
	if err != nil {
		return nil, camera.ImageMetadata{}, fmt.Errorf("error when decoding image: %w", err)
	}

	// Draw crosshair on it
	imgWithCrosshair := s.drawCrosshair(decodedImg)

	// Encode back to []byte
	var buf bytes.Buffer
	switch mimeType {
	case utils.MimeTypePNG:
		err = png.Encode(&buf, imgWithCrosshair)
	default:
		// Default to JPEG for unknown types or explicit JPEG request
		err = jpeg.Encode(&buf, imgWithCrosshair, nil)
	}
	if err != nil {
		return nil, camera.ImageMetadata{}, fmt.Errorf("error when encoding image with crosshair: %w", err)
	}

	outputMetadata := camera.ImageMetadata{
		MimeType: mimeType,
	}

	return buf.Bytes(), outputMetadata, nil
}

func isImageMimeTypeSupported(mimeType string) bool {
	switch mimeType {
	case utils.MimeTypeJPEG, utils.MimeTypePNG, utils.MimeTypeRawRGBA, "":
		return true
	default:
		return false
	}
}

func (s *aimingCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	// Get images from underlying camera
	imgs, meta, err := s.underlyingCam.Images(ctx, filterSourceNames, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, fmt.Errorf("error when requesting images from underlying camera: %w", err)
	}

	// Create new named images with crosshair overlay
	for _, namedImg := range imgs {
		if !isImageMimeTypeSupported(namedImg.MimeType()) {
			continue
		}
		// Get the actual image
		img, err := namedImg.Image(ctx)
		if err != nil {
			return nil, resource.ResponseMetadata{}, fmt.Errorf("error when getting image from named image: %w", err)
		}

		// Draw crosshair on it
		imgWithCrosshair := s.drawCrosshair(img)

		// Create new NamedImage
		resultImg, err := camera.NamedImageFromImage(imgWithCrosshair, namedImg.SourceName, namedImg.MimeType())
		if err != nil {
			return nil, resource.ResponseMetadata{}, fmt.Errorf("error when creating named image from image with crosshair: %w", err)
		}
		// Just return the first supported image with crosshair
		return []camera.NamedImage{resultImg}, meta, nil
	}
	return nil, resource.ResponseMetadata{}, fmt.Errorf("no supported image mime types found from underlying camera")
}

func (s *aimingCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	// Return point cloud from underlying camera
	pointcloud, err := s.underlyingCam.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, fmt.Errorf("error when requesting point cloud from underlying camera: %w", err)
	}
	return pointcloud, nil
}

func (s *aimingCamera) Properties(ctx context.Context) (camera.Properties, error) {
	// Return properties from underlying camera
	properties, err := s.underlyingCam.Properties(ctx)
	if err != nil {
		return camera.Properties{}, fmt.Errorf("error when requesting properties from underlying camera: %w", err)
	}
	return properties, nil
}

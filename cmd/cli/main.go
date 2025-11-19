package main

import (
	"context"
	ptztargettracker "ptztargettracker"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	ctx := context.Background()
	logger := logging.NewLogger("cli")

	deps := resource.Dependencies{}
	// can load these from a remote machine if you need

	cfg := ptztargettracker.Config{
		TargetComponentName: "target_component_name",
		PTZCameraName:       "ptz_camera",
		OnvifPTZClientName:  "onvif-ptz-client",
		UpdateRateHz:        10.0,
		EnableOnStart:       true,
	}

	thing, err := ptztargettracker.NewPoseTracker(ctx, deps, genericservice.Named("foo"), &cfg, logger)
	if err != nil {
		return err
	}
	defer thing.Close(ctx)

	return nil
}

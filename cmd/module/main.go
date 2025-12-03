package main

import (
	"ptztargettracker/models"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(
		resource.APIModel{API: generic.API, Model: models.ModelComponentTracker},
		resource.APIModel{API: camera.API, Model: models.ModelAimingCamera},
	)
}

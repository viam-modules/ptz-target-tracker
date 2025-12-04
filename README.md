# Module ptz-target-tracker 

Provide a description of the purpose of the module and any relevant information.

## Model viam:ptz-target-tracker:component-tracker

This component allows users to use a ptz camera with Onvif AbsoluteMove capability to track a moving component whose position can be accurately retrieve from viam's frame system.

### Configuration
The following attribute template can be used to configure this model:

```json
{
  "target_component_name": <string>,
  "ptz_camera_name": <string>,
  "onvif_ptz_client_name": <string>,
  "pan_min_deg": <float>,
  "pan_max_deg": <float>,
  "tilt_min_deg": <float>,
  "tilt_max_deg": <float>,
  "min_zoom_distance_mm": <float>,
  "max_zoom_distance_mm": <float>,
  "update_rate_hz": <float>,
  "deadzone": <float>,
  "calibration": {
    "is_calibrated": <bool>,
    "pan_poly_coeffs": <array>,
    "tilt_poly_coeffs": <array>,
  },
  "enable_on_start": <bool>
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `target_component_name` | string  | Required  | Name of the target component to track |
| `ptz_camera_name` | string | Required  | Name of the PTZ Camera to be used |
| `onvif_ptz_client_name` | string | Required  | Name of the PTZ Client that is connected to the PTZ camera used |
| `pan_min_deg` | float | Required  | Minimum value in degrees of the pan of the ptz camera, can be obtained from the camera specs |
| `pan_max_deg` | float | Required  | Maximum value in degrees of the pan of the ptz camera, can be obtained from the camera specs |
| `tilt_min_deg` | float | Required  | Minimum value in degrees of the tilt of the ptz camera, can be obtained from the camera specs |
| `tilt_max_deg` | float | Required  | Maximum value in degrees of the tilt of the ptz camera, can be obtained from the camera specs |
| `min_zoom_distance_mm` | float | Required  | Distance in mm from the target to the camera where the minimum zoom should be applied |
| `max_zoom_distance_mm` | float | Required  | Distance in mm from the target to the camera where the maximum zoom should be applied |
| `update_rate_hz` | float | Required  | Rate at which this module tracks the target |
| `deadzone` | float | Optional  | To avoid overtracking (and making the image feed to jumpy), this parameter can be specified, if the difference between the current normalized Pan, Tilt & Zoom and the next one to be applied is less than this threshold, no PTZ command is sent  |
| `calibration` | object | Optional  | Corresponds to the coefficients of the polynomial function that maps target position to PTZ values, can be obtained via calibration and left here as a config parameter from that point onwards |
| `enable_on_start` | bool | Optional  | Instructs this tracker to start tracking automatically as soon as this component is loaded |


#### Example Configuration
```json
{
  "target_component_name": "my-arm-end-effector",
  "ptz_camera_name": "ptz-camera-1",
  "onvif_ptz_client_name": "ptz-client-1",
  "pan_min_deg": 0,
  "pan_max_deg": 355,
  "tilt_min_deg": 5,
  "tilt_max_deg": 90,
  "min_zoom_distance_mm": 100,
  "max_zoom_distance_mm": 2000,
  "update_rate_hz": 5,
  "deadzone": 0,
  "calibration": {
    "is_calibrated": true,
    "pan_poly_coeffs": [
      -2.163750357403037e-8,
      9.973991590816574e-8,
      -1.0834180068787597e-7,
      -8.654151476530682e-9,
      7.422166501239921e-9,
      -1.1453232528444476e-8,
      -0.00006229515340084487,
      -0.0002892745093281764,
      0.00003565702583424444,
      0.4398168759830914
    ],
    "tilt_poly_coeffs": [
      -1.6551322560686657e-8,
      1.9440629816393203e-7,
      -3.351297267301809e-7,
      1.0531050625438366e-8,
      -1.7547216510394483e-7,
      7.444328368378235e-8,
      -0.00013072065397707472,
      -0.00039794633597679466,
      -0.0006247769655574049,
      0.5310472398742934
    ],
  },
  "enable_on_start": true
}
```

### DoCommand
Gets the list of all the calibration samples currently saved by this module, useful to save them by the user as they are lost on module restart
```json
{
  "command": "get-calibration-samples"
}
```

Loads a list of previously captured PTZ samples, useful to improve this calibration if there are spots where the tracking is not precise enough due to not having used a sample on that spot to calculate the polynomial function
```json
{
  "command": "load-calibration-samples"
}
```

Pushes a new sample to our TargetPosition/PTZValues samples list, it will capture the current PTZ value and the current targetPosition and bundle them together as one calibration sample
```json
{
  "command": "push-sample"
}
```

Pops the last captured sample, useful is the user made a mistake in capturing the last sample by not having the PTZ camera precisely aiming at the target
```json
{
  "command": "pop-sample"
}
```

Clears all current calibration samples and the polynomial calibration function
```json
{
  "command": "clear-calibration"
}
```

Computes the polynomial calibration function based on the samples currently stored in the module, this function is used then to derive the Pan and Tilt values that corresponds to a position in space of the target
```json
{
  "command": "compute-polynomial"
}
```
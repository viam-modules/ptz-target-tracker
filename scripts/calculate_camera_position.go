package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"math"
	"os"

	"github.com/erh/vmodutils/touch"
	"github.com/golang/geo/r3"
	"github.com/joho/godotenv"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/utils/rpc"
	"gonum.org/v1/gonum/mat"
)

func calculateCameraPosition(envFile string) {
	// Load .env file
	if envFile != "" {
		if err := godotenv.Load(envFile); err != nil {
			fmt.Printf("Error loading %s: %v\n", envFile, err)
			fmt.Println("Using system environment variables instead")
		} else {
			fmt.Printf("✅ Loaded environment from: %s\n", envFile)
		}
	} else {
		// Try to load default .env file
		if err := godotenv.Load(); err != nil {
			fmt.Println("Warning: .env file not found, using system environment variables")
		} else {
			fmt.Println("✅ Loaded environment from: .env")
		}
	}

	logger := logging.NewDebugLogger("client")

	// Get credentials from environment
	robotAddress := os.Getenv("VIAM_ROBOT_ADDRESS")
	apiKeyID := os.Getenv("VIAM_API_KEY_ID")
	apiKey := os.Getenv("VIAM_API_KEY")

	if robotAddress == "" || apiKeyID == "" || apiKey == "" {
		logger.Fatal("Missing required environment variables: VIAM_ROBOT_ADDRESS, VIAM_API_KEY_ID, VIAM_API_KEY")
	}

	machine, err := client.New(
		context.Background(),
		robotAddress,
		logger,
		client.WithDialOptions(rpc.WithEntityCredentials(
			apiKeyID,
			rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			})),
	)
	if err != nil {
		logger.Fatal(err)
	}

	defer machine.Close(context.Background())
	logger.Info("Resources:")
	logger.Info(machine.ResourceNames())

	// Get target component name from environment (default to "sander")
	targetComponentName := os.Getenv("TARGET_COMPONENT_NAME")
	if targetComponentName == "" {
		targetComponentName = "sander"
	}

	// LINE 1: Center of camera frame
	fmt.Println("=== LINE 1: Center of frame ===")
	line1Points := []r3.Vector{}
	for i := 0; i < 3; i++ {
		fmt.Printf("Move %s to CENTER of camera frame (position %d/3), then press Enter\n", targetComponentName, i+1)
		fmt.Scanln()

		targetPose := getTargetPose(context.Background(), machine, targetComponentName)
		if targetPose == nil {
			logger.Error(err)
			return
		}
		logger.Infof("target EndPosition return value: %+v", targetPose)
		pos := targetPose.Pose().Point()
		line1Points = append(line1Points, pos)
		logger.Infof("Line 1, Point %d: (%f, %f, %f)", i+1, pos.X, pos.Y, pos.Z)
	}

	// LINE 2: Different location in frame (e.g., top-left corner)
	fmt.Println("\n=== LINE 2: Same location in frame (e.g., top-left) ===")
	line2Points := []r3.Vector{}
	for i := 0; i < 3; i++ {
		fmt.Printf("Move %s to TOP-LEFT of camera frame (position %d/3), then press Enter\n", targetComponentName, i+1)
		fmt.Scanln()

		targetPose := getTargetPose(context.Background(), machine, targetComponentName)
		if targetPose == nil {
			logger.Error(err)
			return
		}
		logger.Infof("target EndPosition return value: %+v", targetPose)
		pos := targetPose.Pose().Point()
		line2Points = append(line2Points, pos)
		logger.Infof("Line 2, Point %d: (%f, %f, %f)", i+1, pos.X, pos.Y, pos.Z)
	}

	// Fit lines through both sets of points
	p1, d1 := fitLine3D(line1Points)
	p2, d2 := fitLine3D(line2Points)

	logger.Infof("Line 1: point=(%f, %f, %f), dir=(%f, %f, %f)",
		p1.X, p1.Y, p1.Z, d1.X, d1.Y, d1.Z)
	logger.Infof("Line 2: point=(%f, %f, %f), dir=(%f, %f, %f)",
		p2.X, p2.Y, p2.Z, d2.X, d2.Y, d2.Z)

	// Find intersection of two 3D lines
	cameraPos, distance, err := intersectLines3D(p1, d1, p2, d2)
	if err != nil {
		logger.Error(err)
		return
	}

	logger.Infof("Camera position: (%f, %f, %f)", cameraPos.X, cameraPos.Y, cameraPos.Z)
	logger.Infof("Lines miss distance: %f mm (should be ~0)", distance)

	if distance > 50.0 { // 50mm tolerance
		logger.Warn("Lines don't intersect well - check measurements!")
	}
}

// Find closest point between two 3D lines (handles skew lines)
func intersectLines3D(p1, d1, p2, d2 r3.Vector) (intersection r3.Vector, distance float64, err error) {
	// Line 1: P1 + s * d1
	// Line 2: P2 + t * d2
	// Find s and t that minimize |P1 + s*d1 - P2 - t*d2|

	// Vector between line points
	w := p1.Sub(p2)

	a := d1.Dot(d1) // Always >= 0
	b := d1.Dot(d2)
	c := d2.Dot(d2) // Always >= 0
	d := d1.Dot(w)
	e := d2.Dot(w)

	denom := a*c - b*b

	if math.Abs(denom) < 1e-10 {
		return r3.Vector{}, 0, errors.New("lines are parallel")
	}

	// Parameters for closest points
	s := (b*e - c*d) / denom
	t := (a*e - b*d) / denom

	// Closest points on each line
	closest1 := p1.Add(d1.Mul(s))
	closest2 := p2.Add(d2.Mul(t))

	// Intersection point (midpoint of closest points)
	intersection = closest1.Add(closest2).Mul(0.5)

	// Distance between lines (should be ~0 if they actually intersect)
	distance = closest1.Sub(closest2).Norm()

	return intersection, distance, nil
}

// Fit a 3D line through points using PCA
func fitLine3D(points []r3.Vector) (point r3.Vector, direction r3.Vector) {
	if len(points) < 2 {
		// Not enough points, return zero
		return r3.Vector{}, r3.Vector{X: 0, Y: 0, Z: 1}
	}

	// 1. Calculate centroid
	centroid := r3.Vector{}
	for _, p := range points {
		centroid = centroid.Add(p)
	}
	centroid = centroid.Mul(1.0 / float64(len(points)))

	// 2. Build covariance matrix
	n := len(points)
	covData := make([]float64, 9) // 3x3 matrix

	for _, p := range points {
		dx := p.X - centroid.X
		dy := p.Y - centroid.Y
		dz := p.Z - centroid.Z

		covData[0] += dx * dx // XX
		covData[1] += dx * dy // XY
		covData[2] += dx * dz // XZ
		covData[3] += dy * dx // YX
		covData[4] += dy * dy // YY
		covData[5] += dy * dz // YZ
		covData[6] += dz * dx // ZX
		covData[7] += dz * dy // ZY
		covData[8] += dz * dz // ZZ
	}

	// Normalize by n
	for i := range covData {
		covData[i] /= float64(n)
	}

	// 3. Find eigenvector of largest eigenvalue
	cov := mat.NewSymDense(3, covData)

	var eig mat.EigenSym
	ok := eig.Factorize(cov, true) // true = compute eigenvectors
	if !ok {
		// Fallback: use simple line from first to last point
		direction = points[len(points)-1].Sub(points[0])
		direction = direction.Mul(1.0 / direction.Norm())
		return centroid, direction
	}

	// Get eigenvalues and eigenvectors
	values := eig.Values(nil)

	// Find index of largest eigenvalue
	maxIdx := 0
	maxVal := values[0]
	for i := 1; i < len(values); i++ {
		if values[i] > maxVal {
			maxVal = values[i]
			maxIdx = i
		}
	}

	// Get corresponding eigenvector
	var evec mat.Dense
	eig.VectorsTo(&evec)

	direction = r3.Vector{
		X: evec.At(0, maxIdx),
		Y: evec.At(1, maxIdx),
		Z: evec.At(2, maxIdx),
	}

	// Normalize (should already be normalized, but just in case)
	direction = direction.Mul(1.0 / direction.Norm())

	// Point on line is the centroid
	point = centroid

	return point, direction
}

func getTargetPose(ctx context.Context, robotClient *client.RobotClient, targetComponentName string) *referenceframe.PoseInFrame {
	fsc, err := robotClient.FrameSystemConfig(ctx)
	if err != nil {
		fmt.Printf("Failed to get frame system config: %v", err.Error())
		return nil
	}
	targetFramePart := touch.FindPart(fsc, targetComponentName)
	if targetFramePart == nil {
		fmt.Println("can't find frame for", targetComponentName)
		return nil
	}
	targetPose, err := robotClient.GetPose(ctx, targetFramePart.FrameConfig.Name(), "", []*referenceframe.LinkInFrame{}, map[string]interface{}{})
	if err != nil {
		fmt.Printf("Failed to get pose: %v", err.Error())
		return nil
	}

	return targetPose
}

func main() {
	// Parse command-line flags
	envFile := flag.String("env", "", "Path to .env file (e.g., .env.robot1)")
	flag.Parse()

	calculateCameraPosition(*envFile)
}

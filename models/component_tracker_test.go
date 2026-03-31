package models

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"gonum.org/v1/gonum/mat"
)

// realCalibrationSamples are the 14 manually-collected samples from the NJ2 session.
// X/Y/Z are in millimetres; Pan/Tilt are in normalised [-1, 1] range.
var realCalibrationSamples = []TrackingSample{
	{TargetPos: r3.Vector{X: -976.8718904185582, Y: 688.0726449734655, Z: -308.44922857823883}, Pan: 0.055436619718309925, Tilt: -0.07866666666666666},
	{TargetPos: r3.Vector{X: 0.0027494616486123643, Y: 688.0753683511571, Z: -308.4450590964292}, Pan: 0.026591549295774675, Tilt: -0.062222222222222186},
	{TargetPos: r3.Vector{X: 976.8688846321818, Y: 688.0757959054829, Z: -308.4540781553749}, Pan: -0.02783098591549304, Tilt: 0.0006666666666665932},
	{TargetPos: r3.Vector{X: -976.8658485130267, Y: 924.9771681428174, Z: -308.4452264417055}, Pan: 0.018084507042253617, Tilt: -0.1808888888888889},
	{TargetPos: r3.Vector{X: 0.004918343033715482, Y: 924.9687027273684, Z: -308.4499781965605}, Pan: -0.01645070422535211, Tilt: -0.16488888888888897},
	{TargetPos: r3.Vector{X: 976.8630978695658, Y: 924.9747212402899, Z: -308.4484895758794}, Pan: -0.10377464788732405, Tilt: -0.009333333333333416},
	{TargetPos: r3.Vector{X: -976.8707590562691, Y: 1161.855840073072, Z: -236.69815720617817}, Pan: -0.015830985915493034, Tilt: -0.21199999999999997},
	{TargetPos: r3.Vector{X: -0.0015030975220719314, Y: 1161.8575743749461, Z: -236.69901249620455}, Pan: -0.06907042253521123, Tilt: -0.18533333333333346},
	{TargetPos: r3.Vector{X: 976.8633179048849, Y: 1161.8523691791802, Z: -308.4671318536604}, Pan: -0.16602816901408457, Tilt: 0.02866666666666662},
	{TargetPos: r3.Vector{X: -1362.4651888480619, Y: 688.0764581692226, Z: -308.4484873618165}, Pan: 0.06461971830985913, Tilt: -0.0868888888888888},
	{TargetPos: r3.Vector{X: 1362.4652986249155, Y: 688.0791876924957, Z: -380.14715798051566}, Pan: -0.06935211267605634, Tilt: 0.12711111111111117},
	{TargetPos: r3.Vector{X: -976.8724222707157, Y: 688.0793828334199, Z: 523.1470553766869}, Pan: 0.104281690140845, Tilt: -0.42977777777777787},
	{TargetPos: r3.Vector{X: -0.006293899741256411, Y: 688.0849976732654, Z: 523.1496141895727}, Pan: 0.09256338028169019, Tilt: -0.5402222222222222},
	{TargetPos: r3.Vector{X: 976.8735527165312, Y: 688.0799303716159, Z: 523.1369591289011}, Pan: 0.004507042253521054, Tilt: -0.7706666666666665},
}

// fitPolynomialSingleNormalEquations is the OLD (buggy) implementation.
// It builds XᵀX and Xᵀy (the normal equations), then solves the resulting
// 10×10 square system with QR.  When feature values are large (x~1000 →
// x²~10⁶), XᵀX has entries ~10¹² and its condition number is so high that
// the solver returns all-zeros without reporting an error.
func fitPolynomialSingleNormalEquations(samples []TrackingSample, getValue func(TrackingSample) float64) []float64 {
	var XtX [10][10]float64
	var XtY [10]float64

	for _, s := range samples {
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

	// Solve XᵀX · c = XᵀY with QR on the already-squared matrix.
	aData := make([]float64, 100)
	for i := 0; i < 10; i++ {
		for j := 0; j < 10; j++ {
			aData[i*10+j] = XtX[i][j]
		}
	}
	bData := make([]float64, 10)
	copy(bData, XtY[:])

	aMat := mat.NewDense(10, 10, aData)
	bMat := mat.NewDense(10, 1, bData)

	var qr mat.QR
	qr.Factorize(aMat)

	var result mat.Dense
	if err := qr.SolveTo(&result, false, bMat); err != nil {
		return make([]float64, 10)
	}

	coeffs := make([]float64, 10)
	for i := 0; i < 10; i++ {
		coeffs[i] = result.At(i, 0)
	}
	return coeffs
}

// fitPolynomialSingleDirectQR is the NEW (fixed) implementation.
// It builds the full n×10 design matrix and solves the overdetermined
// least-squares problem directly, without squaring the condition number.
func fitPolynomialSingleDirectQR(samples []TrackingSample, getValue func(TrackingSample) float64) []float64 {
	n := len(samples)
	xData := make([]float64, n*10)
	yData := make([]float64, n)

	for i, s := range samples {
		x, y, z := s.TargetPos.X, s.TargetPos.Y, s.TargetPos.Z
		features := [10]float64{
			x * x, y * y, z * z,
			x * y, x * z, y * z,
			x, y, z, 1,
		}
		for j := 0; j < 10; j++ {
			xData[i*10+j] = features[j]
		}
		yData[i] = getValue(s)
	}

	xMat := mat.NewDense(n, 10, xData)
	yMat := mat.NewDense(n, 1, yData)

	var qr mat.QR
	qr.Factorize(xMat)

	var result mat.Dense
	if err := qr.SolveTo(&result, false, yMat); err != nil {
		return make([]float64, 10)
	}

	coeffs := make([]float64, 10)
	for i := 0; i < 10; i++ {
		coeffs[i] = result.At(i, 0)
	}
	return coeffs
}

func evalPoly(coeffs []float64, pos r3.Vector) float64 {
	x, y, z := pos.X, pos.Y, pos.Z
	features := [10]float64{
		x * x, y * y, z * z,
		x * y, x * z, y * z,
		x, y, z, 1,
	}
	var v float64
	for i, c := range coeffs {
		v += c * features[i]
	}
	return v
}

func meanAbsError(samples []TrackingSample, coeffs []float64, getValue func(TrackingSample) float64) float64 {
	var sum float64
	for _, s := range samples {
		sum += math.Abs(evalPoly(coeffs, s.TargetPos) - getValue(s))
	}
	return sum / float64(len(samples))
}

func allZero(coeffs []float64) bool {
	for _, c := range coeffs {
		if c != 0 {
			return false
		}
	}
	return true
}

// TestNormalEquationsProducesZeroCoeffs shows that the old approach silently
// returns all-zero coefficients when coordinates are in the millimetre range
// (~1000 mm), because forming XᵀX squares the condition number to the point
// where the QR solve cannot distinguish a non-trivial solution from zero.
func TestNormalEquationsProducesZeroCoeffs(t *testing.T) {
	getPan := func(s TrackingSample) float64 { return s.Pan }
	getTilt := func(s TrackingSample) float64 { return s.Tilt }

	panCoeffs := fitPolynomialSingleNormalEquations(realCalibrationSamples, getPan)
	tiltCoeffs := fitPolynomialSingleNormalEquations(realCalibrationSamples, getTilt)

	if !allZero(panCoeffs) {
		t.Logf("pan coeffs (normal equations): %v", panCoeffs)
		t.Error("expected normal-equations approach to return all-zero pan coefficients due to ill-conditioning, but got non-zero values")
	}
	if !allZero(tiltCoeffs) {
		t.Logf("tilt coeffs (normal equations): %v", tiltCoeffs)
		t.Error("expected normal-equations approach to return all-zero tilt coefficients due to ill-conditioning, but got non-zero values")
	}

	// With all-zero coefficients, the predicted value is always 0.
	// The "error" therefore equals the mean absolute value of the observations
	// themselves — it tells us nothing about fit quality.
	panErr := meanAbsError(realCalibrationSamples, panCoeffs, getPan)
	tiltErr := meanAbsError(realCalibrationSamples, tiltCoeffs, getTilt)
	t.Logf("normal-equations avg error  — pan: %.5f  tilt: %.5f  (these are just mean(|obs|), not a real fit error)", panErr, tiltErr)
}

// TestDirectQRProducesNonZeroCoeffs shows that the fixed approach produces
// non-zero coefficients and a substantially lower prediction error on the
// same samples.
func TestDirectQRProducesNonZeroCoeffs(t *testing.T) {
	getPan := func(s TrackingSample) float64 { return s.Pan }
	getTilt := func(s TrackingSample) float64 { return s.Tilt }

	panCoeffs := fitPolynomialSingleDirectQR(realCalibrationSamples, getPan)
	tiltCoeffs := fitPolynomialSingleDirectQR(realCalibrationSamples, getTilt)

	if allZero(panCoeffs) {
		t.Error("direct-QR approach returned all-zero pan coefficients")
	}
	if allZero(tiltCoeffs) {
		t.Error("direct-QR approach returned all-zero tilt coefficients")
	}

	panErr := meanAbsError(realCalibrationSamples, panCoeffs, getPan)
	tiltErr := meanAbsError(realCalibrationSamples, tiltCoeffs, getTilt)
	t.Logf("direct-QR avg error         — pan: %.5f  tilt: %.5f", panErr, tiltErr)

	// The fit error must be strictly less than the mean-absolute-observation
	// baseline (what the all-zero model achieves).  For a polynomial with 10
	// parameters fitted on 14 samples this should be a large improvement.
	panBaseline := meanAbsError(realCalibrationSamples, make([]float64, 10), getPan)
	tiltBaseline := meanAbsError(realCalibrationSamples, make([]float64, 10), getTilt)

	if panErr >= panBaseline {
		t.Errorf("direct-QR pan error %.5f is not better than zero-model baseline %.5f", panErr, panBaseline)
	}
	if tiltErr >= tiltBaseline {
		t.Errorf("direct-QR tilt error %.5f is not better than zero-model baseline %.5f", tiltErr, tiltBaseline)
	}
}

// TestDirectQRBetterThanNormalEquations is the head-to-head comparison.
func TestDirectQRBetterThanNormalEquations(t *testing.T) {
	getPan := func(s TrackingSample) float64 { return s.Pan }
	getTilt := func(s TrackingSample) float64 { return s.Tilt }

	oldPan := fitPolynomialSingleNormalEquations(realCalibrationSamples, getPan)
	oldTilt := fitPolynomialSingleNormalEquations(realCalibrationSamples, getTilt)
	newPan := fitPolynomialSingleDirectQR(realCalibrationSamples, getPan)
	newTilt := fitPolynomialSingleDirectQR(realCalibrationSamples, getTilt)

	oldPanErr := meanAbsError(realCalibrationSamples, oldPan, getPan)
	oldTiltErr := meanAbsError(realCalibrationSamples, oldTilt, getTilt)
	newPanErr := meanAbsError(realCalibrationSamples, newPan, getPan)
	newTiltErr := meanAbsError(realCalibrationSamples, newTilt, getTilt)

	t.Logf("pan  error — old (normal equations): %.5f   new (direct QR): %.5f", oldPanErr, newPanErr)
	t.Logf("tilt error — old (normal equations): %.5f   new (direct QR): %.5f", oldTiltErr, newTiltErr)

	if newPanErr >= oldPanErr {
		t.Errorf("expected direct-QR pan error (%.5f) < normal-equations pan error (%.5f)", newPanErr, oldPanErr)
	}
	if newTiltErr >= oldTiltErr {
		t.Errorf("expected direct-QR tilt error (%.5f) < normal-equations tilt error (%.5f)", newTiltErr, oldTiltErr)
	}
}

// nj1CalibrationSamples are from the NJ1 arm.  The old normal-equations
// approach happened to return non-zero coefficients here because the Y
// coordinates span a much wider range (≈0 to 1414 mm, including a near-zero
// sample), which keeps the condition number of XᵀX below the failure
// threshold.  The new direct-QR approach works correctly in both cases.
var nj1CalibrationSamples = []TrackingSample{
	{TargetPos: r3.Vector{X: -953.3491432714728, Y: 585.7907819705993, Z: -320.9456137937267}, Pan: 0.11949295774647874, Tilt: -0.2544444444444445},
	{TargetPos: r3.Vector{X: 12.22459272870433, Y: 585.7926298834248, Z: -320.95152733378967}, Pan: -0.045971830985915396, Tilt: -0.2786666666666666},
	{TargetPos: r3.Vector{X: 977.7879535582074, Y: 585.8060098913409, Z: -320.95546385829954}, Pan: -0.08957746478873242, Tilt: -0.2646666666666666},
	{TargetPos: r3.Vector{X: 12.22488897333005, Y: 867.3013916066554, Z: -320.9473680048485}, Pan: 0.04552112676056341, Tilt: -0.3255555555555555},
	{TargetPos: r3.Vector{X: 977.8060635537685, Y: 867.2974907123339, Z: -320.93781046256765}, Pan: -0.040338028169013995, Tilt: -0.30400000000000005},
	{TargetPos: r3.Vector{X: 12.228311203200898, Y: 1148.8097034906702, Z: -249.75256914293897}, Pan: 0.13295774647887326, Tilt: -0.34711111111111126},
	{TargetPos: r3.Vector{X: 977.7903099911653, Y: 1148.8187494546353, Z: -249.7519567186808}, Pan: 0.021408450704225368, Tilt: -0.3624444444444443},
	{TargetPos: r3.Vector{X: -1406.8986145715799, Y: 585.7936521746398, Z: -392.0036706664391}, Pan: 0.2847887323943662, Tilt: 0.21666666666666656},
	{TargetPos: r3.Vector{X: 1406.9009916311277, Y: 585.7906685371239, Z: -320.95291264917887}, Pan: -0.10180281690140847, Tilt: -0.26088888888888895},
	{TargetPos: r3.Vector{X: -977.8033613173814, Y: 585.7910155183789, Z: 516.605255854374}, Pan: 0.028000000000000025, Tilt: -1.0},
	{TargetPos: r3.Vector{X: -0.002608286516959529, Y: 585.7915039454213, Z: 516.600925701184}, Pan: -0.2205070422535211, Tilt: -0.7248888888888889},
	{TargetPos: r3.Vector{X: 977.7930131422801, Y: 585.7987921578524, Z: 516.6073211556032}, Pan: -0.20845070422535208, Tilt: -0.5600000000000002},
	{TargetPos: r3.Vector{X: 0.001196246304537493, Y: 1414.5796180617308, Z: 518.6079538555016}, Pan: 0.37707042253521134, Tilt: -0.6728888888888889},
	{TargetPos: r3.Vector{X: 1550.001978982877, Y: 0.00020836924961295565, Z: -114.99952368128382}, Pan: -0.36760563380281686, Tilt: -0.12177777777777776},
}

// TestNJ1NormalEquationsAccidentallyWorks shows that the old approach
// produced non-zero coefficients for NJ1 samples.  This was numerical
// luck: the wide Y spread (≈0 to 1414 mm) kept XᵀX just barely
// invertible, unlike NJ2 where Y was clustered at only three levels.
func TestNJ1NormalEquationsAccidentallyWorks(t *testing.T) {
	getPan := func(s TrackingSample) float64 { return s.Pan }
	getTilt := func(s TrackingSample) float64 { return s.Tilt }

	panCoeffs := fitPolynomialSingleNormalEquations(nj1CalibrationSamples, getPan)
	tiltCoeffs := fitPolynomialSingleNormalEquations(nj1CalibrationSamples, getTilt)

	if allZero(panCoeffs) {
		t.Error("expected normal-equations to accidentally produce non-zero pan coefficients for NJ1 data")
	}
	if allZero(tiltCoeffs) {
		t.Error("expected normal-equations to accidentally produce non-zero tilt coefficients for NJ1 data")
	}

	panErr := meanAbsError(nj1CalibrationSamples, panCoeffs, getPan)
	tiltErr := meanAbsError(nj1CalibrationSamples, tiltCoeffs, getTilt)
	t.Logf("NJ1 normal-equations avg error — pan: %.5f  tilt: %.5f", panErr, tiltErr)
}

// TestNJ1DirectQREqualsOrBetterThanNormalEquations confirms that the new
// approach matches or improves on the accidentally-working NJ1 result.
func TestNJ1DirectQREqualsOrBetterThanNormalEquations(t *testing.T) {
	getPan := func(s TrackingSample) float64 { return s.Pan }
	getTilt := func(s TrackingSample) float64 { return s.Tilt }

	oldPanErr := meanAbsError(nj1CalibrationSamples, fitPolynomialSingleNormalEquations(nj1CalibrationSamples, getPan), getPan)
	oldTiltErr := meanAbsError(nj1CalibrationSamples, fitPolynomialSingleNormalEquations(nj1CalibrationSamples, getTilt), getTilt)
	newPanErr := meanAbsError(nj1CalibrationSamples, fitPolynomialSingleDirectQR(nj1CalibrationSamples, getPan), getPan)
	newTiltErr := meanAbsError(nj1CalibrationSamples, fitPolynomialSingleDirectQR(nj1CalibrationSamples, getTilt), getTilt)

	t.Logf("NJ1 pan  error — old: %.5f   new: %.5f", oldPanErr, newPanErr)
	t.Logf("NJ1 tilt error — old: %.5f   new: %.5f", oldTiltErr, newTiltErr)

	if newPanErr > oldPanErr*1.01 { // 1% tolerance for floating-point noise
		t.Errorf("direct-QR pan error (%.5f) is worse than normal-equations (%.5f)", newPanErr, oldPanErr)
	}
	if newTiltErr > oldTiltErr*1.01 {
		t.Errorf("direct-QR tilt error (%.5f) is worse than normal-equations (%.5f)", newTiltErr, oldTiltErr)
	}
}

// TestFitPolynomialSingleExactRecovery verifies that when the observations
// were generated by a known polynomial, the direct-QR fit recovers those
// coefficients accurately (residual error near machine epsilon).
func TestFitPolynomialSingleExactRecovery(t *testing.T) {
	// True coefficients: pan = 0.3·x + 0.1·z + 0.05 (all others zero)
	trueCoeffs := [10]float64{0, 0, 0, 0, 0, 0, 0.3, 0, 0.1, 0.05}

	samples := make([]TrackingSample, len(realCalibrationSamples))
	for i, s := range realCalibrationSamples {
		x, y, z := s.TargetPos.X, s.TargetPos.Y, s.TargetPos.Z
		features := [10]float64{x * x, y * y, z * z, x * y, x * z, y * z, x, y, z, 1}
		var pan float64
		for j, c := range trueCoeffs {
			pan += c * features[j]
		}
		samples[i] = TrackingSample{TargetPos: s.TargetPos, Pan: pan, Tilt: 0}
	}

	getPan := func(s TrackingSample) float64 { return s.Pan }
	coeffs := fitPolynomialSingleDirectQR(samples, getPan)

	err := meanAbsError(samples, coeffs, getPan)
	if err > 1e-8 {
		t.Errorf("exact-recovery residual %.2e exceeds tolerance 1e-8; coefficients may be wrong", err)
	}
	t.Logf("exact-recovery residual: %.2e", err)
}

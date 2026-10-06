package decad

import (
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file publishes the exact points of a planar manifold
// (docs/multibody-dynamics-design.md §9.4): each exact witness is converted
// once to a float with an outward ball, exactly as the shipped clips do; each
// exact normal direction is normalized with an enclosing ball; entries are
// ordered by A's feature, then B's, then exact coordinate (§9.5).

type planarEntry struct {
	point      ContactPoint
	keyA, keyB [2]int
	onA, onB   pair.Point3
}

// planarPatchManifold converts the kernel's exact points to a public
// manifold, or reports why it is withheld: a feature without one source
// identity (AmbiguousFeature), a witness ball over PointResolution
// (PointTooCoarse), or a normal angle over NormalResolution (NoNormalProof).
func planarPatchManifold(req ContactRequest, points []pair.PatchPoint,
	features *planarFeatureMap) (*ContactManifold, ContactReason) {
	entries := make([]planarEntry, 0, len(points))
	for _, p := range points {
		featureA, okA := features.feature(0, p.A)
		featureB, okB := features.feature(1, p.B)
		if !okA || !okB {
			return nil, ContactAmbiguousFeature
		}
		onA, validA := orientedBoxPoint([3]*big.Rat(p.OnA))
		onB, validB := orientedBoxPoint([3]*big.Rat(p.OnB))
		if !validA || !validB || onA.Bound.Base() > req.PointResolution.Base() ||
			onB.Bound.Base() > req.PointResolution.Base() {
			return nil, ContactPointTooCoarse
		}
		normal, angle, ok := planarNormal(p.Normal)
		if !ok || angle.Base() > req.NormalResolution.Base() {
			return nil, ContactNoNormalProof
		}
		separation := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: Exact}
		if p.Separation != (pair.ScalarReading{}) {
			separation = sourceBoxScalar(p.Separation)
		}
		point := ContactPoint{OnA: onA, OnB: onB, Normal: normal, NormalAngle: angle,
			Separation: separation, FaceA: featureA.Face, FaceB: featureB.Face,
			FeatureA: featureA, FeatureB: featureB}
		entries = append(entries, planarEntry{point: point, keyA: features.order(0, featureA),
			keyB: features.order(1, featureB), onA: p.OnA, onB: p.OnB})
	}
	sort.SliceStable(entries, func(i, j int) bool { return planarEntryLess(&entries[i], &entries[j]) })
	out := make([]ContactPoint, 0, len(entries))
	for i := range entries {
		if i > 0 && !planarEntryLess(&entries[i-1], &entries[i]) {
			// Equal keys and equal exact coordinates: the same entry reached
			// through two pieces, merged under certified equality.
			continue
		}
		out = append(out, entries[i].point)
	}
	return &ContactManifold{Points: out}, ContactNoReason
}

func planarEntryLess(x, y *planarEntry) bool {
	for _, c := range []int{compareKey(x.keyA, y.keyA), compareKey(x.keyB, y.keyB),
		comparePoint3(x.onA, y.onA), comparePoint3(x.onB, y.onB)} {
		if c != 0 {
			return c < 0
		}
	}
	return false
}

func compareKey(x, y [2]int) int {
	if x[0] != y[0] {
		return x[0] - y[0]
	}
	return x[1] - y[1]
}

func comparePoint3(x, y pair.Point3) int {
	for axis := range 3 {
		if c := x[axis].Cmp(y[axis]); c != 0 {
			return c
		}
	}
	return 0
}

// planarNormal normalizes an exact direction. It first scales the direction
// by its largest absolute component, an exact rational that is the same for
// every positive multiple of the direction and exactly negated for its
// opposite, so a pair queried in either order publishes exactly opposite
// normals. The ball encloses the true unit vector through exact square-root
// brackets of the scaled direction's squared length.
func planarNormal(dir proofarith.DyV3) (VecMeasurement, units.Value, bool) {
	largest := proofarith.DyAbs(dir[0])
	for _, c := range dir[1:] {
		if proofarith.DyCmp(proofarith.DyAbs(c), largest) > 0 {
			largest = proofarith.DyAbs(c)
		}
	}
	if largest.Sign() == 0 {
		return VecMeasurement{}, units.Value{}, false
	}
	var scaled [3]*big.Rat
	squared := new(big.Rat)
	for k := range 3 {
		scaled[k] = new(big.Rat).Quo(dir[k].Rat(), largest.Rat())
		squared.Add(squared, new(big.Rat).Mul(scaled[k], scaled[k]))
	}
	low, high := proofbound.RatSqrtDown(squared), proofbound.RatSqrtUp(squared)
	if low <= 0 || !finiteMeasurementValues(low, high) {
		return VecMeasurement{}, units.Value{}, false
	}
	raw := r3.Vec{X: ratFloatNearest(scaled[0]), Y: ratFloatNearest(scaled[1]), Z: ratFloatNearest(scaled[2])}
	value, ok := raw.Normalize()
	if !ok || !proofbound.FiniteVec(value) {
		return VecMeasurement{}, units.Value{}, false
	}
	components := [3]float64{value.X, value.Y, value.Z}
	maxError := new(big.Rat)
	for k := range 3 {
		for _, length := range []float64{low, high} {
			deviation := new(big.Rat).Quo(scaled[k], proofarith.FloatRat(length))
			deviation.Sub(proofarith.FloatRat(components[k]), deviation)
			deviation.Abs(deviation)
			if deviation.Cmp(maxError) > 0 {
				maxError = deviation
			}
		}
	}
	bound := proofbound.Radius3D(proofbound.RatFloatUp(maxError))
	angle := proofbound.UpRound(4 * bound)
	if !finiteMeasurementValues(bound, angle) || angle >= math.Pi {
		return VecMeasurement{}, units.Value{}, false
	}
	return VecMeasurement{Value: value, Bound: units.Scalar(bound), Exactness: exactnessOf(bound)},
		units.Radians(angle), true
}

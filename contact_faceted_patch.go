package decad

import (
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/measurement"
	pairbox "github.com/lestrrat-3d/decad/internal/pair/box"
	"github.com/lestrrat-3d/decad/internal/reportvocab"

	"github.com/lestrrat-3d/decad/internal/pair/planar"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/units"
)

// This file publishes the exact points of a planar manifold
// (docs/multibody-dynamics-design.md §9.4): each exact witness is converted
// once to a float with an outward ball, exactly as the shipped clips do; each
// exact normal direction is normalized with an enclosing ball; entries are
// ordered by A's feature, then B's, then exact coordinate (§9.5).

type planarEntry struct {
	point      reportvocab.ContactPoint
	keyA, keyB [2]int
	onA, onB   planar.Point3
}

// planarPatchManifold converts the kernel's exact points to a contact
// manifold, or reports why it is withheld: a feature without one source
// identity (AmbiguousFeature), a witness ball over PointResolution
// (PointTooCoarse), or a normal angle over NormalResolution (NoNormalProof).
func planarPatchManifold(req reportvocab.ContactRequest, points []planar.PatchPoint,
	features *reportvocab.PlanarFeatureMap) (*reportvocab.ContactManifold, reportvocab.ContactReason) {
	entries := make([]planarEntry, 0, len(points))
	for _, p := range points {
		featureA, okA := features.Feature(0, p.A)
		featureB, okB := features.Feature(1, p.B)
		if !okA || !okB {
			return nil, reportvocab.ContactAmbiguousFeature
		}
		onA, validA := orientedBoxPoint([3]*big.Rat(p.OnA))
		onB, validB := orientedBoxPoint([3]*big.Rat(p.OnB))
		if !validA || !validB || onA.Bound.Base() > req.PointResolution.Base() ||
			onB.Bound.Base() > req.PointResolution.Base() {
			return nil, reportvocab.ContactPointTooCoarse
		}
		normal, angle, ok := planarNormal(p.Normal)
		if !ok || angle.Base() > req.NormalResolution.Base() {
			return nil, reportvocab.ContactNoNormalProof
		}
		separation := measurement.Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(0), Exactness: measurement.Exact}
		if p.Separation != (pair.ScalarReading{}) {
			separation = sourceBoxScalar(p.Separation)
		}
		point := reportvocab.ContactPoint{OnA: vecMeasurementToInternal(onA), OnB: vecMeasurementToInternal(onB),
			Normal: vecMeasurementToInternal(normal), NormalAngle: angle,
			Separation: separation, FaceA: featureA.Face, FaceB: featureB.Face,
			FeatureA: featureA, FeatureB: featureB}
		entries = append(entries, planarEntry{point: point, keyA: features.Order(0, featureA),
			keyB: features.Order(1, featureB), onA: p.OnA, onB: p.OnB})
	}
	sort.SliceStable(entries, func(i, j int) bool { return planarEntryLess(&entries[i], &entries[j]) })
	out := make([]reportvocab.ContactPoint, 0, len(entries))
	for i := range entries {
		if i > 0 && !planarEntryLess(&entries[i-1], &entries[i]) {
			// Equal keys and equal exact coordinates: the same entry reached
			// through two pieces, merged under certified equality.
			continue
		}
		out = append(out, entries[i].point)
	}
	return &reportvocab.ContactManifold{Points: out}, reportvocab.ContactNoReason
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

func comparePoint3(x, y planar.Point3) int {
	for axis := range 3 {
		if c := x[axis].Cmp(y[axis]); c != 0 {
			return c
		}
	}
	return 0
}

// planarNormal publishes the existing bounded normal reading.
func planarNormal(dir proofarith.DyV3) (VecMeasurement, units.Value, bool) {
	reading, ok := pairbox.PlanarNormalReading(dir)
	if !ok {
		return VecMeasurement{}, units.Value{}, false
	}
	return VecMeasurement{Value: reading.Value, Bound: units.Scalar(reading.Bound),
		Exactness: exactnessOf(reading.Bound)}, units.Radians(reading.Angle), true
}

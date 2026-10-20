package pointquery

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// RatIntervalMeasurement publishes an exact length interval [lo, hi] as its
// nearest midpoint float and an outward bound covering both ends.
func RatIntervalMeasurement(lo, hi *big.Rat) (measurement.Measurement, bool) {
	mid := new(big.Rat).Quo(new(big.Rat).Add(lo, hi), big.NewRat(2, 1))
	value := sweeppath.RatFloatNearest(mid)
	if !measurement.FiniteValues(value) {
		return measurement.Measurement{}, false
	}
	held := proofarith.FloatRat(value)
	spread := proofbound.RatMax(new(big.Rat).Abs(new(big.Rat).Sub(lo, held)), new(big.Rat).Abs(new(big.Rat).Sub(hi, held)))
	bound := proofbound.RatFloatUp(spread)
	if !measurement.FiniteValues(value, bound) {
		return measurement.Measurement{}, false
	}
	return measurement.Measurement{Value: units.Millimeters(value), Bound: units.Millimeters(bound),
		Exactness: measurement.ExactnessOf(bound)}, true
}

// BodyDistance bounds the distance from p to the solid a VerifyAll mesh stands
// for. An interior or boundary point has distance zero; when membership cannot
// be proved the lower bound is zero.
func BodyDistance(ctx context.Context, p r3.Vec, mesh *topo.Mesh) (measurement.Measurement, error) {
	if !mesh.BoundaryOK || proofbound.IsNonFinite(mesh.Bound) || mesh.Bound < 0 {
		return measurement.Measurement{}, fmt.Errorf(`%w: point distance needs a verified finite mesh boundary bound`, decaderr.ErrUnsupported)
	}
	distanceSquared, err := MeshDistanceSquared(ctx, p, mesh.Vertices, mesh.Triangles)
	if err != nil {
		return measurement.Measurement{}, err
	}
	location := Undecided
	if mesh.SymDiffOK {
		location, err = LocateWithDistance(ctx, p, mesh.Vertices, mesh.Triangles,
			mesh.Bound, mesh.VolSymDiff, distanceSquared)
		if err != nil {
			return measurement.Measurement{}, err
		}
	}
	if location == Inside || location == OnBoundary {
		return measurement.Measurement{Value: units.Millimeters(0), Exactness: measurement.Exact, Bound: units.Millimeters(0)}, nil
	}
	distanceLo := proofbound.RatSqrtDown(distanceSquared)
	distanceHi := proofbound.RatSqrtUp(distanceSquared)
	if proofbound.IsNonFinite(distanceLo) || proofbound.IsNonFinite(distanceHi) {
		return measurement.Measurement{}, fmt.Errorf(`%w: point distance exceeds the representable range`, decaderr.ErrUnsupported)
	}
	bound := proofarith.FloatRat(mesh.Bound)
	lo := new(big.Rat)
	if location == Outside {
		lo.Sub(proofarith.FloatRat(distanceLo), bound)
		if lo.Sign() < 0 {
			lo.SetInt64(0)
		}
	}
	hi := new(big.Rat).Add(proofarith.FloatRat(distanceHi), bound)
	result, ok := RatIntervalMeasurement(lo, hi)
	if !ok {
		return measurement.Measurement{}, fmt.Errorf(`%w: point distance has no finite measurement`, decaderr.ErrUnsupported)
	}
	return result, nil
}

// FaceDistance bounds the distance from p to the trimmed patch of face, read
// off the triangles a VerifyBoundary mesh attributes to it and widened by the
// face's verified displacement bound.
func FaceDistance(ctx context.Context, p r3.Vec, mesh *topo.Mesh, face *topo.Face) (measurement.Measurement, error) {
	if !mesh.BoundaryOK {
		return measurement.Measurement{}, fmt.Errorf(`%w: face distance needs a verified mesh boundary`, decaderr.ErrUnsupported)
	}
	triangles := make([][3]int, 0)
	if len(mesh.Source) != len(mesh.Triangles) {
		return measurement.Measurement{}, fmt.Errorf(`%w: mesh source faces do not match its triangles`, decaderr.ErrBooleanFailed)
	}
	for i, source := range mesh.Source {
		if source == face {
			triangles = append(triangles, mesh.Triangles[i])
		}
	}
	if len(triangles) == 0 {
		return measurement.Measurement{}, fmt.Errorf(`%w: the face has no mesh triangles`, decaderr.ErrUnsupported)
	}
	bound, ok := mesh.FaceBound[face]
	if !ok {
		return measurement.Measurement{}, fmt.Errorf(`%w: the face has no displacement bound`, decaderr.ErrBooleanFailed)
	}
	if proofbound.IsNonFinite(bound) || bound < 0 {
		return measurement.Measurement{}, fmt.Errorf(`%w: face distance needs a finite displacement bound`, decaderr.ErrUnsupported)
	}
	distanceSquared, err := MeshDistanceSquared(ctx, p, mesh.Vertices, triangles)
	if err != nil {
		return measurement.Measurement{}, err
	}
	distanceLo := proofbound.RatSqrtDown(distanceSquared)
	distanceHi := proofbound.RatSqrtUp(distanceSquared)
	if proofbound.IsNonFinite(distanceLo) || proofbound.IsNonFinite(distanceHi) {
		return measurement.Measurement{}, fmt.Errorf(`%w: face distance exceeds the representable range`, decaderr.ErrUnsupported)
	}
	allow := proofarith.FloatRat(bound)
	lo := new(big.Rat).Sub(proofarith.FloatRat(distanceLo), allow)
	if lo.Sign() < 0 {
		lo.SetInt64(0)
	}
	hi := new(big.Rat).Add(proofarith.FloatRat(distanceHi), allow)
	result, ok := RatIntervalMeasurement(lo, hi)
	if !ok {
		return measurement.Measurement{}, fmt.Errorf(`%w: face distance has no finite measurement`, decaderr.ErrUnsupported)
	}
	return result, nil
}

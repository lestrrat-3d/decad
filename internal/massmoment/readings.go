package massmoment

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// MassReadings carries the integrator's computed scalar and tensor entries.
// The root package assembles the public MassProperties value.
type MassReadings struct {
	Mass   measurement.Measurement
	Center measurement.VecMeasurement
	Tensor [6]measurement.Measurement // XX, YY, ZZ, XY, XZ, YZ
}

func readingExactness(bound float64) measurement.Exactness {
	if bound == 0 {
		return measurement.Exact
	}
	return measurement.Approximate
}

// IntervalReading rounds a certified interval to a public scalar reading.
func IntervalReading(iv proofbound.RatInterval, unit units.Unit) (measurement.Measurement, error) {
	if iv.Lo.Cmp(iv.Hi) == 0 {
		return Reading(iv.Lo, unit)
	}
	held, _ := new(big.Rat).Quo(new(big.Rat).Add(iv.Lo, iv.Hi), big.NewRat(2, 1)).Float64()
	bound := proofbound.IntervalFloatError(iv, held)
	if proofbound.IsNonFinite(held) || proofbound.IsNonFinite(bound) {
		return measurement.Measurement{}, fmt.Errorf("%w: mass property cannot be represented finitely", decaderr.ErrNotFinite)
	}
	return measurement.Measurement{Value: units.New(held, unit), Bound: units.New(bound, unit),
		Exactness: readingExactness(bound)}, nil
}

// Reading rounds one exact rational to a public scalar reading.
func Reading(exact *big.Rat, unit units.Unit) (measurement.Measurement, error) {
	value, _ := exact.Float64()
	bound := proofarith.RationalFloatError(exact, value)
	if proofbound.IsNonFinite(value) || proofbound.IsNonFinite(bound) {
		return measurement.Measurement{}, fmt.Errorf("%w: mass property cannot be represented finitely", decaderr.ErrNotFinite)
	}
	return measurement.Measurement{Value: units.New(value, unit), Bound: units.New(bound, unit),
		Exactness: readingExactness(bound)}, nil
}

// Publish rounds a mass interval and world inertia intervals to readings and
// proves that every tensor within the published bounds is positive definite.
func Publish(ctx context.Context, center measurement.VecMeasurement,
	massIv proofbound.RatInterval, world [3][3]proofbound.RatInterval) (MassReadings, error) {
	if err := ctx.Err(); err != nil {
		return MassReadings{}, err
	}
	var err error
	result := MassReadings{Center: center}
	result.Mass, err = IntervalReading(massIv, units.Kilogram)
	if err != nil {
		return MassReadings{}, err
	}
	if result.Mass.Bound.Base() >= result.Mass.Value.Base() {
		return MassReadings{}, fmt.Errorf("%w: mass reading is not positive", decaderr.ErrUnsupported)
	}
	entries := []struct {
		iv      proofbound.RatInterval
		reading *measurement.Measurement
	}{
		{world[0][0], &result.Tensor[0]}, {world[1][1], &result.Tensor[1]},
		{world[2][2], &result.Tensor[2]}, {world[0][1], &result.Tensor[3]},
		{world[0][2], &result.Tensor[4]}, {world[1][2], &result.Tensor[5]},
	}
	for _, entry := range entries {
		*entry.reading, err = IntervalReading(entry.iv, units.KilogramSquareMillimeter)
		if err != nil {
			return MassReadings{}, err
		}
	}
	if !PositiveDefinite(publishedTensor(result.Tensor)) {
		return MassReadings{}, fmt.Errorf("%w: inertia interval does not prove positive definiteness", decaderr.ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassReadings{}, err
	}
	return result, nil
}

// publishedTensor encloses the six computed inertia readings.
func publishedTensor(reading [6]measurement.Measurement) [3][3]proofbound.RatInterval {
	entry := func(m measurement.Measurement) proofbound.RatInterval {
		return proofbound.IntervalWiden(proofbound.PointInterval(proofarith.FloatRat(m.Value.Base())),
			proofarith.FloatRat(m.Bound.Base()))
	}
	xy, xz, yz := entry(reading[3]), entry(reading[4]), entry(reading[5])
	return [3][3]proofbound.RatInterval{
		{entry(reading[0]), xy, xz},
		{xy, entry(reading[1]), yz},
		{xz, yz, entry(reading[2])},
	}
}

// HeldMeshMassProperties integrates an audited outward triangle set and
// widens its moments by the certified occupied-volume difference.
func HeldMeshMassProperties(ctx context.Context, bounds measurement.Box, anchor r3.Vec,
	verts []r3.Vec, tris [][3]int, volSymDiff float64, density units.Value) (MassReadings, error) {
	intervals, err := HeldMeshIntervals(ctx, MeshBounds{
		Min: bounds.Min, Max: bounds.Max, Bound: bounds.Bound,
	}, anchor, verts, tris, volSymDiff)
	if err != nil {
		return MassReadings{}, err
	}
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	result := MassReadings{}
	result.Mass, err = IntervalReading(proofbound.IntervalScale(intervals.Volume, rho), units.Kilogram)
	if err != nil {
		return MassReadings{}, err
	}
	var centerValue [3]float64
	centerBound := 0.0
	for i, enclosure := range intervals.Center {
		centerValue[i], _ = new(big.Rat).Quo(new(big.Rat).Add(enclosure.Lo, enclosure.Hi), big.NewRat(2, 1)).Float64()
		if proofbound.IsNonFinite(centerValue[i]) {
			return MassReadings{}, fmt.Errorf("%w: mesh mass center is nonfinite", decaderr.ErrNotFinite)
		}
		centerBound = math.Max(centerBound, proofbound.IntervalFloatError(enclosure, centerValue[i]))
	}
	centerBound = proofbound.Radius3D(centerBound)
	if proofbound.IsNonFinite(centerBound) {
		return MassReadings{}, fmt.Errorf("%w: mesh mass center bound is nonfinite", decaderr.ErrNotFinite)
	}
	result.Center = measurement.VecMeasurement{
		Value: r3.Vec{X: centerValue[0], Y: centerValue[1], Z: centerValue[2]},
		Bound: units.Millimeters(centerBound), Exactness: readingExactness(centerBound),
	}
	components := [6]*measurement.Measurement{&result.Tensor[0], &result.Tensor[1], &result.Tensor[2],
		&result.Tensor[3], &result.Tensor[4], &result.Tensor[5]}
	indices := [6][2]int{{0, 0}, {1, 1}, {2, 2}, {0, 1}, {0, 2}, {1, 2}}
	for k, pair := range indices {
		i, j := pair[0], pair[1]
		term := proofbound.IntervalNeg(intervals.Central[i][j])
		if i == j {
			term = proofbound.IntervalSub(intervals.Trace, intervals.Central[i][j])
		}
		*components[k], err = IntervalReading(proofbound.IntervalScale(term, rho), units.KilogramSquareMillimeter)
		if err != nil {
			return MassReadings{}, err
		}
	}
	read := func(m measurement.Measurement) proofbound.BoundedScalar {
		return proofbound.BoundedScalar{Value: m.Value.Base(), Bound: m.Bound.Base()}
	}
	if !MeshReadingsPositive(read(result.Mass),
		[3]proofbound.BoundedScalar{read(result.Tensor[0]), read(result.Tensor[1]), read(result.Tensor[2])},
		[3]proofbound.BoundedScalar{read(result.Tensor[3]), read(result.Tensor[4]), read(result.Tensor[5])}) {
		return MassReadings{}, ErrMeshIntervalUnproved
	}
	if err := ctx.Err(); err != nil {
		return MassReadings{}, err
	}
	return result, nil
}

// AuditMesh reruns shell closure, orientation, and vertex-link checks before
// integration. It also checks facet crossings unless the producer did so.
func AuditMesh(ctx context.Context, verts []r3.Vec, tris [][3]int, contactAudited bool) error {
	if _, err := facetproof.AuditFacetedMesh(ctx, verts, tris); err != nil {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		return fmt.Errorf("%w: mesh mass shell audit failed: %v", decaderr.ErrUnsupported, err)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		return fmt.Errorf("%w: mesh mass vertex-link audit failed: %v", decaderr.ErrUnsupported, err)
	}
	if contactAudited {
		return nil
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, tris); err != nil {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		return fmt.Errorf("%w: mesh mass crossing audit failed: %v", decaderr.ErrUnsupported, err)
	}
	return nil
}

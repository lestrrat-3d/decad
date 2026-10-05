package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// facetedMassProperties integrates a Boolean result only when its held solid
// has zero occupied-volume error. Equality almost everywhere then transfers
// all polynomial volume moments from the audited held mesh to the true body.
func facetedMassProperties(ctx context.Context, pp facetedPayload, density units.Value) (MassProperties, error) {
	if pp.meshBound != 0 || pp.volSymDiff != 0 || len(pp.verts) == 0 || len(pp.tris) == 0 {
		return MassProperties{}, fmt.Errorf("%w: faceted mass needs a zero-error occupied-volume proof", ErrUnsupported)
	}
	if _, err := auditFacetedMesh(ctx, pp.verts, pp.tris); err != nil {
		if ctx.Err() != nil {
			return MassProperties{}, ctx.Err()
		}
		return MassProperties{}, fmt.Errorf("%w: faceted mass shell audit failed: %v", ErrUnsupported, err)
	}
	if err := requireVertexLinks(ctx, &Mesh{vertices: pp.verts, triangles: pp.tris}); err != nil {
		if ctx.Err() != nil {
			return MassProperties{}, ctx.Err()
		}
		return MassProperties{}, fmt.Errorf("%w: faceted mass vertex-link audit failed: %v", ErrUnsupported, err)
	}
	if err := loftCrossingAudit(newWorkBudget(ctx), pp.verts, pp.tris); err != nil {
		if ctx.Err() != nil {
			return MassProperties{}, ctx.Err()
		}
		return MassProperties{}, fmt.Errorf("%w: faceted mass crossing audit failed: %v", ErrUnsupported, err)
	}

	// Anchor at a held corner before summing tetrahedra. The binary64 vertex
	// coordinates are exact rational inputs; only the final readings round.
	anchor := pp.verts[0]
	anchorExact := xptOf(anchor)
	vertices := make([][3]*big.Rat, len(pp.verts))
	lifted := make([]xpt, len(pp.verts))
	budget := newWorkBudget(ctx)
	for i, v := range pp.verts {
		if err := budget.step(); err != nil {
			return MassProperties{}, err
		}
		if !finiteVec(v) {
			return MassProperties{}, fmt.Errorf("%w: faceted mass vertex is nonfinite", ErrUnsupported)
		}
		lifted[i] = xsub(xptOf(v), anchorExact)
		x, y, z := xhpRat(xhp(lifted[i]))
		vertices[i] = [3]*big.Rat{x, y, z}
	}
	volume6 := new(big.Rat)
	var first [3]*big.Rat
	var second [3][3]*big.Rat
	for i := range 3 {
		first[i] = new(big.Rat)
		for j := range 3 {
			second[i][j] = new(big.Rat)
		}
	}
	for _, tri := range pp.tris {
		if err := budget.step(); err != nil {
			return MassProperties{}, err
		}
		a, b, c := vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
		det := xdotRat(lifted[tri[0]], xcross(lifted[tri[1]], lifted[tri[2]]))
		volume6.Add(volume6, det)
		var sum [3]*big.Rat
		for i := range 3 {
			sum[i] = new(big.Rat).Add(new(big.Rat).Add(a[i], b[i]), c[i])
			first[i].Add(first[i], new(big.Rat).Mul(det, sum[i]))
		}
		for i := range 3 {
			for j := i; j < 3; j++ {
				paired := new(big.Rat).Mul(sum[i], sum[j])
				for _, v := range [][3]*big.Rat{a, b, c} {
					paired.Add(paired, new(big.Rat).Mul(v[i], v[j]))
				}
				second[i][j].Add(second[i][j], new(big.Rat).Mul(det, paired))
			}
		}
	}
	if volume6.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: faceted mass volume is not positive", ErrUnsupported)
	}
	volume := new(big.Rat).Quo(volume6, big.NewRat(6, 1))
	for i := range 3 {
		first[i].Quo(first[i], big.NewRat(24, 1))
		for j := i; j < 3; j++ {
			second[i][j].Quo(second[i][j], big.NewRat(120, 1))
		}
	}

	var center [3]*big.Rat
	var central [3][3]*big.Rat
	for i, origin := range []*big.Rat{floatRat(anchor.X), floatRat(anchor.Y), floatRat(anchor.Z)} {
		center[i] = new(big.Rat).Add(origin, new(big.Rat).Quo(first[i], volume))
		for j := i; j < 3; j++ {
			shift := new(big.Rat).Quo(new(big.Rat).Mul(first[i], first[j]), volume)
			central[i][j] = new(big.Rat).Sub(second[i][j], shift)
		}
	}
	trace := new(big.Rat).Add(central[0][0], central[1][1])
	trace.Add(trace, central[2][2])
	rho := new(big.Rat).Mul(floatRat(density.Mag()), floatRat(density.Unit().Factor()))
	result := MassProperties{}
	var err error
	result.Mass, err = massReading(new(big.Rat).Mul(rho, volume), units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	var centerValue [3]float64
	centerBound := 0.0
	for i, exact := range center {
		centerValue[i], _ = exact.Float64()
		if isNonFinite(centerValue[i]) {
			return MassProperties{}, fmt.Errorf("%w: faceted mass center is nonfinite", ErrNotFinite)
		}
		centerBound = math.Max(centerBound, rationalFloatError(exact, centerValue[i]))
	}
	centerBound = radius3D(centerBound)
	if isNonFinite(centerBound) {
		return MassProperties{}, fmt.Errorf("%w: faceted mass center bound is nonfinite", ErrNotFinite)
	}
	result.Center = VecMeasurement{
		Value: r3.Vec{X: centerValue[0], Y: centerValue[1], Z: centerValue[2]},
		Bound: units.Millimeters(centerBound), Exactness: exactnessOf(centerBound),
	}
	components := [6]*Measurement{&result.Inertia.XX, &result.Inertia.YY, &result.Inertia.ZZ,
		&result.Inertia.XY, &result.Inertia.XZ, &result.Inertia.YZ}
	indices := [6][2]int{{0, 0}, {1, 1}, {2, 2}, {0, 1}, {0, 2}, {1, 2}}
	for k, pair := range indices {
		i, j := pair[0], pair[1]
		term := new(big.Rat).Neg(central[i][j])
		if i == j {
			term.Add(term, trace)
		}
		*components[k], err = massReading(new(big.Rat).Mul(rho, term), units.KilogramSquareMillimeter)
		if err != nil {
			return MassProperties{}, err
		}
	}
	if !facetedMassReadingsPositive(result) {
		return MassProperties{}, fmt.Errorf("%w: faceted mass or inertia interval is not provably positive", ErrUnsupported)
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}

// A positive row-dominance margin certifies every tensor in the six rounded
// component intervals, including the mixed entries, as positive definite.
func facetedMassReadingsPositive(m MassProperties) bool {
	if m.Mass.Value.Base() <= m.Mass.Bound.Base() {
		return false
	}
	diagonal := [3]Measurement{m.Inertia.XX, m.Inertia.YY, m.Inertia.ZZ}
	off := [3]Measurement{m.Inertia.XY, m.Inertia.XZ, m.Inertia.YZ}
	var offUpper [3]*big.Rat
	for i, v := range off {
		magnitude := floatRat(v.Value.Base())
		magnitude.Abs(magnitude)
		offUpper[i] = new(big.Rat).Add(magnitude, floatRat(v.Bound.Base()))
	}
	for i, v := range diagonal {
		lower := new(big.Rat).Sub(floatRat(v.Value.Base()), floatRat(v.Bound.Base()))
		switch i {
		case 0:
			lower.Sub(lower, offUpper[0]).Sub(lower, offUpper[1])
		case 1:
			lower.Sub(lower, offUpper[0]).Sub(lower, offUpper[2])
		case 2:
			lower.Sub(lower, offUpper[1]).Sub(lower, offUpper[2])
		}
		if lower.Sign() <= 0 {
			return false
		}
	}
	return true
}

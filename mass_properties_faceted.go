package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// facetedMassProperties integrates an audited Boolean mesh and widens its
// volume moments by the payload's certified occupied-volume difference.
func facetedMassProperties(ctx context.Context, b *Body, pp facetedPayload, density units.Value) (MassProperties, error) {
	if len(pp.verts) == 0 || len(pp.tris) == 0 || isNonFinite(pp.meshBound) || pp.meshBound < 0 ||
		isNonFinite(pp.volSymDiff) || pp.volSymDiff < 0 {
		return MassProperties{}, fmt.Errorf("%w: faceted mass has no finite occupied-volume certificate", ErrUnsupported)
	}
	mesh, err := b.Tessellate(ctx, units.Millimeters(math.Max(1, pp.meshBound)), WithVerification(VerifyAll))
	if err != nil {
		return MassProperties{}, err
	}
	if !mesh.BoundaryVerified() || !mesh.VolumeVerified() ||
		mesh.volSymDiff != pp.volSymDiff || mesh.bound != pp.meshBound {
		return MassProperties{}, fmt.Errorf("%w: faceted mass has no matching verified mesh certificate", ErrUnsupported)
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
	maxMesh := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
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
		for axis, coord := range vertices[i] {
			magnitude := new(big.Rat).Abs(coord)
			if magnitude.Cmp(maxMesh[axis]) > 0 {
				maxMesh[axis] = magnitude
			}
		}
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
	radius, err := facetedMassRadius(b.bounds, anchor, maxMesh)
	if err != nil {
		return MassProperties{}, err
	}
	volumeError := floatRat(pp.volSymDiff)
	firstError := new(big.Rat).Mul(radius, volumeError)
	secondError := new(big.Rat).Mul(new(big.Rat).Mul(radius, radius), volumeError)
	volumeIV := facetedMomentInterval(volume, volumeError)
	if volumeIV.lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: faceted volume interval includes zero", ErrUnsupported)
	}
	var firstIV [3]ratInterval
	var secondIV [3][3]ratInterval
	for i := range 3 {
		firstIV[i] = facetedMomentInterval(first[i], firstError)
		for j := i; j < 3; j++ {
			secondIV[i][j] = facetedMomentInterval(second[i][j], secondError)
		}
	}

	var center [3]ratInterval
	var central [3][3]ratInterval
	for i, origin := range []*big.Rat{floatRat(anchor.X), floatRat(anchor.Y), floatRat(anchor.Z)} {
		offset, _ := intervalQuo(firstIV[i], volumeIV)
		center[i] = intervalAdd(pointInterval(origin), offset)
		for j := i; j < 3; j++ {
			shift, _ := intervalQuo(intervalMul(firstIV[i], firstIV[j]), volumeIV)
			central[i][j] = intervalSub(secondIV[i][j], shift)
		}
	}
	trace := intervalAdd(intervalAdd(central[0][0], central[1][1]), central[2][2])
	rho := new(big.Rat).Mul(floatRat(density.Mag()), floatRat(density.Unit().Factor()))
	result := MassProperties{}
	result.Mass, err = massIntervalReading(intervalScale(volumeIV, rho), units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	var centerValue [3]float64
	centerBound := 0.0
	for i, enclosure := range center {
		centerValue[i], _ = intervalMid(enclosure).Float64()
		if isNonFinite(centerValue[i]) {
			return MassProperties{}, fmt.Errorf("%w: faceted mass center is nonfinite", ErrNotFinite)
		}
		centerBound = math.Max(centerBound, intervalFloatError(enclosure, centerValue[i]))
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
		term := intervalNeg(central[i][j])
		if i == j {
			term = intervalSub(trace, central[i][j])
		}
		*components[k], err = massIntervalReading(intervalScale(term, rho), units.KilogramSquareMillimeter)
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

// facetedMassRadius bounds |x-O| by its L1 norm for both the held mesh and
// the denoted solid. Box.Bound widens each true coordinate from its held
// extreme; the exact rational sum introduces no new rounding allowance.
func facetedMassRadius(box Box, anchor r3.Vec, maxMesh [3]*big.Rat) (*big.Rat, error) {
	allow := box.Bound.Base()
	if box.Bound.Kind() != units.Length || isNonFinite(allow) || allow < 0 ||
		!finiteVec(box.Min) || !finiteVec(box.Max) {
		return nil, fmt.Errorf("%w: faceted mass has no finite spatial bound", ErrUnsupported)
	}
	radius := new(big.Rat)
	ends := [3][3]float64{
		{box.Min.X, box.Max.X, anchor.X},
		{box.Min.Y, box.Max.Y, anchor.Y},
		{box.Min.Z, box.Max.Z, anchor.Z},
	}
	for axis, end := range ends {
		lo := new(big.Rat).Abs(new(big.Rat).Sub(floatRat(end[0]), floatRat(end[2])))
		hi := new(big.Rat).Abs(new(big.Rat).Sub(floatRat(end[1]), floatRat(end[2])))
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		lo.Add(lo, floatRat(allow))
		if maxMesh[axis].Cmp(lo) > 0 {
			lo = maxMesh[axis]
		}
		radius.Add(radius, lo)
	}
	return radius, nil
}

func facetedMomentInterval(value, bound *big.Rat) ratInterval {
	return intervalOwned(new(big.Rat).Sub(value, bound), new(big.Rat).Add(value, bound))
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

package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file owns the mass properties read off an audited triangle set whose
// occupied volume differs from the denoted solid by a certified E
// (docs/dynamic-mass-design.md §2.2, docs/multibody-dynamics-design.md §8.3-§8.5):
//
//   - tetraMoments, the exact signed-tetrahedron sums every held triangle set
//     shares, faceted Booleans included;
//   - heldMeshMassProperties, which widens those sums by E, R_i·E and
//     R_i·R_j·E and forms the center and centroidal tensor;
//   - verifiedMeshMassProperties, the VerifyAll tolerance ladder. It is the
//     path every solid without an analytic arm takes: a loft or an exact
//     stitched solid restates its own held triangles at any tolerance, so it
//     settles at the first step, and a curved payload refines until its tensor
//     interval proves positive or its own tessellation refuses the next step.
//     A revolve mesh reaches k = 10 at most: its facet-contact audit refuses a
//     finer mesh at the fixed facet-pair ceiling.

// meshLadderFirst and meshLadderLast are §8.5's ladder: tol_k = diameter·2^-k
// for k = meshLadderFirst … meshLadderLast.
const (
	meshLadderFirst = 8
	meshLadderLast  = 14
)

// errMassIntervalUnproved marks a held-mesh reading whose volume or tensor
// interval does not prove positive. A finer mesh may prove it, so the ladder
// continues on it and on nothing else.
var errMassIntervalUnproved = fmt.Errorf("%w: mesh mass or inertia interval is not provably positive", ErrUnsupported)

// tetraMoments accumulates, over signed tetrahedra (O, a, b, c) with O the
// anchor, six times the volume, 24 times each first moment and 120 times each
// second moment: for one tetrahedron with det = a·(b×c) and s = a+b+c,
// 6V = det, 24P_i = det·s_i and 120Q_ij = det·(s_i s_j + Σ_v v_i v_j).
// Every coordinate is an exact rational, so the sums round nothing.
type tetraMoments struct {
	volume6 *big.Rat
	first   [3]*big.Rat
	second  [3][3]*big.Rat // upper triangle, i <= j
}

func newTetraMoments() *tetraMoments {
	m := &tetraMoments{volume6: new(big.Rat)}
	for i := range 3 {
		m.first[i] = new(big.Rat)
		for j := range 3 {
			m.second[i][j] = new(big.Rat)
		}
	}
	return m
}

// add folds one tetrahedron given its three anchored vertices and their exact
// determinant.
func (m *tetraMoments) add(a, b, c [3]*big.Rat, det *big.Rat) {
	m.volume6.Add(m.volume6, det)
	var sum [3]*big.Rat
	for i := range 3 {
		sum[i] = new(big.Rat).Add(new(big.Rat).Add(a[i], b[i]), c[i])
		m.first[i].Add(m.first[i], new(big.Rat).Mul(det, sum[i]))
	}
	for i := range 3 {
		for j := i; j < 3; j++ {
			paired := new(big.Rat).Mul(sum[i], sum[j])
			for _, v := range [][3]*big.Rat{a, b, c} {
				paired.Add(paired, new(big.Rat).Mul(v[i], v[j]))
			}
			m.second[i][j].Add(m.second[i][j], new(big.Rat).Mul(det, paired))
		}
	}
}

// moments returns V, P and the upper triangle of Q about the anchor.
func (m *tetraMoments) moments() (*big.Rat, [3]*big.Rat, [3][3]*big.Rat) {
	volume := new(big.Rat).Quo(m.volume6, big.NewRat(6, 1))
	var first [3]*big.Rat
	var second [3][3]*big.Rat
	for i := range 3 {
		first[i] = new(big.Rat).Quo(m.first[i], big.NewRat(24, 1))
		for j := i; j < 3; j++ {
			second[i][j] = new(big.Rat).Quo(m.second[i][j], big.NewRat(120, 1))
		}
	}
	return volume, first, second
}

// heldMeshMassProperties integrates an already audited, outward triangle set
// about anchor and widens the held V, each P_i and each Q_ij by E, R_i·E and
// R_i·R_j·E (docs/dynamic-mass-design.md §2.2). E = volSymDiff bounds the
// occupied volume between the triangle set and the denoted solid, and R_i
// bounds |x_i - anchor_i| over both. bounds is the body's own bounded box,
// which encloses the denoted solid. Interval division forms the center and
// the centroidal tensor; a volume or tensor interval that does not prove
// positive returns errMassIntervalUnproved.
func heldMeshMassProperties(ctx context.Context, bounds Box, anchor r3.Vec, verts []r3.Vec, tris [][3]int, volSymDiff float64, density units.Value) (MassProperties, error) {
	if len(verts) == 0 || len(tris) == 0 || !finiteVec(anchor) || !nonNegativeFinite(volSymDiff) {
		return MassProperties{}, fmt.Errorf("%w: mesh mass has no finite occupied-volume certificate", ErrUnsupported)
	}
	// The binary64 vertex coordinates are exact rational inputs; only the
	// final readings round.
	anchorExact := xptOf(anchor)
	vertices := make([][3]*big.Rat, len(verts))
	lifted := make([]xpt, len(verts))
	maxMesh := [3]*big.Rat{new(big.Rat), new(big.Rat), new(big.Rat)}
	budget := newWorkBudget(ctx)
	for i, v := range verts {
		if err := budget.step(); err != nil {
			return MassProperties{}, err
		}
		if !finiteVec(v) {
			return MassProperties{}, fmt.Errorf("%w: mesh mass vertex is nonfinite", ErrUnsupported)
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
	sums := newTetraMoments()
	for _, tri := range tris {
		if err := budget.step(); err != nil {
			return MassProperties{}, err
		}
		det := xdotRat(lifted[tri[0]], xcross(lifted[tri[1]], lifted[tri[2]]))
		sums.add(vertices[tri[0]], vertices[tri[1]], vertices[tri[2]], det)
	}
	if err := budget.err(); err != nil {
		return MassProperties{}, err
	}
	volume, first, second := sums.moments()
	if volume.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: mesh volume is not positive", errMassIntervalUnproved)
	}
	extent, err := meshMassExtent(bounds, anchor, maxMesh)
	if err != nil {
		return MassProperties{}, err
	}
	// Over the symmetric difference D, |∫_D q_i| <= R_i·E and
	// |∫_D q_i q_j| <= R_i·R_j·E, since |q_i| <= R_i on both regions.
	volumeError := proofarith.FloatRat(volSymDiff)
	volumeIV := intervalWiden(pointInterval(volume), volumeError)
	if volumeIV.lo.Sign() <= 0 {
		return MassProperties{}, fmt.Errorf("%w: mesh volume interval includes zero", errMassIntervalUnproved)
	}
	var firstIV [3]ratInterval
	var secondIV [3][3]ratInterval
	for i := range 3 {
		firstIV[i] = intervalWiden(pointInterval(first[i]), new(big.Rat).Mul(extent[i], volumeError))
		for j := i; j < 3; j++ {
			secondError := new(big.Rat).Mul(new(big.Rat).Mul(extent[i], extent[j]), volumeError)
			secondIV[i][j] = intervalWiden(pointInterval(second[i][j]), secondError)
		}
	}

	var center [3]ratInterval
	var central [3][3]ratInterval
	for i, origin := range []*big.Rat{proofarith.FloatRat(anchor.X), proofarith.FloatRat(anchor.Y), proofarith.FloatRat(anchor.Z)} {
		offset, _ := intervalQuo(firstIV[i], volumeIV)
		center[i] = intervalAdd(pointInterval(origin), offset)
		for j := i; j < 3; j++ {
			shift, _ := intervalQuo(intervalMul(firstIV[i], firstIV[j]), volumeIV)
			central[i][j] = intervalSub(secondIV[i][j], shift)
		}
	}
	trace := intervalAdd(intervalAdd(central[0][0], central[1][1]), central[2][2])
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
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
			return MassProperties{}, fmt.Errorf("%w: mesh mass center is nonfinite", ErrNotFinite)
		}
		centerBound = math.Max(centerBound, intervalFloatError(enclosure, centerValue[i]))
	}
	centerBound = radius3D(centerBound)
	if isNonFinite(centerBound) {
		return MassProperties{}, fmt.Errorf("%w: mesh mass center bound is nonfinite", ErrNotFinite)
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
	if !meshMassReadingsPositive(result) {
		return MassProperties{}, errMassIntervalUnproved
	}
	if err := ctx.Err(); err != nil {
		return MassProperties{}, err
	}
	return result, nil
}

// verifiedMeshMassProperties is docs/multibody-dynamics-design.md §8.5's
// ladder for a solid with no analytic arm: tessellate at VerifyAll with
// tol_k = diameter·2^-k, diameter the body box's diagonal, and return the
// first reading whose volume and tensor intervals prove positive. A
// tessellation refusal, a mesh that carries no occupied-volume proof, or a
// failed embedding audit ends the ladder at once: a finer mesh of the same
// payload is refused, or carries no proof, the same way.
func verifiedMeshMassProperties(ctx context.Context, b *Body, density units.Value) (MassProperties, error) {
	diameter := b.bounds.Max.Sub(b.bounds.Min).Len()
	if isNonFinite(diameter) || diameter <= 0 {
		return MassProperties{}, fmt.Errorf("%w: mesh mass has no finite positive diameter", ErrUnsupported)
	}
	for k := meshLadderFirst; k <= meshLadderLast; k++ {
		result, err := meshMassPropertiesAt(ctx, b, math.Ldexp(diameter, -k), density)
		if err == nil {
			return result, nil
		}
		if !errors.Is(err, errMassIntervalUnproved) {
			return MassProperties{}, err
		}
	}
	return MassProperties{}, fmt.Errorf("%w: tolerance ladder exhausted at diameter·2^-%d", errMassIntervalUnproved, meshLadderLast)
}

// meshMassPropertiesAt is one step of the ladder at chord tolerance tol
// (millimetres). The VerifyAll mesh's closure, orientation and vertex-link
// audits run again on its triangles here, as does the exact facet-crossing
// audit unless the tessellation itself ran its payload's facet-contact audit
// and reported the boundary verified.
func meshMassPropertiesAt(ctx context.Context, b *Body, tol float64, density units.Value) (MassProperties, error) {
	mesh, err := b.Tessellate(ctx, units.Millimeters(tol), WithVerification(VerifyAll))
	if err != nil {
		return MassProperties{}, err
	}
	if !mesh.BoundaryVerified() || !mesh.VolumeVerified() {
		return MassProperties{}, fmt.Errorf("%w: the body's mesh carries no occupied-volume proof", ErrUnsupported)
	}
	if err := auditMassMesh(ctx, mesh.vertices, mesh.triangles, payloadAuditsFacetContact(b.payload)); err != nil {
		return MassProperties{}, err
	}
	anchor := b.bounds.Min.Add(b.bounds.Max).Scale(.5)
	return heldMeshMassProperties(ctx, b.bounds, anchor, mesh.vertices, mesh.triangles, mesh.volSymDiff, density)
}

// auditMassMesh reruns the shell closure and orientation, vertex-link and,
// unless contactAudited, exact facet-crossing audits on a held triangle set
// before its tetrahedra are integrated (docs/dynamic-mass-design.md §2.2).
func auditMassMesh(ctx context.Context, verts []r3.Vec, tris [][3]int, contactAudited bool) error {
	if _, err := auditFacetedMesh(ctx, verts, tris); err != nil {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		return fmt.Errorf("%w: mesh mass shell audit failed: %v", ErrUnsupported, err)
	}
	if err := requireVertexLinks(ctx, &Mesh{vertices: verts, triangles: tris}); err != nil {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		return fmt.Errorf("%w: mesh mass vertex-link audit failed: %v", ErrUnsupported, err)
	}
	if contactAudited {
		return nil
	}
	if err := loftCrossingAudit(newWorkBudget(ctx), verts, tris); err != nil {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		return fmt.Errorf("%w: mesh mass crossing audit failed: %v", ErrUnsupported, err)
	}
	return nil
}

// meshMassExtent bounds |x_i - O_i| per axis for both the held mesh and the
// denoted solid. Box.Bound widens each true coordinate from its held extreme;
// the exact rational arithmetic introduces no new rounding allowance.
func meshMassExtent(box Box, anchor r3.Vec, maxMesh [3]*big.Rat) ([3]*big.Rat, error) {
	var extent [3]*big.Rat
	allow := box.Bound.Base()
	if box.Bound.Kind() != units.Length || isNonFinite(allow) || allow < 0 ||
		!finiteVec(box.Min) || !finiteVec(box.Max) {
		return extent, fmt.Errorf("%w: mesh mass has no finite spatial bound", ErrUnsupported)
	}
	ends := [3][3]float64{
		{box.Min.X, box.Max.X, anchor.X},
		{box.Min.Y, box.Max.Y, anchor.Y},
		{box.Min.Z, box.Max.Z, anchor.Z},
	}
	for axis, end := range ends {
		lo := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(end[0]), proofarith.FloatRat(end[2])))
		hi := new(big.Rat).Abs(new(big.Rat).Sub(proofarith.FloatRat(end[1]), proofarith.FloatRat(end[2])))
		if hi.Cmp(lo) > 0 {
			lo = hi
		}
		lo.Add(lo, proofarith.FloatRat(allow))
		if maxMesh[axis].Cmp(lo) > 0 {
			lo = maxMesh[axis]
		}
		extent[axis] = lo
	}
	return extent, nil
}

// A positive row-dominance margin certifies every tensor in the six rounded
// component intervals, including the mixed entries, as positive definite.
func meshMassReadingsPositive(m MassProperties) bool {
	if m.Mass.Value.Base() <= m.Mass.Bound.Base() {
		return false
	}
	diagonal := [3]Measurement{m.Inertia.XX, m.Inertia.YY, m.Inertia.ZZ}
	off := [3]Measurement{m.Inertia.XY, m.Inertia.XZ, m.Inertia.YZ}
	var offUpper [3]*big.Rat
	for i, v := range off {
		magnitude := proofarith.FloatRat(v.Value.Base())
		magnitude.Abs(magnitude)
		offUpper[i] = new(big.Rat).Add(magnitude, proofarith.FloatRat(v.Bound.Base()))
	}
	for i, v := range diagonal {
		lower := new(big.Rat).Sub(proofarith.FloatRat(v.Value.Base()), proofarith.FloatRat(v.Bound.Base()))
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

package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/massmoment"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file owns the mass properties read off an audited triangle set whose
// occupied volume differs from the denoted solid by a certified E
// (docs/dynamic-mass-design.md §2.2, docs/multibody-dynamics-design.md §8.3-§8.5):
//
//   - internal/massmoment integrates exact signed tetrahedra and widens their
//     moments by E, R_i·E and R_i·R_j·E;
//   - heldMeshMassProperties rounds those intervals into public readings;
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
var errMassIntervalUnproved = massmoment.ErrMeshIntervalUnproved

// heldMeshMassProperties integrates an already audited, outward triangle set
// about anchor and widens the held V, each P_i and each Q_ij by E, R_i·E and
// R_i·R_j·E (docs/dynamic-mass-design.md §2.2). E = volSymDiff bounds the
// occupied volume between the triangle set and the denoted solid, and R_i
// bounds |x_i - anchor_i| over both. bounds is the body's own bounded box,
// which encloses the denoted solid. Interval division forms the center and
// the centroidal tensor; a volume or tensor interval that does not prove
// positive returns errMassIntervalUnproved.
func heldMeshMassProperties(ctx context.Context, bounds Box, anchor r3.Vec, verts []r3.Vec, tris [][3]int, volSymDiff float64, density units.Value) (MassProperties, error) {
	intervals, err := massmoment.HeldMeshIntervals(ctx, massmoment.MeshBounds{
		Min: bounds.Min, Max: bounds.Max, Bound: bounds.Bound,
	}, anchor, verts, tris, volSymDiff)
	if err != nil {
		return MassProperties{}, err
	}
	rho := new(big.Rat).Mul(proofarith.FloatRat(density.Mag()), proofarith.FloatRat(density.Unit().Factor()))
	result := MassProperties{}
	result.Mass, err = massIntervalReading(proofbound.IntervalScale(intervals.Volume, rho), units.Kilogram)
	if err != nil {
		return MassProperties{}, err
	}
	var centerValue [3]float64
	centerBound := 0.0
	for i, enclosure := range intervals.Center {
		centerValue[i], _ = intervalMid(enclosure).Float64()
		if proofbound.IsNonFinite(centerValue[i]) {
			return MassProperties{}, fmt.Errorf("%w: mesh mass center is nonfinite", ErrNotFinite)
		}
		centerBound = math.Max(centerBound, proofbound.IntervalFloatError(enclosure, centerValue[i]))
	}
	centerBound = proofbound.Radius3D(centerBound)
	if proofbound.IsNonFinite(centerBound) {
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
		term := proofbound.IntervalNeg(intervals.Central[i][j])
		if i == j {
			term = proofbound.IntervalSub(intervals.Trace, intervals.Central[i][j])
		}
		*components[k], err = massIntervalReading(proofbound.IntervalScale(term, rho), units.KilogramSquareMillimeter)
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
	if proofbound.IsNonFinite(diameter) || diameter <= 0 {
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
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), verts, tris); err != nil {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		return fmt.Errorf("%w: mesh mass crossing audit failed: %v", ErrUnsupported, err)
	}
	return nil
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

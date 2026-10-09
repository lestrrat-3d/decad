package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/revolveproof"
)

// This file adapts docs/tessellation-design.md §13's increment T4
// (docs/tessellation-reach-design.md §6, R5): the OCCUPIED-VOLUME proof a
// revolve mesh must carry before any boolean may consume it. §§8-10 prove the
// mesh's facets and area; internal/revolveproof bounds the VOLUME difference
// between that mesh and the body it represents.
//
// §11 forbids the obvious shortcut outright: Mesh.Bound × held area is NOT that
// proof. A two-sided Hausdorff bound does not bound occupied volume, because a
// torus's inner and outer walls move in OPPOSITE material senses and a
// doubly-curved cell can gain material where another loses it — the signed
// error cancels while the symmetric difference does not. So §11 instead walks
// the four explicit stages between the analytic body and the returned
// polyhedron and charges each of them separately:
//
//	B0 → BM   the meridian arcs replaced by their chords     Mmeridian
//	BM → BH   the angular direction chorded                  Σ_cells Icell
//	BH → BC   coordinate-construction rounding               Mconstruct
//	BC → BR   final placement rounding                       Mround
//
// The set triangle inequality composes them, and every term is an ABSOLUTE
// swept volume, so nothing cancels between stages, between loops, or between a
// hole and the outline it sits in.
//
// Two structural facts make the middle term cheap:
//
//   - BM's meridian is a POLYLINE. Every circular generator has already been
//     replaced by its chords at this stage, so §11's straight-generator
//     homotopy H answers for EVERY cell of a sphere or a torus too; the
//     curvature it dropped is charged in full by Mmeridian, one stage earlier.
//   - The angular factor of Icell does not depend on the cell. Rotating a cell
//     about the axis is an isometry, so the same reading answers for every
//     angular interval, and separating the meridian direction out of the triple
//     integral (below) leaves a factor that depends on dφ ALONE. It is
//     therefore proven once per mesh rather than once per cell.

// revolveMeridianMoment reads the circular meridian displacement.
func revolveMeridianMoment(p *revolvePlan) float64 {
	return revolveproof.MeridianMoment(revolveWalkView(p.resolved), p.Meridian, p.Sweep, p.rp.full)
}

// revolveSweepUpper reads the outward bound on the swept angle.
func revolveSweepUpper(p *revolvePlan) float64 {
	return revolveproof.SweepUpper(p.Sweep, p.rp.full)
}

// revolveSymDiff reads the occupied-volume difference bound.
func revolveSymDiff(m *Mesh, p *revolvePlan, angular *big.Rat, deltaC, deltaR float64) (float64, error) {
	return revolveproof.SymDiff(m.vertices, m.triangles, revolveWalkView(p.resolved),
		p.Meridian, p.Sweep, p.rp.full, angular, deltaC, deltaR)
}

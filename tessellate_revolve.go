package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
)

// This file is docs/tessellation-design.md §13's increment T2
// (docs/tessellation-reach-design.md §6, R3): revolve tessellation for LINE
// generators. It assembles the mesh — the meridian samples, the one global
// angular sequence, the rings, the cylinder/cone/plane cells, the poles and
// apexes, the partial caps and the full-turn cycles — and
// tessellate_revolve_proof.go proves it.
//
// A section carrying a CIRCULAR meridian generator (a sphere or a torus wall)
// is refused here: those cells are §13's increment T3, and refusing is what
// keeps this increment's own proofs honest rather than stretching a
// straight-generator argument over a curved one. The occupied-volume proof
// (§11) is increment T4, so the mesh this file returns serves EXPORT and
// carries symDiffOK false — the mesh boolean refuses a revolve operand through
// operandSymDiff, which is the one gate that decides it.
//
// Two structural facts shape everything below, and both are
// docs/tessellation-design.md §8's:
//
//   - The angular sequence is GLOBAL. Every off-axis ring in every loop uses
//     the same angles, so adjacent generator faces share their whole latitude
//     edge, a full turn closes with no seam ring, and the partial caps sit
//     exactly on the wall vertices at φ0 and φ1.
//   - Samples are stored UNPLACED and placed once, so the two coordinate
//     stages — construction and placement — are measured apart from one
//     another (deltaC and deltaR) rather than folded into one figure.

// maxFacetsPerMesh, maxFacetWorkPerCall are docs/tessellation-design.md §3's
// two per-call facet ceilings, beside budget.go's maxFacetPairTestsPerCall.
// Every one of them is checked with unsigned integer arithmetic BEFORE the
// allocation or audit it governs, so an over-budget request refuses rather
// than building the thing that would have blown the budget.
const (
	maxFacetsPerMesh    = 65_536
	maxFacetWorkPerCall = 262_144
)

// revMeridian is one meridian sample: a junction between two consecutive walks
// of one recorded loop, in the axis coordinates the payload's own axisFrame
// re-expressed it into, beside the certified enclosure of the (z, ρ) pair the
// RECORD denotes there and the mesh vertices it owns.
//
// A sample on the axis owns exactly ONE vertex, interned by construction: it is
// the same junction for every angular index and for both partial caps, which is
// how an on-axis line's single geometric edge ends up shared by the two caps
// (docs/tessellation-design.md §9).
type revMeridian struct {
	z, rho     float64
	zIv, rhoIv ratInterval
	onAxis     bool
	ring       []int
}

// at is the mesh vertex this sample contributes at angular index l. A pole has
// one vertex and answers it for every angle; an off-axis ring of a full turn
// wraps, so index n is index 0 and the seam needs no duplicate.
func (s revMeridian) at(l int) int {
	if s.onAxis {
		return s.ring[0]
	}
	return s.ring[l%len(s.ring)]
}

// revLoopMesh is one recorded loop's meridian polyline plus the resolution it
// came from, held together because the cells read both.
type revLoopMesh struct {
	resolved revolveWalks
	samples  []revMeridian
}

// tessellateRevolve meshes a revolved body (docs/tessellation-design.md §§8-10).
func tessellateRevolve(ctx context.Context, b *Body, rp revolvePayload, chord float64) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	byRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			byRole[o.Role] = f
		}
	}
	faceOfRole := func(role string) (*Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}

	basis := rp.basis()
	idealBasis, ok := revolveIdealBasis(rp)
	if !ok {
		return nil, fmt.Errorf(`%w: this revolve's axis basis holds a coordinate that cannot be enclosed, so the mesh can state no construction bound`, ErrUnsupported)
	}

	// One resolution of every loop, shared with the builder (revolveLoopWalks),
	// so the mesh is read off the walks the body was built from.
	work := newFreeformWork()
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	mesh := &Mesh{}
	loopMesh := make([]revLoopMesh, len(loops))
	meridianGap := 0.0
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		resolved, err := revolveLoopWalks(ctx, rp, loop, work, "revolve tessellation")
		if err != nil {
			return nil, err
		}
		if err := requireStraightGenerators(resolved); err != nil {
			return nil, err
		}
		samples, gap, err := revolveMeridianSamples(rp, resolved)
		if err != nil {
			return nil, err
		}
		meridianGap = math.Max(meridianGap, gap)
		loopMesh[li] = revLoopMesh{resolved: resolved, samples: samples}
	}
	if err := requireRevolveAxisIncidence(loopMesh); err != nil {
		return nil, err
	}
	// docs/tessellation-design.md §9's meridian section proof, run for a full
	// and a partial sweep alike and BEFORE any three-dimensional cell is
	// formed. A straight generator chords nothing along the meridian, so every
	// loop's sagitta tube is the loop itself and the endpoint clearance is the
	// whole of the proof; R4's circular generators add the tube check.
	sectionPts, sectionLoops := revolveSectionPoints(loopMesh)
	if err := requireLoopClearance(ctx, sectionPts, sectionLoops, make([]float64, len(sectionLoops))); err != nil {
		return nil, err
	}

	rhoMax, zAbsMax, err := revolveExtents(loopMesh)
	if err != nil {
		return nil, err
	}
	coordMax := revolveCoordMax(basis, zAbsMax, rhoMax)
	if isNonFinite(coordMax) || isNonFinite(meridianGap) {
		return nil, fmt.Errorf(`%w: this revolve's coordinate envelope is not finite, so no chord budget can be reserved against it`, ErrUnsupported)
	}
	deltaCPrior := revolveConstructionPrior(basis, meridianGap, rhoMax, coordMax)
	deltaRPrior := rigidRoundAllow(absSumUpper(coordMax, deltaCPrior), vecMaxAbs(rp.xform.Translation()))
	available, err := revolveBudget(chord, deltaCPrior, deltaRPrior)
	if err != nil {
		return nil, err
	}

	// A section carrying no circular meridian walk chords nothing along the
	// meridian, so deltaM is zero and the whole remaining budget is the
	// angular sequence's (docs/tessellation-design.md §8 steps 2-4).
	sweep := math.Abs(rp.phi1 - rp.phi0)
	nPhi, deltaPhi, err := chordCount(segmentWalk{radius: rhoMax, th1: sweep, closed: rp.full}, available)
	if err != nil {
		return nil, err
	}
	angular, err := revolveAngularSequence(rp.phi0, rp.phi1, rp.full, nPhi)
	if err != nil {
		return nil, err
	}

	if err := revolvePreflightFacets(loopMesh, nPhi, rp.full); err != nil {
		return nil, err
	}

	// Rings. Every sample's vertices are evaluated UNPLACED, measured against
	// the ideal enclosure, then placed once and measured again — §8's two
	// stages, apart.
	budget := newWorkBudget(ctx)
	deltaC, deltaR := 0.0, 0.0
	for li := range loopMesh {
		for si := range loopMesh[li].samples {
			s := &loopMesh[li].samples[si]
			count := angular.samples
			if s.onAxis {
				count = 1
			}
			s.ring = make([]int, count)
			for l := range count {
				if err := budget.step(); err != nil {
					return nil, err
				}
				cos, sin := angular.cos[l], angular.sin[l]
				local := basis.a3.Add(basis.w.Scale(s.z)).Add(basis.e0.Scale(cos).Add(basis.e1.Scale(sin)).Scale(s.rho))
				placed := rp.xform.Apply(local)
				if !finiteVec(local) || !finiteVec(placed) {
					return nil, fmt.Errorf(`%w: a revolve mesh vertex is not finite`, ErrUnsupported)
				}
				cosIv, sinIv := angular.cosIv[l], angular.sinIv[l]
				if s.onAxis {
					// A pole's single vertex stands for the ideal sample at
					// EVERY angle, so its enclosure must cover them all.
					cosIv = interval(minusOneRat(), oneRat())
					sinIv = interval(minusOneRat(), oneRat())
				}
				ideal := revolveIdealPoint(idealBasis, s.zIv, s.rhoIv, cosIv, sinIv)
				gapC := radius3D(max(
					intervalFloatError(ideal[0], local.X),
					intervalFloatError(ideal[1], local.Y),
					intervalFloatError(ideal[2], local.Z),
				))
				gapR := exactRigidPointRound(rp.xform, local, placed)
				if isNonFinite(gapC) || isNonFinite(gapR) {
					return nil, fmt.Errorf(`%w: a revolve mesh vertex states no bound on the rounding its own construction committed`, ErrUnsupported)
				}
				deltaC = math.Max(deltaC, gapC)
				deltaR = math.Max(deltaR, gapR)
				s.ring[l] = len(mesh.vertices)
				mesh.vertices = append(mesh.vertices, placed)
			}
		}
	}
	if deltaC > deltaCPrior || deltaR > deltaRPrior {
		return nil, fmt.Errorf(`%w: this revolve's stored coordinates sit farther from the samples they denote than the tolerance split reserved for them`, ErrUnsupported)
	}
	deltaR = math.Min(deltaR, rigidRoundAllow(absSumUpper(coordMax, deltaC), vecMaxAbs(rp.xform.Translation())))

	// Walls, cell by cell. Both rings off the axis give a planar quad on the
	// fixed diagonal; exactly one on the axis gives a fan; both on the axis is
	// a wall only an axis line may erase (docs/tessellation-design.md §9).
	faceCells := map[*Face]float64{}
	cellSlack := 0.0
	for li := range loopMesh {
		lm := loopMesh[li]
		n := len(lm.samples)
		for k, w := range lm.resolved.walks {
			if err := budget.step(); err != nil {
				return nil, err
			}
			lo, hi := lm.samples[k], lm.samples[(k+1)%n]
			if lm.resolved.kinds[k] == wallAxis {
				continue
			}
			if lo.onAxis && hi.onAxis {
				return nil, fmt.Errorf(`%w: a revolve generator with both ends on the axis sweeps no face, yet the recorded walk is not an axis line`, ErrUnsupported)
			}
			face, err := faceOfRole(fmt.Sprintf("side(%d,%d)", li, w.segs[0]))
			if err != nil {
				return nil, err
			}
			emitRevolveCell(mesh, lo, hi, nPhi, face)
			faceCells[face] = math.Max(faceCells[face], math.Max(lo.rho, hi.rho))
			slack, err := revolveCellSlack(idealBasis, angular, lo, hi)
			if err != nil {
				return nil, err
			}
			cellSlack = absSumUpper(cellSlack, productUpper(float64(nPhi), slack))
		}
	}

	// Partial caps: one shared triangulation of the meridian region in the
	// (z, ρ) plane, mapped onto the wall vertices at φ0 and φ1. Pole vertices
	// are ordinary samples there, so an on-axis line's edge is shared by both.
	if !rp.full {
		if err := emitRevolveCaps(ctx, mesh, loopMesh, sectionPts, sectionLoops, angular.samples-1, faceOfRole); err != nil {
			return nil, err
		}
	}
	if len(mesh.triangles) == 0 {
		return nil, fmt.Errorf(`%w: this revolve's recorded section sweeps no face`, ErrDegenerate)
	}

	// A reflected placement flips handedness, turning every counter-clockwise
	// winding clockwise; reversing them restores outward orientation.
	if rp.reflected() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}

	if err := requireClosedMesh(mesh); err != nil {
		return nil, fmt.Errorf(`%w: this revolve's cells do not close into a watertight boundary`, ErrUnsupported)
	}
	if err := requireVertexLinks(ctx, mesh); err != nil {
		return nil, err
	}
	if err := revolveContactAudit(newWorkBudget(ctx), mesh.vertices, mesh.triangles, absSumUpper(deltaC, deltaR)); err != nil {
		return nil, err
	}
	anchor := rp.xform.Apply(basis.a3)
	if !finiteVec(anchor) || meshOrientationSign(mesh.vertices, mesh.triangles, anchor) <= 0 {
		return nil, fmt.Errorf(`%w: this revolve's assembled cells do not enclose a positive volume`, ErrUnsupported)
	}

	if err := publishRevolveProof(mesh, faceCells, sweep, nPhi, deltaPhi, deltaC, deltaR, cellSlack, chord); err != nil {
		return nil, err
	}
	return mesh, nil
}

func oneRat() *big.Rat      { return big.NewRat(1, 1) }
func minusOneRat() *big.Rat { return big.NewRat(-1, 1) }

// requireStraightGenerators is R3's own reach gate: a circular meridian walk
// sweeps a sphere or a torus, whose cells and area proof are
// docs/tessellation-design.md §13's increment T3. Refusing is the whole of the
// staging — nothing below assumes a straight generator without this having run.
func requireStraightGenerators(r revolveWalks) error {
	if r.singleClosed {
		return fmt.Errorf(`%w: tessellating a revolve whose generator is a whole closed curve needs circular meridian cells, which this evaluator stages`, ErrUnsupported)
	}
	for _, w := range r.walks {
		if w.isCircular() {
			return fmt.Errorf(`%w: tessellating a revolve whose section carries a circular generator needs sphere and torus cells, which this evaluator stages`, ErrUnsupported)
		}
	}
	return nil
}

// revolveMeridianSamples turns one loop's resolved walks into its meridian
// polyline: sample k is walk k's own start, which is walk k−1's end, so each
// junction is emitted exactly once and the polyline closes by construction.
//
// Each sample carries the certified enclosure of the (z, ρ) the RECORD denotes
// there, read from the recorded plane-local point the axis re-expression
// consumed rather than from the re-expressed floats themselves. The returned
// gap is the largest distance any sample's stored pair sits from its own
// enclosure — the count-independent half of deltaC the tolerance split spends
// before an angular count exists.
func revolveMeridianSamples(rp revolvePayload, r revolveWalks) ([]revMeridian, float64, error) {
	out := make([]revMeridian, len(r.walks))
	worst := 0.0
	for k, w := range r.walks {
		plane := r.plane[w.segs[0]]
		zIv, rhoIv, ok := revolveMeridianEnclosure(rp.ax, plane.startU, plane.startV, plane.startBound)
		if !ok {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian sample states no enclosure of the axis coordinates its record denotes`, ErrUnsupported)
		}
		gap := math.Max(intervalFloatError(zIv, w.startU), intervalFloatError(rhoIv, w.startV))
		if isNonFinite(gap) {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian sample states no bound on its own axis coordinates`, ErrUnsupported)
		}
		worst = math.Max(worst, gap)
		if w.startV < 0 {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian sample sits on the negative side of the axis`, ErrDegenerate)
		}
		out[k] = revMeridian{z: w.startU, rho: w.startV, zIv: zIv, rhoIv: rhoIv, onAxis: w.startV == 0}
	}
	return out, worst, nil
}

// requireRevolveAxisIncidence re-runs docs/evaluator-design.md §6's exact
// axis-incidence audit over the resolved walks, which
// docs/tessellation-design.md §9 requires before any sample is emitted: a
// manifold pole has exactly one off-axis walk end and one on-axis line end from
// the same loop, and no two on-axis junctions may land on the same axis point.
// A second off-axis sector, a repeated incidence, or a missing on-axis
// continuation is ErrDegenerate — the profile itself does not revolve into a
// solid, so no tolerance can rescue it.
func requireRevolveAxisIncidence(loops []revLoopMesh) error {
	seen := map[float64]struct{}{}
	for _, lm := range loops {
		n := len(lm.samples)
		for k, s := range lm.samples {
			if !s.onAxis {
				continue
			}
			if _, dup := seen[s.z]; dup {
				return fmt.Errorf(`%w: two recorded boundary junctions meet the revolve axis at the same point, so the swept solid pinches there`, ErrDegenerate)
			}
			seen[s.z] = struct{}{}
			incoming := lm.resolved.kinds[(k+n-1)%n]
			outgoing := lm.resolved.kinds[k]
			if (incoming == wallAxis) == (outgoing == wallAxis) {
				return fmt.Errorf(`%w: the recorded boundary meets the revolve axis at a junction with %s, which sweeps no manifold pole`, ErrDegenerate, axisIncidenceReason(incoming == wallAxis))
			}
		}
	}
	return nil
}

func axisIncidenceReason(bothAxis bool) string {
	if bothAxis {
		return "two on-axis segments"
	}
	return "two off-axis segments"
}

// revolveExtents is docs/tessellation-design.md §8's ρ and |z| envelope over
// every loop. A straight generator attains both at its endpoints, so the
// meridian samples are the whole candidate set; a section with no material off
// the axis is an invariant failure the builder's own area gate already refuses.
func revolveExtents(loops []revLoopMesh) (float64, float64, error) {
	rhoMax, zAbsMax := 0.0, 0.0
	for _, lm := range loops {
		for _, s := range lm.samples {
			if isNonFinite(s.rho) || isNonFinite(s.z) {
				return 0, 0, fmt.Errorf(`%w: a revolve meridian sample is not finite`, ErrUnsupported)
			}
			rhoMax = math.Max(rhoMax, s.rho)
			zAbsMax = math.Max(zAbsMax, math.Abs(s.z))
		}
	}
	if rhoMax <= 0 {
		return 0, 0, fmt.Errorf(`%w: the recorded region lies entirely on the revolve axis, so it sweeps no solid`, ErrDegenerate)
	}
	return rhoMax, zAbsMax, nil
}

// revolvePreflightFacets charges docs/tessellation-design.md §3's per-mesh and
// cumulative facet ceilings with unsigned integer arithmetic, BEFORE a single
// vertex or facet is allocated. The cap triangles are counted from Euler's own
// identity for a polygon with holes — n + 2h − 2 triangles for n boundary
// samples and h holes — so the charge covers the whole mesh rather than its
// walls alone.
func revolvePreflightFacets(loops []revLoopMesh, nPhi int, full bool) error {
	var walls, samples uint64
	for _, lm := range loops {
		n := len(lm.samples)
		for k := range lm.resolved.walks {
			if lm.resolved.kinds[k] == wallAxis {
				continue
			}
			lo, hi := lm.samples[k], lm.samples[(k+1)%n]
			per := uint64(2)
			if lo.onAxis || hi.onAxis {
				per = 1
			}
			step, ok := mulChecked(per, uint64(nPhi))
			if !ok {
				return errRevolveFacetCeiling
			}
			walls, ok = addChecked(walls, step)
			if !ok {
				return errRevolveFacetCeiling
			}
		}
		var ok bool
		samples, ok = addChecked(samples, uint64(n))
		if !ok {
			return errRevolveFacetCeiling
		}
	}
	// A full revolution emits no cap at all; a partial sweep emits both.
	caps := uint64(0)
	if !full && samples+2*uint64(len(loops)) >= 4 {
		caps = 2 * (samples + 2*uint64(len(loops)) - 4)
	}
	total, ok := addChecked(walls, caps)
	if !ok || total > maxFacetsPerMesh || total > maxFacetWorkPerCall {
		return errRevolveFacetCeiling
	}
	// The facet-pair audit's own ceiling, charged here rather than at the audit:
	// §3 requires the conservative F·(F−1)/2 to be checked before the audit
	// starts, and checking it before the ALLOCATION is strictly earlier.
	pairs, ok := wallChoose2(total)
	if !ok || pairs > maxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: this chord tolerance asks for %d facets in one revolve mesh, whose pairwise audit exceeds the fixed ceiling of %d exact tests; retry with a coarser tolerance`, ErrUnsupported, total, maxFacetPairTestsPerCall)
	}
	return nil
}

var errRevolveFacetCeiling = fmt.Errorf(`%w: this chord tolerance asks for more than %d facets in one revolve mesh`, ErrUnsupported, maxFacetsPerMesh)

func addChecked(a, b uint64) (uint64, bool) {
	sum := a + b
	return sum, sum >= a
}

func mulChecked(a, b uint64) (uint64, bool) {
	if a == 0 || b == 0 {
		return 0, true
	}
	product := a * b
	return product, product/a == b
}

// emitRevolveCell writes one meridian cell's facets across the whole angular
// sequence (docs/tessellation-design.md §9's cell table).
//
// The winding is the one §4's rule gives: with the region's material on the
// walk's LEFT in (z, ρ) and φ increasing right-handedly about the axis,
// ∂X/∂t × ∂X/∂φ is ρ times the outward in-plane normal, so a facet wound
// counter-clockwise in (t, φ) faces outward. Nothing here corrects for the
// axis side — resolveAxisSide already folded it into the axis frame, and the
// (u, v) → (z, ρ) map is a rotation either way, so the recorded loop's own
// sense survives it — and the reflection correction is applied once, over the
// whole assembled mesh, by the caller.
func emitRevolveCell(m *Mesh, lo, hi revMeridian, nPhi int, face *Face) {
	for l := range nPhi {
		a, d := lo.at(l), lo.at(l+1)
		bb, c := hi.at(l), hi.at(l+1)
		switch {
		case lo.onAxis:
			m.addTriangle([3]int{a, bb, c}, face)
		case hi.onAxis:
			m.addTriangle([3]int{a, bb, d}, face)
		default:
			m.addTriangle([3]int{a, bb, c}, face)
			m.addTriangle([3]int{a, c, d}, face)
		}
	}
}

// emitRevolveCaps triangulates the meridian region once in the (z, ρ) plane and
// maps it onto the wall vertices at φ0 and φ1 (docs/tessellation-design.md §9).
// The (z, ρ) frame's own normal is the sweep-velocity direction, which IS the
// end cap's outward normal, so the end cap takes the triangulation as it stands
// and the start cap reverses it. Pole samples answer their one interned vertex
// for either angle, which is how an on-axis line's single geometric edge ends
// up shared by both caps.
func emitRevolveCaps(ctx context.Context, m *Mesh, loops []revLoopMesh, pts []Point2, loopIdx [][]int, last int, faceOfRole func(string) (*Face, error)) error {
	capStart, err := faceOfRole(roleCapStart)
	if err != nil {
		return err
	}
	capEnd, err := faceOfRole(roleCapEnd)
	if err != nil {
		return err
	}
	var startV, endV []int
	for _, lm := range loops {
		for _, s := range lm.samples {
			startV = append(startV, s.at(0))
			endV = append(endV, s.at(last))
		}
	}
	tris, err := triangulate2DContext(ctx, pts, loopIdx)
	if err != nil {
		return err
	}
	for _, tri := range tris {
		m.addTriangle([3]int{endV[tri[0]], endV[tri[1]], endV[tri[2]]}, capEnd)
		m.addTriangle([3]int{startV[tri[0]], startV[tri[2]], startV[tri[1]]}, capStart)
	}
	return nil
}

// revolveSectionPoints flattens every loop's meridian polyline into the one
// (z, ρ) sample array the section proof and the partial caps both read, in the
// same order the rings were allocated in — so index j of the flat array is the
// j-th sample overall and needs no second mapping.
func revolveSectionPoints(loops []revLoopMesh) ([]Point2, [][]int) {
	var pts []Point2
	var loopIdx [][]int
	for _, lm := range loops {
		base := len(pts)
		idx := make([]int, len(lm.samples))
		for k, s := range lm.samples {
			pts = append(pts, Point2{U: s.z, V: s.rho})
			idx[k] = base + k
		}
		loopIdx = append(loopIdx, idx)
	}
	return pts, loopIdx
}

// revolveCellSlack is one meridian cell's Ecell
// (docs/tessellation-design.md §10.2), stated ONCE for the cell and multiplied
// by the angular count by the caller.
//
// One evaluation answers for every angular interval because the ideal samples
// at interval l are the EXACT rotation, about the axis by l·dφ, of those at
// interval 0. A rotation is an isometry, so the true patch and the held facets
// alike are congruent across intervals and their area densities are equal — the
// enclosures differ in width alone, and this reads interval 0's.
func revolveCellSlack(b revolveBasis3Iv, angular revolveAngular, lo, hi revMeridian) (float64, error) {
	dz, drho := floatRat(hi.z-lo.z), floatRat(hi.rho-lo.rho)
	if dz == nil || drho == nil {
		return 0, errRevolveCellSlack
	}
	lenSq := intervalAdd(intervalSquare(pointInterval(dz)), intervalSquare(pointInterval(drho)))
	meridian, ok := intervalSqrt(lenSq)
	if !ok {
		return 0, errRevolveCellSlack
	}
	corner := func(s revMeridian, l int) ivVec3 {
		return revolveIdealPoint(b, s.zIv, s.rhoIv, angular.cosIv[l], angular.sinIv[l])
	}
	p00, p01 := corner(lo, 0), corner(lo, 1)
	p10, p11 := corner(hi, 0), corner(hi, 1)
	switch {
	case lo.onAxis:
		area, ok := ivTwoTriangleArea(p00, p10, p11)
		if !ok {
			return 0, errRevolveCellSlack
		}
		return revolveFanAreaSlack(hi.rho, true, meridian, angular.step, area), nil
	case hi.onAxis:
		area, ok := ivTwoTriangleArea(p00, p10, p01)
		if !ok {
			return 0, errRevolveCellSlack
		}
		return revolveFanAreaSlack(lo.rho, false, meridian, angular.step, area), nil
	default:
		lowHalf, ok0 := ivTwoTriangleArea(p00, p10, p11)
		highHalf, ok1 := ivTwoTriangleArea(p00, p11, p01)
		if !ok0 || !ok1 {
			return 0, errRevolveCellSlack
		}
		return revolveCellAreaSlack(lo.rho, hi.rho, meridian, angular.step, [2]ratInterval{lowHalf, highHalf}), nil
	}
}

var errRevolveCellSlack = fmt.Errorf(`%w: a revolve cell states no enclosure of the area its held facets and the patch they stand for differ by`, ErrUnsupported)

// publishRevolveProof writes docs/tessellation-design.md §2's proof record for
// the assembled mesh: §10.1's per-face two-sided displacement, §10.2's area
// slack, and the occupied-volume proof this increment does NOT have.
//
// A wall face carries its own angular displacement, computed at the largest
// radius its own cells reach rather than at the mesh's, plus both coordinate
// stages; a partial cap carries the coordinate stages alone, since a straight
// generator's cap trim is exact. The occupied-volume proof is §13's increment
// T4, so symDiffOK stays false and the mesh boolean refuses this operand at
// operandSymDiff rather than substituting bound × area, which §11 forbids.
func publishRevolveProof(m *Mesh, faceCells map[*Face]float64, sweep float64, nPhi int, deltaPhi, deltaC, deltaR, cellSlack, tol float64) error {
	coord := absSumUpper(deltaC, deltaR)
	if upRound(absSumUpper(deltaPhi, coord)) > tol {
		// The chording component must stay inside the requested tolerance, and
		// revolveBudget already reserved both coordinate stages out of it
		// before the count was chosen. Reaching here would mean the
		// reservation did not hold, which is a proof failure rather than a
		// coarser mesh (docs/tessellation-design.md §8).
		return fmt.Errorf(`%w: this revolve mesh's chording exceeds the tolerance its own budget reserved for it`, ErrUnsupported)
	}
	for f, rho := range faceCells {
		bound := upRound(absSumUpper(chordSagitta(rho, sweep, nPhi), coord))
		if isNonFinite(bound) {
			return fmt.Errorf(`%w: a revolve wall face's composed displacement is not finite`, ErrUnsupported)
		}
		m.setFaceBound(f, bound)
	}
	for _, f := range m.source {
		if _, ok := m.faceBound[f]; !ok {
			m.setFaceBound(f, upRound(coord))
		}
	}
	if isNonFinite(m.bound) {
		return fmt.Errorf(`%w: this revolve mesh states no finite displacement bound`, ErrUnsupported)
	}
	slack := absSumUpper(cellSlack, meshCoordAreaAllow(m, coord))
	if isNonFinite(slack) {
		return fmt.Errorf(`%w: this revolve mesh states no finite area slack`, ErrUnsupported)
	}
	m.areaSlack = slack
	m.symDiffOK = false
	return nil
}

// meshCoordAreaAllow is docs/tessellation-design.md §10.2's coordinate-stage
// area charge: every facet's own area can move by what a displacement of delta
// at each of its three corners allows, and the two stages are charged together
// against the composed displacement, which covers the ideal triangle, the
// stored unplaced one, and the placed one alike.
func meshCoordAreaAllow(m *Mesh, delta float64) float64 {
	if delta <= 0 {
		return 0
	}
	total := 0.0
	for _, tri := range m.triangles {
		a, b, c := m.vertices[tri[0]], m.vertices[tri[1]], m.vertices[tri[2]]
		total = absSumUpper(total, perturbedTriangleAreaAllow(a, b, c, delta))
	}
	return total
}

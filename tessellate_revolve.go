package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/r3"
)

// This file is docs/tessellation-design.md §13's increments T2 and T3
// (docs/tessellation-reach-design.md §6, R3 and R4): revolve tessellation. It
// assembles the mesh — the meridian samples, the one global angular sequence,
// the rings, the cylinder/cone/plane/sphere/torus cells, the poles and apexes,
// the partial caps and the full-turn cycles — while
// tessellate_revolve_proof.go proves it and tessellate_revolve_arc.go owns
// everything a CIRCULAR meridian generator needs that a straight one does not.
//
// A free-form (Tier A NURBS) revolve generator is still refused, by
// revolveLoopWalks' own requireAnalyticWalk: those cells are §13's increment
// T5. The occupied-volume proof (§11) is increment T4, so the mesh this file
// returns serves EXPORT and carries symDiffOK false — the mesh boolean refuses
// a revolve operand through operandSymDiff, which is the one gate that decides
// it.
//
// Three structural facts shape everything below, and all three are
// docs/tessellation-design.md §8's and §9's:
//
//   - The angular sequence is GLOBAL. Every off-axis ring in every loop uses
//     the same angles, so adjacent generator faces share their whole latitude
//     edge, a full turn closes with no seam ring, and the partial caps sit
//     exactly on the wall vertices at φ0 and φ1.
//   - Samples are stored UNPLACED and placed once, so the two coordinate
//     stages — construction and placement — are measured apart from one
//     another (deltaC and deltaR) rather than folded into one figure.
//   - The meridian is chorded and the angles are chorded, and the tolerance is
//     split between them BEFORE either count is chosen. Refinement may only
//     make a count larger, never smaller: a failed audit refines the first
//     failing meridian walk in payload order, or the one global angular count,
//     and refuses when the fixed budget runs out. It never snaps, welds, drops
//     a facet, or rounds a near-axis ring onto the axis (§12).

// maxFacetsPerMesh, maxFacetWorkPerCall are docs/tessellation-design.md §3's
// two per-call facet ceilings, beside budget.go's maxFacetPairTestsPerCall.
// Every one of them is checked with unsigned integer arithmetic BEFORE the
// allocation or audit it governs, so an over-budget request refuses rather
// than building the thing that would have blown the budget.
const (
	maxFacetsPerMesh    = 65_536
	maxFacetWorkPerCall = 262_144
)

// maxRevolveRefinements caps how many times one call may refine a count and
// rebuild (docs/tessellation-design.md §3: refinement is deterministic and
// bounded, and exhausting it refuses). The cumulative facet-work and pair-test
// counters do NOT reset across those attempts, so this is the second of two
// independent ceilings and whichever binds first ends the call.
const maxRevolveRefinements = 6

// revMeridian is one meridian sample: either a junction between two
// consecutive walks of one recorded loop or an interior chord station of a
// circular walk, in the axis coordinates the payload's own axisFrame
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
	// walk is the index, into its loop's resolved walks, of the walk whose
	// OUTGOING chord starts at this sample; sag is that chord's proven meridian
	// sagitta, zero for a straight walk, which chords nothing. arc is the
	// circular chord's own meridian model, nil for a straight one.
	walk int
	sag  float64
	arc  *revArcCell
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

// revolveWork is the pair of cumulative ceilings docs/tessellation-design.md §3
// forbids a refinement retry from resetting: every facet assembled across every
// attempt, and every exact facet-pair predicate charged across all of them.
type revolveWork struct {
	facets uint64
	pairs  uint64
}

// revolveRefine names the deterministic refinement a failed attempt asks for:
// one meridian walk of one loop, or — with a negative loop — the single global
// angular count.
type revolveRefine struct {
	loop, walk int
}

// revolveRefineError is a refusal a finer chording may still answer. The
// orchestrator retries it against maxRevolveRefinements and the cumulative work
// ceilings; the wrapped error is what a caller sees once those run out, so the
// refusal a user reads is the one the last attempt actually hit.
type revolveRefineError struct {
	err   error
	retry revolveRefine
}

func (e *revolveRefineError) Error() string { return e.err.Error() }
func (e *revolveRefineError) Unwrap() error { return e.err }

// revolvePlan is one revolve's count-independent resolution beside the counts
// the current attempt is building at. Everything above the counts is computed
// once; the counts and the two chording displacements below them are what a
// refinement moves.
type revolvePlan struct {
	rp        revolvePayload
	basis     revolveBasis
	ideal     revolveBasis3Iv
	loops     []LoopRecord
	resolved  []revolveWalks
	junctions [][]revMeridian
	faceOf    func(string) (*Face, error)
	work      *revolveWork

	counts [][]int
	sags   [][]float64
	nPhi   int

	deltaM      float64
	deltaPhi    float64
	deltaCPrior float64
	deltaRPrior float64
	samplePrior float64
	rhoMax      float64
	coordMax    float64
	sweep       float64
	chord       float64
}

// tessellateRevolve meshes a revolved body (docs/tessellation-design.md §§8-10).
func tessellateRevolve(ctx context.Context, b *Body, rp revolvePayload, chord float64) (*Mesh, error) {
	plan, err := planRevolve(ctx, b, rp, chord)
	if err != nil {
		return nil, err
	}
	for attempt := 0; ; attempt++ {
		mesh, err := buildRevolveMesh(ctx, plan)
		if err == nil {
			return mesh, nil
		}
		var refine *revolveRefineError
		if !errors.As(err, &refine) {
			return nil, err
		}
		// The refinement wrapper is this file's own bookkeeping: a caller that
		// runs out of attempts reads the refusal the last attempt actually hit,
		// with nothing of the retry machinery in its chain.
		if attempt >= maxRevolveRefinements {
			return nil, refine.err
		}
		if rerr := plan.refine(refine.retry); rerr != nil {
			return nil, refine.err
		}
	}
}

// refine applies one deterministic refinement step and recomputes the two
// chording displacements it moved. Every count only ever GROWS, and both
// sagittas shrink strictly with their count, so a retry can never spend more
// tolerance than the attempt before it did.
func (p *revolvePlan) refine(r revolveRefine) error {
	if r.loop < 0 {
		n := p.nPhi + 1
		if n > maxChordsPerWalk {
			return errTooManyChords
		}
		p.nPhi = n
		p.deltaPhi = chordSagitta(p.rhoMax, p.sweep, n)
		return nil
	}
	w := p.resolved[r.loop].walks[r.walk]
	if !w.isCircular() {
		return fmt.Errorf(`%w: a straight revolve generator carries no meridian chording to refine`, ErrUnsupported)
	}
	n := p.counts[r.loop][r.walk] + 1
	if n > maxChordsPerWalk {
		return errTooManyChords
	}
	p.counts[r.loop][r.walk] = n
	p.sags[r.loop][r.walk] = chordSagitta(w.radius, math.Abs(w.th1-w.th0), n)
	p.deltaM = 0
	for li := range p.sags {
		for _, s := range p.sags[li] {
			p.deltaM = math.Max(p.deltaM, s)
		}
	}
	return nil
}

// planRevolve resolves the payload once and spends docs/tessellation-design.md
// §8's tolerance split in its stated order: both coordinate stages are reserved
// against count-independent ceilings, the meridian takes half of what is left
// and chords every circular walk, and the angular sequence takes the remainder.
func planRevolve(ctx context.Context, b *Body, rp revolvePayload, chord float64) (*revolvePlan, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	byRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			byRole[o.Role] = f
		}
	}
	faceOf := func(role string) (*Face, error) {
		f, ok := byRole[role]
		if !ok {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, role)
		}
		return f, nil
	}

	ideal, ok := revolveIdealBasis(rp)
	if !ok {
		return nil, fmt.Errorf(`%w: this revolve's axis basis holds a coordinate that cannot be enclosed, so the mesh can state no construction bound`, ErrUnsupported)
	}

	// One resolution of every loop, shared with the builder (revolveLoopWalks),
	// so the mesh is read off the walks the body was built from.
	work := newFreeformWork()
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	resolved := make([]revolveWalks, len(loops))
	junctions := make([][]revMeridian, len(loops))
	junctionGap := 0.0
	for li, loop := range loops {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		r, err := revolveLoopWalks(ctx, rp, loop, work, "revolve tessellation")
		if err != nil {
			return nil, err
		}
		js, gap, err := revolveJunctions(rp, r)
		if err != nil {
			return nil, err
		}
		junctionGap = math.Max(junctionGap, gap)
		resolved[li], junctions[li] = r, js
	}
	if err := requireRevolveAxisIncidence(resolved, junctions); err != nil {
		return nil, err
	}

	rhoMax, zAbsMax, err := revolveExtents(resolved)
	if err != nil {
		return nil, err
	}
	basis := rp.basis()
	coordMax := revolveCoordMax(basis, zAbsMax, rhoMax)
	// A chorded meridian station is stored as the float nearest its own
	// certified enclosure, so its gap is bounded before any count exists; a
	// junction's gap is already count-independent and is measured outright.
	stationPrior := productUpper(revolveStationRoundUlps, ulpOf(math.Max(math.Max(zAbsMax, rhoMax), 1)))
	samplePrior := math.Max(junctionGap, stationPrior)
	if isNonFinite(coordMax) || isNonFinite(samplePrior) {
		return nil, fmt.Errorf(`%w: this revolve's coordinate envelope is not finite, so no chord budget can be reserved against it`, ErrUnsupported)
	}
	deltaCPrior := revolveConstructionPrior(basis, samplePrior, rhoMax, coordMax)
	deltaRPrior := rigidRoundAllow(absSumUpper(coordMax, deltaCPrior), vecMaxAbs(rp.xform.Translation()))
	available, err := revolveBudget(chord, deltaCPrior, deltaRPrior)
	if err != nil {
		return nil, err
	}

	// §8 steps 2-3: the meridian takes half the remaining budget and chords
	// every circular walk; deltaM is the largest sagitta those choices prove.
	meridian := downRound(available / 2)
	counts := make([][]int, len(resolved))
	sags := make([][]float64, len(resolved))
	deltaM := 0.0
	for li, r := range resolved {
		counts[li] = make([]int, len(r.walks))
		sags[li] = make([]float64, len(r.walks))
		for k, w := range r.walks {
			counts[li][k] = 1
			if !w.isCircular() {
				continue
			}
			n, sag, err := chordCount(w.segmentWalk, meridian, revolveMeridianMin(w.segmentWalk))
			if err != nil {
				return nil, err
			}
			counts[li][k], sags[li][k] = n, sag
			deltaM = math.Max(deltaM, sag)
		}
	}

	// §8 steps 4-5: the angular sequence takes what the meridian left.
	sweep := math.Abs(rp.phi1 - rp.phi0)
	angular := downRound(available - deltaM)
	if angular <= 0 || isNonFinite(angular) {
		return nil, fmt.Errorf(`%w: this revolve's meridian chording spends the whole chord budget its tolerance left, so no angular count remains; retry with a coarser tolerance`, ErrUnsupported)
	}
	angularWalk := segmentWalk{radius: rhoMax, th1: sweep, closed: rp.full}
	nPhi, deltaPhi, err := chordCount(angularWalk, angular, chordWalkMin(angularWalk))
	if err != nil {
		return nil, err
	}

	return &revolvePlan{
		rp: rp, basis: basis, ideal: ideal, loops: loops, resolved: resolved,
		junctions: junctions, faceOf: faceOf, work: &revolveWork{},
		counts: counts, sags: sags, nPhi: nPhi,
		deltaM: deltaM, deltaPhi: deltaPhi,
		deltaCPrior: deltaCPrior, deltaRPrior: deltaRPrior, samplePrior: samplePrior,
		rhoMax: rhoMax, coordMax: coordMax, sweep: sweep, chord: chord,
	}, nil
}

// revolveMeridianMin is docs/tessellation-design.md §9's meridian minimum:
// three chords for a whole closed generator, so it bounds a polygon; TWO for a
// circular generator whose two ends both sit on the axis — a sphere meridian —
// so it cannot chord to a single on-axis segment; and one otherwise.
func revolveMeridianMin(w segmentWalk) int {
	if w.closed {
		return 3
	}
	if w.startV == 0 && w.endV == 0 {
		return 2
	}
	return 1
}

// buildRevolveMesh is one attempt at the whole mesh, at the plan's current
// counts. A failure a finer chording could still answer is wrapped in a
// revolveRefineError naming what to refine; every other failure is final.
func buildRevolveMesh(ctx context.Context, p *revolvePlan) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	rp := p.rp
	mesh := &Mesh{}
	loopMesh := make([]revLoopMesh, len(p.resolved))
	sampleGap := 0.0
	for li := range p.resolved {
		samples, gap, err := revolveMeridianSamples(rp, p.loops[li], p.resolved[li], p.junctions[li], p.counts[li], p.sags[li])
		if err != nil {
			return nil, err
		}
		sampleGap = math.Max(sampleGap, gap)
		loopMesh[li] = revLoopMesh{resolved: p.resolved[li], samples: samples}
	}
	if sampleGap > p.samplePrior {
		return nil, fmt.Errorf(`%w: a revolve meridian sample sits farther from the axis coordinates its record denotes than the tolerance split reserved for it`, ErrUnsupported)
	}
	if err := requireRevolveMeridianOffAxis(loopMesh); err != nil {
		return nil, err
	}

	// docs/tessellation-design.md §9's meridian section proof, run for a full
	// and a partial sweep alike and BEFORE any three-dimensional cell is
	// formed: every loop simple and correctly nested, and every non-adjacent
	// chord pair — across loops and WITHIN one loop — clear of the two sagitta
	// tubes the analytic-to-chord homotopy moves inside.
	sectionPts, sectionLoops, sectionSag := revolveSectionPoints(loopMesh)
	if err := requireLoopClearance(ctx, sectionPts, sectionLoops, loopMaxSagitta(sectionSag)); err != nil {
		return nil, revolveSectionRetry(loopMesh, err)
	}
	if err := requireWalkClearance(ctx, sectionPts, sectionLoops, sectionSag); err != nil {
		return nil, revolveSectionRetry(loopMesh, err)
	}

	if err := revolvePreflightFacets(loopMesh, p.nPhi, rp.full, p.work); err != nil {
		return nil, err
	}
	angular, err := revolveAngularSequence(rp.phi0, rp.phi1, rp.full, p.nPhi)
	if err != nil {
		return nil, err
	}

	// Rings. Every sample's vertices are evaluated UNPLACED, measured against
	// the ideal enclosure, then placed once and measured again — §8's two
	// stages, apart.
	budget := newWorkBudget(ctx)
	deltaC, deltaR := 0.0, 0.0
	// radials[l] is the ideal radial direction at angular index l, the term
	// of revolveIdealPoint's sum that every ring shares; the pole's covers
	// every angle at once.
	radials := make([]ivVec3, angular.samples)
	for l := range angular.samples {
		radials[l] = ivVec3Add(ivVec3Mul(p.ideal.e0, angular.cosIv[l]), ivVec3Mul(p.ideal.e1, angular.sinIv[l]))
	}
	poleIv := interval(minusOneRat(), oneRat())
	poleRadial := ivVec3Add(ivVec3Mul(p.ideal.e0, poleIv), ivVec3Mul(p.ideal.e1, poleIv))
	for li := range loopMesh {
		for si := range loopMesh[li].samples {
			s := &loopMesh[li].samples[si]
			count := angular.samples
			if s.onAxis {
				count = 1
			}
			s.ring = make([]int, count)
			axial := ivVec3Mul(p.ideal.w, s.zIv)
			// docs/tessellation-design.md §9's ring-collapse detection, run
			// BEFORE and AFTER placement: a sample with ρ > 0 whose angular
			// vertices coincide is not an axis sample, and §12 forbids merging
			// it into a pole. Both stages are compared because either can
			// collapse a ring the other keeps apart.
			var prevLocal, prevPlaced r3.Vec
			var firstLocal, firstPlaced r3.Vec
			for l := range count {
				if err := budget.step(); err != nil {
					return nil, err
				}
				cos, sin := angular.cos[l], angular.sin[l]
				local := p.basis.a3.Add(p.basis.w.Scale(s.z)).Add(p.basis.e0.Scale(cos).Add(p.basis.e1.Scale(sin)).Scale(s.rho))
				placed := rp.xform.Apply(local)
				if !finiteVec(local) || !finiteVec(placed) {
					return nil, fmt.Errorf(`%w: a revolve mesh vertex is not finite`, ErrUnsupported)
				}
				radial := radials[l]
				if s.onAxis {
					// A pole's single vertex stands for the ideal sample at
					// EVERY angle, so its enclosure must cover them all.
					radial = poleRadial
				}
				ideal := ivVec3Add(p.ideal.a3, ivVec3Add(axial, ivVec3Mul(radial, s.rhoIv)))
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
				if l == 0 {
					firstLocal, firstPlaced = local, placed
				} else if !s.onAxis && (local == prevLocal || placed == prevPlaced) {
					return nil, errRevolveRingCollapse
				}
				prevLocal, prevPlaced = local, placed
				s.ring[l] = len(mesh.vertices)
				mesh.vertices = append(mesh.vertices, placed)
			}
			// A full turn closes onto its own first vertex, so the wrap is the
			// one adjacent pair the walk above never compared.
			if rp.full && !s.onAxis && count > 1 && (prevLocal == firstLocal || prevPlaced == firstPlaced) {
				return nil, errRevolveRingCollapse
			}
		}
	}
	if deltaC > p.deltaCPrior || deltaR > p.deltaRPrior {
		return nil, fmt.Errorf(`%w: this revolve's stored coordinates sit farther from the samples they denote than the tolerance split reserved for them`, ErrUnsupported)
	}
	deltaR = math.Min(deltaR, rigidRoundAllow(absSumUpper(p.coordMax, deltaC), vecMaxAbs(rp.xform.Translation())))
	coord := absSumUpper(deltaC, deltaR)

	// Walls, cell by cell. Both rings off the axis give a planar quad on the
	// fixed diagonal; exactly one on the axis gives a fan; both on the axis is
	// a wall only an axis line may erase (docs/tessellation-design.md §9).
	faceCells := map[*Face]revFaceExtent{}
	cellSlack := 0.0
	for li := range loopMesh {
		lm := loopMesh[li]
		n := len(lm.samples)
		for j := range lm.samples {
			if err := budget.step(); err != nil {
				return nil, err
			}
			lo, hi := lm.samples[j], lm.samples[(j+1)%n]
			k := lo.walk
			if lm.resolved.kinds[k] == wallAxis {
				continue
			}
			if lo.onAxis && hi.onAxis {
				return nil, fmt.Errorf(`%w: a revolve generator with both ends on the axis sweeps no face, yet the recorded walk is not an axis line`, ErrUnsupported)
			}
			face, err := p.faceOf(fmt.Sprintf("side(%d,%d)", li, lm.resolved.walks[k].segs[0]))
			if err != nil {
				return nil, err
			}
			emitRevolveCell(mesh, lo, hi, p.nPhi, face)
			cur := faceCells[face]
			cur.rho = math.Max(cur.rho, math.Max(lo.rho, hi.rho))
			cur.sag = math.Max(cur.sag, lo.sag)
			faceCells[face] = cur
			slack, err := revolveCellSlack(p.ideal, angular, lo, hi, coord)
			if err != nil {
				return nil, err
			}
			cellSlack = absSumUpper(cellSlack, productUpper(float64(p.nPhi), slack))
		}
	}

	// Partial caps: one shared triangulation of the meridian region in the
	// (z, ρ) plane, mapped onto the wall vertices at φ0 and φ1. Pole vertices
	// are ordinary samples there, so an on-axis line's edge is shared by both.
	// Their curved trim omits one circular segment per meridian chord, per cap.
	if !rp.full {
		if err := emitRevolveCaps(ctx, mesh, loopMesh, sectionPts, sectionLoops, angular.samples-1, p.faceOf); err != nil {
			return nil, err
		}
		cellSlack = absSumUpper(cellSlack, productUpper(2, revolveCapSegmentArea(p)))
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
	if err := revolveContactAudit(newWorkBudget(ctx), mesh.vertices, mesh.triangles, coord); err != nil {
		// A crossing or an undecided contact is the one failure a finer
		// angular sequence can still answer, and §3 makes the global count the
		// thing an angular failure increments. A canceled context or an
		// exhausted work budget is not that failure and is returned unchanged.
		if !errors.Is(err, ErrUnsupported) {
			return nil, err
		}
		return nil, &revolveRefineError{err: err, retry: revolveRefine{loop: -1}}
	}
	anchor := rp.xform.Apply(p.basis.a3)
	if !finiteVec(anchor) || meshOrientationSign(mesh.vertices, mesh.triangles, anchor) <= 0 {
		return nil, fmt.Errorf(`%w: this revolve's assembled cells do not enclose a positive volume`, ErrUnsupported)
	}

	if err := publishRevolveProof(mesh, faceCells, p, deltaC, deltaR, cellSlack); err != nil {
		return nil, err
	}
	return mesh, nil
}

func oneRat() *big.Rat      { return big.NewRat(1, 1) }
func minusOneRat() *big.Rat { return big.NewRat(-1, 1) }

var errRevolveRingCollapse = fmt.Errorf(`%w: a revolve ring at a positive radius collapses onto itself at this angular count, and docs/tessellation-design.md §9 forbids merging it into a pole; retry with a coarser tolerance`, ErrUnsupported)

// revFaceExtent is what one source face's own §10.1 bound reads: the largest
// radius any of its cells reaches, which sets its angular displacement, and the
// largest meridian sagitta any of them carries.
type revFaceExtent struct {
	rho, sag float64
}

// revolveJunctions is one loop's count-independent meridian samples: junction k
// is walk k's own start, which is walk k−1's end, so each junction is emitted
// exactly once and the polyline closes by construction.
//
// Each carries the certified enclosure of the (z, ρ) the RECORD denotes there,
// read from the recorded plane-local point the axis re-expression consumed
// rather than from the re-expressed floats themselves. The returned gap is the
// largest distance any junction's stored pair sits from its own enclosure — the
// count-independent half of deltaC the tolerance split spends before any count
// exists.
func revolveJunctions(rp revolvePayload, r revolveWalks) ([]revMeridian, float64, error) {
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
		out[k] = revMeridian{z: w.startU, rho: w.startV, zIv: zIv, rhoIv: rhoIv, onAxis: w.startV == 0, walk: k}
	}
	return out, worst, nil
}

// revolveMeridianSamples expands one loop's junctions into the polyline the
// current counts ask for: walk k contributes its own junction plus, for a
// CIRCULAR walk chorded into counts[k] pieces, that walk's interior stations,
// each enclosed at the recorded parameter it denotes (revolveArcStation).
//
// The returned gap is the largest distance any sample's stored pair sits from
// its own enclosure, junctions and stations alike; the caller holds it against
// what the tolerance split reserved.
func revolveMeridianSamples(rp revolvePayload, loop LoopRecord, r revolveWalks, junctions []revMeridian, counts []int, sags []float64) ([]revMeridian, float64, error) {
	out := make([]revMeridian, 0, len(junctions))
	worst := 0.0
	for k, w := range r.walks {
		n := counts[k]
		if n <= 0 {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian walk carries no chord`, ErrUnsupported)
		}
		start := junctions[k]
		start.sag, start.walk = sags[k], k
		if !w.isCircular() {
			out = append(out, start)
			continue
		}
		cell, ok := revolveArcChordCell(w.segmentWalk, 0, n)
		if !ok {
			return nil, 0, errRevolveArcCellSlack
		}
		start.arc = cell
		out = append(out, start)
		for i := 1; i < n; i++ {
			station, gap, err := revolveArcStation(rp.ax, loop.Segments[w.segs[0]], i, n)
			if err != nil {
				return nil, 0, err
			}
			cell, ok := revolveArcChordCell(w.segmentWalk, i, n)
			if !ok {
				return nil, 0, errRevolveArcCellSlack
			}
			station.walk, station.sag, station.arc = k, sags[k], cell
			worst = math.Max(worst, gap)
			out = append(out, station)
		}
	}
	return out, worst, nil
}

// requireRevolveMeridianOffAxis is docs/tessellation-design.md §9's rule that a
// circular generator with positive interior ρ may not chord to an axis-only
// polyline: a sphere meridian whose two ends are both on the axis MUST produce
// at least one proven off-axis interior sample. Failing it asks for a finer
// chording of that walk, and refuses when the refinement budget runs out.
func requireRevolveMeridianOffAxis(loops []revLoopMesh) error {
	for li, lm := range loops {
		offAxis := map[int]bool{}
		for _, s := range lm.samples {
			if !s.onAxis {
				offAxis[s.walk] = true
			}
		}
		for k, w := range lm.resolved.walks {
			if !w.isCircular() || lm.resolved.kinds[k] == wallAxis || offAxis[k] {
				continue
			}
			return &revolveRefineError{
				err:   fmt.Errorf(`%w: a circular revolve generator chords to a polyline lying entirely on the axis, which sweeps no face`, ErrUnsupported),
				retry: revolveRefine{loop: li, walk: k},
			}
		}
	}
	return nil
}

// revolveSectionRetry turns a section-proof refusal into the deterministic
// refinement docs/tessellation-design.md §3 names: the FIRST FAILING meridian
// walk in payload order.
//
// A within-loop gate names the two chords it refused, so the walk one of them
// belongs to is the one refined. A cross-loop gate names no walk, so it falls
// back to the first circular walk in payload order. Either way only a CIRCULAR
// walk is worth refining: a straight generator chords nothing along the
// meridian and its tube is already zero, so a section of straight generators
// alone states a refusal that is final and is returned unchanged. A refusal
// that is not the clearance gate's — a canceled context, an exhausted work
// budget — is returned unchanged too.
func revolveSectionRetry(loops []revLoopMesh, err error) error {
	if !errors.Is(err, ErrDegenerate) {
		return err
	}
	var named *sectionClearanceError
	if errors.As(err, &named) && named.loop < len(loops) {
		lm := loops[named.loop]
		for _, j := range [2]int{named.a, named.b} {
			if j < 0 || j >= len(lm.samples) {
				continue
			}
			k := lm.samples[j].walk
			if lm.resolved.walks[k].isCircular() {
				return &revolveRefineError{err: err, retry: revolveRefine{loop: named.loop, walk: k}}
			}
		}
	}
	for li, lm := range loops {
		for k, w := range lm.resolved.walks {
			if w.isCircular() {
				return &revolveRefineError{err: err, retry: revolveRefine{loop: li, walk: k}}
			}
		}
	}
	return err
}

// requireRevolveAxisIncidence re-runs docs/evaluator-design.md §6's exact
// axis-incidence audit over the resolved walks, which
// docs/tessellation-design.md §9 requires before any sample is emitted: a
// manifold pole has exactly one off-axis walk end and one on-axis line end from
// the same loop, and no two on-axis junctions may land on the same axis point.
// A second off-axis sector, a repeated incidence, or a missing on-axis
// continuation is ErrDegenerate — the profile itself does not revolve into a
// solid, so no tolerance can rescue it.
//
// It reads the JUNCTIONS alone. An interior chord station never sits on the
// axis (revolveArcStation refuses one that does), so a chorded meridian adds no
// incidence this audit could miss.
func requireRevolveAxisIncidence(resolved []revolveWalks, junctions [][]revMeridian) error {
	seen := map[float64]struct{}{}
	for li, js := range junctions {
		n := len(js)
		for k, s := range js {
			if !s.onAxis {
				continue
			}
			if _, dup := seen[s.z]; dup {
				return fmt.Errorf(`%w: two recorded boundary junctions meet the revolve axis at the same point, so the swept solid pinches there`, ErrDegenerate)
			}
			seen[s.z] = struct{}{}
			incoming := resolved[li].kinds[(k+n-1)%n]
			outgoing := resolved[li].kinds[k]
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
// every loop, read from the WALKS rather than from a chording of them: a
// straight generator attains both at its endpoints, and a circular one at its
// endpoints plus every cardinal point its own parameter interval contains. A
// cardinal point needs no trig — the four of them are (cU ± r, cV) and
// (cU, cV ± r) exactly — so this envelope carries no library assumption.
//
// A section with no material off the axis is an invariant failure the builder's
// own area gate already refuses.
func revolveExtents(loops []revolveWalks) (float64, float64, error) {
	rhoMax, zAbsMax := 0.0, 0.0
	see := func(z, rho float64) error {
		if isNonFinite(rho) || isNonFinite(z) {
			return fmt.Errorf(`%w: a revolve meridian sample is not finite`, ErrUnsupported)
		}
		rhoMax = math.Max(rhoMax, rho)
		zAbsMax = math.Max(zAbsMax, math.Abs(z))
		return nil
	}
	for _, r := range loops {
		for _, w := range r.walks {
			for _, p := range revolveWalkExtremes(w.segmentWalk) {
				if err := see(p[0], p[1]); err != nil {
					return 0, 0, err
				}
			}
		}
	}
	if rhoMax <= 0 {
		return 0, 0, fmt.Errorf(`%w: the recorded region lies entirely on the revolve axis, so it sweeps no solid`, ErrDegenerate)
	}
	return rhoMax, zAbsMax, nil
}

// revolveWalkExtremes lists the (z, ρ) points where one walk can attain either
// envelope: its two endpoints, plus, for a circular walk, each cardinal point
// its own angular interval contains.
func revolveWalkExtremes(w segmentWalk) [][2]float64 {
	out := [][2]float64{{w.startU, w.startV}, {w.endU, w.endV}}
	if !w.isCircular() {
		return out
	}
	span := math.Abs(w.th1 - w.th0)
	lo := math.Min(w.th0, w.th1)
	cardinals := [4][2]float64{
		{w.cU + w.radius, w.cV},
		{w.cU, w.cV + w.radius},
		{w.cU - w.radius, w.cV},
		{w.cU, w.cV - w.radius},
	}
	for q, p := range cardinals {
		// The cardinal's own angle is q·π/2; shift it into [lo, lo+2π) and keep
		// it when the walk's interval reaches that far.
		d := math.Mod(float64(q)*math.Pi/2-lo, 2*math.Pi)
		if d < 0 {
			d += 2 * math.Pi
		}
		if d <= span {
			out = append(out, p)
		}
	}
	return out
}

// revolveCapSegmentArea is the circular-segment area ONE partial cap's curved
// trim omits (docs/tessellation-design.md §10.2): the chorded meridian region
// differs from the region it denotes by exactly the segments between each
// circular walk's arc and its chords, summed in ABSOLUTE value because a hole
// gains what an outline loses and area slack admits no cancellation.
func revolveCapSegmentArea(p *revolvePlan) float64 {
	total := 0.0
	for li, r := range p.resolved {
		for k, w := range r.walks {
			if !w.isCircular() {
				continue
			}
			total = absSumUpper(total, chordSegmentArea(w.radius, math.Abs(w.th1-w.th0), p.counts[li][k]))
		}
	}
	return total
}

// revolvePreflightFacets charges docs/tessellation-design.md §3's per-mesh and
// cumulative facet ceilings with unsigned integer arithmetic, BEFORE a single
// facet is allocated. The cap triangles are counted from Euler's own identity
// for a polygon with holes — n + 2h − 2 triangles for n boundary samples and h
// holes — so the charge covers the whole mesh rather than its walls alone.
//
// The cumulative counters live on the call's revolveWork and are never reset by
// a refinement retry, so a sequence of attempts is charged the sum of what each
// of them asked for.
func revolvePreflightFacets(loops []revLoopMesh, nPhi int, full bool, work *revolveWork) error {
	var walls, samples uint64
	for _, lm := range loops {
		n := len(lm.samples)
		for j, s := range lm.samples {
			if lm.resolved.kinds[s.walk] == wallAxis {
				continue
			}
			per := uint64(2)
			if s.onAxis || lm.samples[(j+1)%n].onAxis {
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
	if !ok || total > maxFacetsPerMesh {
		return errRevolveFacetCeiling
	}
	spent, ok := addChecked(work.facets, total)
	if !ok || spent > maxFacetWorkPerCall {
		return errRevolveFacetCeiling
	}
	// The facet-pair audit's own ceiling, charged here rather than at the audit:
	// §3 requires the conservative F·(F−1)/2 to be checked before the audit
	// starts, and checking it before the ALLOCATION is strictly earlier.
	pairs, ok := wallChoose2(total)
	if !ok {
		return errRevolveFacetCeiling
	}
	charged, ok := addChecked(work.pairs, pairs)
	if !ok || charged > maxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: this chord tolerance asks for %d facets in one revolve mesh, whose pairwise audit exceeds the fixed ceiling of %d exact tests; retry with a coarser tolerance`, ErrUnsupported, total, maxFacetPairTestsPerCall)
	}
	work.facets, work.pairs = spent, charged
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
// j-th sample overall and needs no second mapping. The third result is each
// sample's OUTGOING chord sagitta, which is the tube half the section proof
// gives that chord.
func revolveSectionPoints(loops []revLoopMesh) ([]Point2, [][]int, [][]float64) {
	var pts []Point2
	var loopIdx [][]int
	var loopSag [][]float64
	for _, lm := range loops {
		base := len(pts)
		idx := make([]int, len(lm.samples))
		sag := make([]float64, len(lm.samples))
		for k, s := range lm.samples {
			pts = append(pts, Point2{U: s.z, V: s.rho})
			idx[k] = base + k
			sag[k] = s.sag
		}
		loopIdx = append(loopIdx, idx)
		loopSag = append(loopSag, sag)
	}
	return pts, loopIdx, loopSag
}

// loopMaxSagitta reduces the per-chord sagittas to the per-loop figure the
// cross-loop clearance gate reads.
func loopMaxSagitta(sag [][]float64) []float64 {
	out := make([]float64, len(sag))
	for i, loop := range sag {
		for _, s := range loop {
			out[i] = math.Max(out[i], s)
		}
	}
	return out
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
//
// A STRAIGHT generator's densities collapse to a difference that is linear in
// the meridian parameter, so its cell is decomposed in closed form
// (revolveCellAreaSlack, tess §15's T2 choice). A CIRCULAR generator's does
// not, so its cell takes certified interval subdivision instead
// (revolveArcCellSlack, tess §15's T3 choice). coord is the composed coordinate
// displacement, which the circular arms widen their meridian model by.
func revolveCellSlack(b revolveBasis3Iv, angular revolveAngular, lo, hi revMeridian, coord float64) (float64, error) {
	corner := func(s revMeridian, l int) ivVec3 {
		return revolveIdealPoint(b, s.zIv, s.rhoIv, angular.cosIv[l], angular.sinIv[l])
	}
	p00, p01 := corner(lo, 0), corner(lo, 1)
	p10, p11 := corner(hi, 0), corner(hi, 1)
	if lo.arc != nil {
		switch {
		case lo.onAxis:
			area, ok := ivTwoTriangleArea(p00, p10, p11)
			if !ok {
				return 0, errRevolveArcCellSlack
			}
			return revolveArcFanSlack(*lo.arc, true, angular.step, area, coord)
		case hi.onAxis:
			area, ok := ivTwoTriangleArea(p00, p10, p01)
			if !ok {
				return 0, errRevolveArcCellSlack
			}
			return revolveArcFanSlack(*lo.arc, false, angular.step, area, coord)
		default:
			lowHalf, ok0 := ivTwoTriangleArea(p00, p10, p11)
			highHalf, ok1 := ivTwoTriangleArea(p00, p11, p01)
			if !ok0 || !ok1 {
				return 0, errRevolveArcCellSlack
			}
			return revolveArcCellSlack(*lo.arc, angular.step, [2]ratInterval{lowHalf, highHalf}, coord)
		}
	}

	dz, drho := floatRat(hi.z-lo.z), floatRat(hi.rho-lo.rho)
	if dz == nil || drho == nil {
		return 0, errRevolveCellSlack
	}
	lenSq := intervalAdd(intervalSquare(pointInterval(dz)), intervalSquare(pointInterval(drho)))
	meridian, ok := intervalSqrt(lenSq)
	if !ok {
		return 0, errRevolveCellSlack
	}
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
// A wall face carries its own meridian and angular displacement, each computed
// at the largest figure its own cells reach rather than at the mesh's, plus
// both coordinate stages; a partial cap carries the mesh's meridian
// displacement and the coordinate stages, since a cap's own trim is exact
// wherever the meridian is straight. The occupied-volume proof is §13's
// increment T4, so symDiffOK stays false and the mesh boolean refuses this
// operand at operandSymDiff rather than substituting bound × area, which §11
// forbids.
func publishRevolveProof(m *Mesh, faceCells map[*Face]revFaceExtent, p *revolvePlan, deltaC, deltaR, cellSlack float64) error {
	coord := absSumUpper(deltaC, deltaR)
	if upRound(absSumUpper(p.deltaM, p.deltaPhi, coord)) > p.chord {
		// The chording component must stay inside the requested tolerance, and
		// revolveBudget already reserved both coordinate stages out of it
		// before the counts were chosen. Reaching here would mean the
		// reservation did not hold, which is a proof failure rather than a
		// coarser mesh (docs/tessellation-design.md §8).
		return fmt.Errorf(`%w: this revolve mesh's chording exceeds the tolerance its own budget reserved for it`, ErrUnsupported)
	}
	for f, ext := range faceCells {
		bound := upRound(absSumUpper(ext.sag, chordSagitta(ext.rho, p.sweep, p.nPhi), coord))
		if isNonFinite(bound) {
			return fmt.Errorf(`%w: a revolve wall face's composed displacement is not finite`, ErrUnsupported)
		}
		m.setFaceBound(f, bound)
	}
	capBound := upRound(absSumUpper(p.deltaM, coord))
	for _, f := range m.source {
		if _, ok := m.faceBound[f]; !ok {
			m.setFaceBound(f, capBound)
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
// against the composed displacement, which covers the ideal, stored unplaced
// and placed triangles alike.
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

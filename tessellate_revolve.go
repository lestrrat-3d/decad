package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// This file is docs/tessellation-design.md §13's increments T2 and T3
// (docs/tessellation-reach-design.md §6, R3 and R4): revolve tessellation. It
// assembles the meridian samples, global angular sequence, rings, cells, poles
// and caps. internal/revolveproof computes envelopes, budgets, cell area slack
// and volume bounds. tessellate_revolve_proof.go wires the facet audits, and
// tessellate_revolve_arc.go handles circular meridian generators.
//
// A free-form (Tier A NURBS) revolve generator is still refused, by
// revolveLoopWalks' own requireAnalyticWalk: those cells are §13's increment
// T5.
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

// maxRevolveRefinements caps how many times one call may refine a count and
// rebuild (docs/tessellation-design.md §3: refinement is deterministic and
// bounded, and exhausting it refuses). The cumulative facet-work and pair-test
// counters do NOT reset across those attempts, so this is the second of two
// independent ceilings and whichever binds first ends the call.
const maxRevolveRefinements = 6

// revLoopMesh is one recorded loop's meridian polyline plus the resolution it
// came from, held together because the cells read both.
type revLoopMesh struct {
	resolved revolveWalks
	samples  []revolvemesh.RevMeridian
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
	body      *Body
	rp        revolvePayload
	basis     revolvemesh.RevolveBasis
	ideal     revolvemesh.RevolveBasis3Iv
	loops     []LoopRecord
	resolved  []revolveWalks
	junctions [][]revolvemesh.RevMeridian
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
	// verify is how much of docs/tessellation-design.md §1's proof this build
	// runs: below VerifyBoundary the facet-contact audit is skipped and §3's
	// pair-test ceiling is charged nothing, and below VerifyAll the per-cell
	// area slack and occupied-volume terms are never composed. Every other
	// audit, and the face bounds, run at each level.
	verify Verification
}

// tessellateRevolve meshes a revolved body (docs/tessellation-design.md §§8-10).
func tessellateRevolve(ctx context.Context, b *Body, rp revolvePayload, chord float64, verify Verification) (*Mesh, error) {
	plan, err := planRevolve(ctx, b, rp, chord, verify)
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
		if n > freeform.MaxChordsPerWalk {
			return freeform.ErrTooManyChords
		}
		p.nPhi = n
		p.deltaPhi = chordSagitta(p.rhoMax, p.sweep, n)
		return nil
	}
	w := p.resolved[r.loop].walks[r.walk]
	if !w.IsCircular() {
		return fmt.Errorf(`%w: a straight revolve generator carries no meridian chording to refine`, ErrUnsupported)
	}
	n := p.counts[r.loop][r.walk] + 1
	if n > freeform.MaxChordsPerWalk {
		return freeform.ErrTooManyChords
	}
	p.counts[r.loop][r.walk] = n
	p.sags[r.loop][r.walk] = chordSagitta(w.Radius, math.Abs(w.Th1-w.Th0), n)
	p.deltaM = 0
	for li := range p.sags {
		for _, s := range p.sags[li] {
			p.deltaM = math.Max(p.deltaM, s)
		}
	}
	return nil
}

// revolveResolution is one revolve payload's COUNT-INDEPENDENT resolution: the
// walks the body was built from, the coordinate envelope they span, and the two
// a-priori coordinate ceilings docs/tessellation-design.md §8 reserves out of
// the tolerance BEFORE any chord count exists.
//
// It is named apart from planRevolve because a second caller needs its last two
// fields without a tolerance to plan against: pairChordTolerance (boolean.go)
// must not hand a revolve operand a tolerance its own coordinate stages already
// spend, exactly as it must not hand a prism one under its section displacement
// (docs/tessellation-reach-design.md §6, R5).
type revolveResolution struct {
	basis       revolvemesh.RevolveBasis
	ideal       revolvemesh.RevolveBasis3Iv
	loops       []LoopRecord
	resolved    []revolveWalks
	junctions   [][]revolvemesh.RevMeridian
	rhoMax      float64
	coordMax    float64
	samplePrior float64
	deltaCPrior float64
	deltaRPrior float64
}

// resolveRevolve reads the payload's loops through the SAME resolution the
// builder used (revolveLoopWalks), then measures the count-independent
// coordinate ceilings §8 spends before the split.
func resolveRevolve(ctx context.Context, rp revolvePayload) (*revolveResolution, error) {
	ideal, ok := revolveIdealBasis(rp)
	if !ok {
		return nil, fmt.Errorf(`%w: this revolve's axis basis holds a coordinate that cannot be enclosed, so the mesh can state no construction bound`, ErrUnsupported)
	}

	// One resolution of every loop, shared with the builder (revolveLoopWalks),
	// so the mesh is read off the walks the body was built from.
	work := freeform.NewFreeformWork()
	loops := append([]LoopRecord{rp.profile.Outer}, rp.profile.Holes...)
	resolved := make([]revolveWalks, len(loops))
	junctions := make([][]revolvemesh.RevMeridian, len(loops))
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
	coordMax := revolvemesh.RevolveCoordMax(basis, zAbsMax, rhoMax)
	// A chorded meridian station is stored as the float nearest its own
	// certified enclosure, so its gap is bounded before any count exists; a
	// junction's gap is already count-independent and is measured outright.
	stationPrior := proofbound.ProductUpper(revolvemesh.RevolveStationRoundUlps, proofbound.UlpOf(math.Max(math.Max(zAbsMax, rhoMax), 1)))
	samplePrior := math.Max(junctionGap, stationPrior)
	if proofbound.IsNonFinite(coordMax) || proofbound.IsNonFinite(samplePrior) {
		return nil, fmt.Errorf(`%w: this revolve's coordinate envelope is not finite, so no chord budget can be reserved against it`, ErrUnsupported)
	}
	deltaCPrior := revolvemesh.RevolveConstructionPrior(basis, samplePrior, rhoMax, coordMax)
	deltaRPrior := proofbound.RigidRoundAllow(proofbound.AbsSumUpper(coordMax, deltaCPrior), proofbound.VecMaxAbs(rp.xform.Translation()))
	return &revolveResolution{
		basis: basis, ideal: ideal, loops: loops, resolved: resolved, junctions: junctions,
		rhoMax: rhoMax, coordMax: coordMax, samplePrior: samplePrior,
		deltaCPrior: deltaCPrior, deltaRPrior: deltaRPrior,
	}, nil
}

// planRevolve resolves the payload once and spends docs/tessellation-design.md
// §8's tolerance split in its stated order: both coordinate stages are reserved
// against count-independent ceilings, the meridian takes half of what is left
// and chords every circular walk, and the angular sequence takes the remainder.
func planRevolve(ctx context.Context, b *Body, rp revolvePayload, chord float64, verify Verification) (*revolvePlan, error) {
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

	res, err := resolveRevolve(ctx, rp)
	if err != nil {
		return nil, err
	}
	ideal, loops, resolved := res.ideal, res.loops, res.resolved
	basis, rhoMax, coordMax := res.basis, res.rhoMax, res.coordMax
	deltaCPrior, deltaRPrior := res.deltaCPrior, res.deltaRPrior
	available, err := revolvemesh.RevolveBudget(chord, deltaCPrior, deltaRPrior)
	if err != nil {
		return nil, err
	}

	// §8 steps 2-3: the meridian takes half the remaining budget and chords
	// every circular walk; deltaM is the largest sagitta those choices prove.
	meridian := freeform.DownRound(available / 2)
	counts := make([][]int, len(resolved))
	sags := make([][]float64, len(resolved))
	deltaM := 0.0
	for li, r := range resolved {
		counts[li] = make([]int, len(r.walks))
		sags[li] = make([]float64, len(r.walks))
		for k, w := range r.walks {
			counts[li][k] = 1
			if !w.IsCircular() {
				continue
			}
			n, sag, err := chordCount(w.SegmentWalk, meridian, revolveMeridianMin(w.SegmentWalk))
			if err != nil {
				return nil, err
			}
			counts[li][k], sags[li][k] = n, sag
			deltaM = math.Max(deltaM, sag)
		}
	}

	// §8 steps 4-5: the angular sequence takes what the meridian left.
	sweep := math.Abs(rp.phi1 - rp.phi0)
	angular := freeform.DownRound(available - deltaM)
	if angular <= 0 || proofbound.IsNonFinite(angular) {
		return nil, fmt.Errorf(`%w: this revolve's meridian chording spends the whole chord budget its tolerance left, so no angular count remains; retry with a coarser tolerance`, ErrUnsupported)
	}
	angularWalk := survey2d.SegmentWalk{Radius: rhoMax, Th1: sweep, Closed: rp.full}
	nPhi, deltaPhi, err := chordCount(angularWalk, angular, chordWalkMin(angularWalk))
	if err != nil {
		return nil, err
	}

	return &revolvePlan{
		body: b, rp: rp, basis: basis, ideal: ideal, loops: loops, resolved: resolved,
		junctions: res.junctions, faceOf: faceOf, work: &revolveWork{},
		counts: counts, sags: sags, nPhi: nPhi,
		deltaM: deltaM, deltaPhi: deltaPhi,
		deltaCPrior: deltaCPrior, deltaRPrior: deltaRPrior, samplePrior: res.samplePrior,
		rhoMax: rhoMax, coordMax: coordMax, sweep: sweep, chord: chord,
		verify: verify,
	}, nil
}

// revolveMeridianMin is docs/tessellation-design.md §9's meridian minimum:
// three chords for a whole closed generator, so it bounds a polygon; TWO for a
// circular generator whose two ends both sit on the axis — a sphere meridian —
// so it cannot chord to a single on-axis segment; and one otherwise.
func revolveMeridianMin(w survey2d.SegmentWalk) int {
	if w.Closed {
		return 3
	}
	if w.StartV == 0 && w.EndV == 0 {
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
	// sheet is docs/surface-design.md §4.1's own flag, read once: a surface
	// result omits both caps from a partial sweep's wall build
	// (revolve_build.go), and every arm below that would otherwise emit a
	// cap, charge a cap-only facet or a cap-only slack term, publish an
	// occupied-volume proof, or run the orientation check that decides
	// nothing on an open mesh reads it to skip that half of the work rather
	// than fault a role this evaluator's own reach never minted. A full
	// revolution mints no cap in either kind for a solid either, so most
	// arms below key on rp.full alone; sheet is its own separate condition
	// wherever a solid's full-turn behavior and a sheet's differ.
	sheet := rp.surfaceResult
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
	if err := requireLoopClearance(ctx, sectionPts, sectionLoops, revolveproof.LoopMaxSagitta(sectionSag)); err != nil {
		return nil, revolveSectionRetry(loopMesh, err)
	}
	if err := requireWalkClearance(ctx, sectionPts, sectionLoops, sectionSag); err != nil {
		return nil, revolveSectionRetry(loopMesh, err)
	}

	if err := revolvePreflightFacets(loopMesh, p.nPhi, rp.full, sheet, p.verify >= VerifyBoundary, p.work); err != nil {
		return nil, err
	}
	angular, err := revolveAngularSequence(rp, p.nPhi)
	if err != nil {
		return nil, err
	}

	// Rings share their meridian samples with the cell builder, so the
	// emitted vertex indices stay attached to the same samples.
	budget := proofbound.NewWorkBudget(ctx)
	samples := make([][]revolvemesh.RevMeridian, len(loopMesh))
	for li := range loopMesh {
		samples[li] = loopMesh[li].samples
	}
	vertices, deltaC, deltaR, err := revolvemesh.EmitRings(
		samples, angular, p.basis, p.ideal, rp.xform, rp.full, budget)
	if err != nil {
		return nil, err
	}
	mesh.vertices = vertices
	if deltaC > p.deltaCPrior || deltaR > p.deltaRPrior {
		return nil, fmt.Errorf(`%w: this revolve's stored coordinates sit farther from the samples they denote than the tolerance split reserved for them`, ErrUnsupported)
	}
	deltaR = math.Min(deltaR, proofbound.RigidRoundAllow(proofbound.AbsSumUpper(p.coordMax, deltaC), proofbound.VecMaxAbs(rp.xform.Translation())))
	coord := proofbound.AbsSumUpper(deltaC, deltaR)

	// Walls, cell by cell. Both rings off the axis give a planar quad on the
	// fixed diagonal; exactly one on the axis gives a fan; both on the axis is
	// a wall only an axis line may erase (docs/tessellation-design.md §9).
	// docs/tessellation-design.md §11's angular homotopy factor, proven ONCE:
	// rotating a cell about the axis is an isometry, so the same reading
	// answers for every cell and every angular interval of it. A sheet
	// publishes no occupied-volume proof at all (docs/surface-design.md §10),
	// so it skips this factor and the per-cell swept volume below rather than
	// prove a figure publishRevolveProof will never read. A level below
	// VerifyAll skips both for the same reason: it publishes neither the area
	// slack nor the occupied-volume bound, so nothing reads either term.
	proofs := p.verify >= VerifyAll
	var angularHomotopy *big.Rat
	if proofs && !sheet {
		var err error
		angularHomotopy, err = revolvemesh.RevolveAngularHomotopyFactor(angular.Step)
		if err != nil {
			return nil, err
		}
	}
	faceCells := map[*Face]revFaceExtent{}
	cellSlack := 0.0
	cellVolume := new(big.Rat)
	for li := range loopMesh {
		lm := loopMesh[li]
		n := len(lm.samples)
		for j := range lm.samples {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			lo, hi := lm.samples[j], lm.samples[(j+1)%n]
			k := lo.Walk
			if lm.resolved.kinds[k] == wallAxis {
				continue
			}
			if lo.OnAxis && hi.OnAxis {
				return nil, fmt.Errorf(`%w: a revolve generator with both ends on the axis sweeps no face, yet the recorded walk is not an axis line`, ErrUnsupported)
			}
			face, err := p.faceOf(fmt.Sprintf("side(%d,%d)", li, lm.resolved.walks[k].Segs[0]))
			if err != nil {
				return nil, err
			}
			emitRevolveCell(mesh, lo, hi, p.nPhi, face)
			cur := faceCells[face]
			cur.rho = math.Max(cur.rho, math.Max(lo.Rho, hi.Rho))
			cur.sag = math.Max(cur.sag, lo.Sag)
			faceCells[face] = cur
			if !proofs {
				continue
			}
			slack, err := revolveproof.CellSlack(p.ideal, angular, lo, hi, coord)
			if err != nil {
				return nil, err
			}
			cellSlack = proofbound.AbsSumUpper(cellSlack, proofbound.ProductUpper(float64(p.nPhi), slack))
			if !sheet {
				cellVolume.Add(cellVolume, revolvemesh.RevolveCellSweptVolume(lo, hi, angularHomotopy))
			}
		}
	}
	// Σ_cells Icell: one meridian cell's reading answers for each of the nPhi
	// angular intervals it spans.
	if proofs && !sheet {
		cellVolume.Mul(cellVolume, new(big.Rat).SetInt64(int64(p.nPhi)))
	}

	// Partial caps: one shared triangulation of the meridian region in the
	// (z, ρ) plane, mapped onto the wall vertices at φ0 and φ1. Pole vertices
	// are ordinary samples there, so an on-axis line's edge is shared by both.
	// Their curved trim omits one circular segment per meridian chord, per
	// cap. A surface-result sheet omits both caps (docs/surface-design.md
	// Table W) and so drops this whole term rather than triangulating one:
	// every per-cell wall term above survives, and this cap-only addition is
	// the only term a sheet's area slack drops — the mirror of the prism
	// sheet's own cap slack drop (tessellate.go). A full revolution mints no
	// cap in either kind: the walls already close.
	switch {
	case rp.full:
		// no cap in either kind.
	case sheet:
		// emitRevolveCaps' own triangulation.Triangulate call is what would
		// otherwise refuse a loop chording to fewer than three meridian
		// samples ("a cap needs at least three boundary samples",
		// triangulate.go); a sheet mints no cap to carry that refusal, so it
		// is restated here directly. requireWalkClearance's own `m < 4`
		// branch (tessellate.go) SKIPS, rather than refuses, a small loop,
		// so it is not a substitute for this check.
		if len(sectionLoops) == 0 || len(sectionLoops[0]) < 3 {
			return nil, fmt.Errorf(`%w: a surface-result revolve wall loop needs at least three meridian samples`, ErrDegenerate)
		}
	default:
		if err := emitRevolveCaps(ctx, mesh, loopMesh, sectionPts, sectionLoops, angular.Samples-1, p.faceOf); err != nil {
			return nil, err
		}
		if proofs {
			cellSlack = proofbound.AbsSumUpper(cellSlack, proofbound.ProductUpper(2, revolveCapSegmentArea(p)))
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

	// A BodySolid keeps the closed-mesh audit plus its own pole-vertex safety
	// net (requireVertexLinks, §9); a BodySheet runs
	// docs/tessellation-design.md §1.2's manifold-with-boundary audit plus
	// its own cycle-or-path vertex-link safety net in their place
	// (requireMeshAudit, tessellate.go — the same dispatch the prism sheet
	// path already reaches). A full revolution's sheet carries no free edge
	// at all, so requireSheetMesh agrees with tessellation.RequireClosedMesh with no arm
	// of its own (docs/surface-design.md §14).
	if sheet {
		if err := requireMeshAudit(ctx, sheet, p.body, mesh); err != nil {
			return nil, err
		}
	} else {
		if err := tessellation.RequireClosedMesh(mesh.triangles); err != nil {
			return nil, fmt.Errorf(`%w: this revolve's cells do not close into a watertight boundary`, ErrUnsupported)
		}
		if err := requireVertexLinks(ctx, mesh); err != nil {
			return nil, err
		}
	}
	// Every facet keeps a positive area under the displacement this mesh
	// carries, at every verification level (docs/tessellation-design.md §1's
	// Geometry row, §9's own paragraph): a zero-area facet has no normal, so a
	// renderer and the boolean both need the check, and it is linear in the
	// facets. It also builds the audit triangles the facet-pair audit below
	// consumes, so the two share one pass and one work budget.
	auditBudget := proofbound.NewWorkBudget(ctx)
	auditTris, err := revolvemesh.RequireRevolveFacetAreas(auditBudget, mesh.vertices, mesh.triangles, coord)
	if err != nil {
		return nil, err
	}
	if p.verify >= VerifyBoundary {
		if err := revolvemesh.RevolveContactAudit(auditBudget, auditTris, mesh.triangles, coord); err != nil {
			// A crossing or an undecided contact is the one failure a finer
			// angular sequence can still answer, and §3 makes the global count the
			// thing an angular failure increments. A canceled context or an
			// exhausted work budget is not that failure and is returned unchanged.
			if !errors.Is(err, ErrUnsupported) {
				return nil, err
			}
			return nil, &revolveRefineError{err: err, retry: revolveRefine{loop: -1}}
		}
	}
	// The signed-volume orientation check decides nothing on an open mesh:
	// the sum is anchor-dependent when nothing pins where the missing
	// material would have been (docs/tessellation-design.md §1.2). A
	// partial-sweep sheet is open and skips it; a solid, or a full-turn
	// sheet — which carries no free edge at all — is closed and keeps it.
	if !sheet || rp.full {
		anchor := rp.xform.Apply(p.basis.A3)
		if !proofbound.FiniteVec(anchor) || tessellation.OrientationSign(mesh.vertices, mesh.triangles, anchor) <= 0 {
			return nil, fmt.Errorf(`%w: this revolve's assembled cells do not enclose a positive volume`, ErrUnsupported)
		}
	}

	if err := publishRevolveProof(mesh, faceCells, p, deltaC, deltaR, cellSlack, cellVolume); err != nil {
		return nil, err
	}
	return mesh, nil
}

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
func revolveJunctions(rp revolvePayload, r revolveWalks) ([]revolvemesh.RevMeridian, float64, error) {
	out := make([]revolvemesh.RevMeridian, len(r.walks))
	worst := 0.0
	for k, w := range r.walks {
		plane := r.plane[w.Segs[0]]
		zIv, rhoIv, ok := revolvemesh.RevolveMeridianEnclosure(rp.ax.aU, rp.ax.aV, rp.ax.dU, rp.ax.dV, plane.StartU, plane.StartV, plane.StartBound)
		if !ok {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian sample states no enclosure of the axis coordinates its record denotes`, ErrUnsupported)
		}
		gap := math.Max(proofbound.IntervalFloatError(zIv, w.StartU), proofbound.IntervalFloatError(rhoIv, w.StartV))
		if proofbound.IsNonFinite(gap) {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian sample states no bound on its own axis coordinates`, ErrUnsupported)
		}
		worst = math.Max(worst, gap)
		if w.StartV < 0 {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian sample sits on the negative side of the axis`, ErrDegenerate)
		}
		out[k] = revolvemesh.RevMeridian{Z: w.StartU, Rho: w.StartV, ZIv: zIv, RhoIv: rhoIv, OnAxis: w.StartV == 0, Walk: k}
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
func revolveMeridianSamples(rp revolvePayload, loop LoopRecord, r revolveWalks, junctions []revolvemesh.RevMeridian, counts []int, sags []float64) ([]revolvemesh.RevMeridian, float64, error) {
	out := make([]revolvemesh.RevMeridian, 0, len(junctions))
	worst := 0.0
	for k, w := range r.walks {
		n := counts[k]
		if n <= 0 {
			return nil, 0, fmt.Errorf(`%w: a revolve meridian walk carries no chord`, ErrUnsupported)
		}
		start := junctions[k]
		start.Sag, start.Walk = sags[k], k
		if !w.IsCircular() {
			out = append(out, start)
			continue
		}
		cell, ok := revolvemesh.RevolveArcChordCell(w.SegmentWalk, 0, n)
		if !ok {
			return nil, 0, revolvemesh.ErrRevolveArcCellSlack
		}
		start.Arc = cell
		out = append(out, start)
		for i := 1; i < n; i++ {
			station, gap, err := revolveArcStation(rp.ax, loop.Segments[w.Segs[0]], i, n)
			if err != nil {
				return nil, 0, err
			}
			cell, ok := revolvemesh.RevolveArcChordCell(w.SegmentWalk, i, n)
			if !ok {
				return nil, 0, revolvemesh.ErrRevolveArcCellSlack
			}
			station.Walk, station.Sag, station.Arc = k, sags[k], cell
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
			if !s.OnAxis {
				offAxis[s.Walk] = true
			}
		}
		for k, w := range lm.resolved.walks {
			if !w.IsCircular() || lm.resolved.kinds[k] == wallAxis || offAxis[k] {
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
			k := lm.samples[j].Walk
			if lm.resolved.walks[k].IsCircular() {
				return &revolveRefineError{err: err, retry: revolveRefine{loop: named.loop, walk: k}}
			}
		}
	}
	for li, lm := range loops {
		for k, w := range lm.resolved.walks {
			if w.IsCircular() {
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
func requireRevolveAxisIncidence(resolved []revolveWalks, junctions [][]revolvemesh.RevMeridian) error {
	seen := map[float64]struct{}{}
	for li, js := range junctions {
		n := len(js)
		for k, s := range js {
			if !s.OnAxis {
				continue
			}
			if _, dup := seen[s.Z]; dup {
				return fmt.Errorf(`%w: two recorded boundary junctions meet the revolve axis at the same point, so the swept solid pinches there`, ErrDegenerate)
			}
			seen[s.Z] = struct{}{}
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

// revolveExtents adapts the resolved meridian walks to the envelope reader.
func revolveExtents(loops []revolveWalks) (float64, float64, error) {
	return revolveproof.Extents(revolveWalkView(loops))
}

type revolveWalkView []revolveWalks

func (loops revolveWalkView) Len() int                        { return len(loops) }
func (loops revolveWalkView) Walks(i int) []survey2d.SideWalk { return loops[i].walks }

// revolveCapSegmentArea is the circular-segment area ONE partial cap's curved
// trim omits (docs/tessellation-design.md §10.2): the chorded meridian region
// differs from the region it denotes by exactly the segments between each
// circular walk's arc and its chords, summed in ABSOLUTE value because a hole
// gains what an outline loses and area slack admits no cancellation.
func revolveCapSegmentArea(p *revolvePlan) float64 {
	total := 0.0
	for li, r := range p.resolved {
		for k, w := range r.walks {
			if !w.IsCircular() {
				continue
			}
			total = proofbound.AbsSumUpper(total, revolvemesh.ChordSegmentArea(w.Radius, math.Abs(w.Th1-w.Th0), p.counts[li][k]))
		}
	}
	return total
}

// revolvePreflightFacets adapts the builder's loops to the facet budget.
func revolvePreflightFacets(loops []revLoopMesh, nPhi int, full, sheet, audit bool, work *revolveWork) error {
	state := revolveproof.FacetWork{Facets: work.facets, Pairs: work.pairs}
	if err := revolveproof.PreflightFacets(revolveFacetLoops(loops), nPhi, full, sheet, audit, &state); err != nil {
		return err
	}
	work.facets, work.pairs = state.Facets, state.Pairs
	return nil
}

type revolveFacetLoops []revLoopMesh

func (loops revolveFacetLoops) Len() int                                { return len(loops) }
func (loops revolveFacetLoops) Samples(i int) []revolvemesh.RevMeridian { return loops[i].samples }
func (loops revolveFacetLoops) AxisWalk(i, walk int) bool {
	return loops[i].resolved.kinds[walk] == wallAxis
}

func addChecked(a, b uint64) (uint64, bool) { return revolveproof.AddChecked(a, b) }
func mulChecked(a, b uint64) (uint64, bool) { return revolveproof.MulChecked(a, b) }

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
func emitRevolveCell(m *Mesh, lo, hi revolvemesh.RevMeridian, nPhi int, face *Face) {
	for l := range nPhi {
		a, d := lo.At(l), lo.At(l+1)
		bb, c := hi.At(l), hi.At(l+1)
		switch {
		case lo.OnAxis:
			m.addTriangle([3]int{a, bb, c}, face)
		case hi.OnAxis:
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
			startV = append(startV, s.At(0))
			endV = append(endV, s.At(last))
		}
	}
	tris, err := triangulation.Triangulate(ctx, pts, loopIdx)
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
			pts = append(pts, Point2{U: s.Z, V: s.Rho})
			idx[k] = base + k
			sag[k] = s.Sag
		}
		loopIdx = append(loopIdx, idx)
		loopSag = append(loopSag, sag)
	}
	return pts, loopIdx, loopSag
}

// publishRevolveProof writes docs/tessellation-design.md §2's proof record for
// the assembled mesh: §10.1's per-face two-sided displacement, §10.2's area
// slack, and §11's occupied-volume bound.
//
// A wall face carries its own meridian and angular displacement, each computed
// at the largest figure its own cells reach rather than at the mesh's, plus
// both coordinate stages; a partial cap carries the mesh's meridian
// displacement and the coordinate stages, since a cap's own trim is exact
// wherever the meridian is straight. volSymDiff composes §11's four stages
// (tessellate_revolve_volume.go) rather than bound × held area, which §11
// forbids outright, and only that composition sets symDiffOK.
//
// A [BodySheet] encloses no region, so there is no occupied volume to prove
// (docs/surface-design.md §10): it publishes the face bounds and area slack
// above, unchanged, and then returns before touching volSymDiff or
// symDiffOK at all, leaving both their permanent zero/false values — the
// same permanent absence the prism sheet path documents (tessellate.go).
// cellVolume stays whatever buildRevolveMesh left it at, the zero *big.Rat
// it never added a term to, and is never read.
func publishRevolveProof(m *Mesh, faceCells map[*Face]revFaceExtent, p *revolvePlan, deltaC, deltaR, cellSlack float64, cellVolume *big.Rat) error {
	coord := proofbound.AbsSumUpper(deltaC, deltaR)
	if proofbound.UpRound(proofbound.AbsSumUpper(p.deltaM, p.deltaPhi, coord)) > p.chord {
		// The chording component must stay inside the requested tolerance, and
		// revolvemesh.RevolveBudget already reserved both coordinate stages out of it
		// before the counts were chosen. Reaching here would mean the
		// reservation did not hold, which is a proof failure rather than a
		// coarser mesh (docs/tessellation-design.md §8).
		return fmt.Errorf(`%w: this revolve mesh's chording exceeds the tolerance its own budget reserved for it`, ErrUnsupported)
	}
	for f, ext := range faceCells {
		bound := proofbound.UpRound(proofbound.AbsSumUpper(ext.sag, chordSagitta(ext.rho, p.sweep, p.nPhi), coord))
		if proofbound.IsNonFinite(bound) {
			return fmt.Errorf(`%w: a revolve wall face's composed displacement is not finite`, ErrUnsupported)
		}
		m.setFaceBound(f, bound)
	}
	capBound := proofbound.UpRound(proofbound.AbsSumUpper(p.deltaM, coord))
	for _, f := range m.source {
		if _, ok := m.faceBound[f]; !ok {
			m.setFaceBound(f, capBound)
		}
	}
	if proofbound.IsNonFinite(m.bound) {
		return fmt.Errorf(`%w: this revolve mesh states no finite displacement bound`, ErrUnsupported)
	}
	if p.verify < VerifyAll {
		// The face bounds above are complete and published; §10.2's area slack
		// and §11's occupied-volume bound were never composed, so neither is
		// stated (tessellateContext's withholdProofs drops the zeroes they
		// would otherwise leave behind).
		return nil
	}
	slack := proofbound.AbsSumUpper(cellSlack, meshCoordAreaAllow(m, coord))
	if proofbound.IsNonFinite(slack) {
		return fmt.Errorf(`%w: this revolve mesh states no finite area slack`, ErrUnsupported)
	}
	m.areaSlack = slack
	if p.rp.surfaceResult {
		return nil
	}
	sym, err := revolveSymDiff(m, p, cellVolume, deltaC, deltaR)
	if err != nil {
		return err
	}
	m.volSymDiff = sym
	m.symDiffOK = true
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
		total = proofbound.AbsSumUpper(total, proofbound.PerturbedTriangleAreaAllow(a, b, c, delta))
	}
	return total
}

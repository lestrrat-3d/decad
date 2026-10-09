package decad

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/revolvesampling"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
)

// This file is docs/tessellation-design.md §13's increments T2 and T3
// (docs/tessellation-reach-design.md §6, R3 and R4): revolve tessellation. It
// assembles the meridian samples, global angular sequence, rings, cells, poles
// and caps. internal/revolveproof computes envelopes, budgets, cell area slack
// and volume bounds. internal/revolvemesh/revolve_proof.go checks the angular
// sequence and axis basis. internal/tessellation audits vertex links.
// internal/revolvesampling forms certified meridian junctions and stations.
//
// A free-form (Tier A NURBS) revolve generator is still refused, by
// revolveaxis.ResolveLoop's own boundarywalk.RequireAnalyticWalk: those cells are §13's increment
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
	// section is the payload's own section-displacement charge
	// (revolveSectionChargeOf): the mesh is chorded from the RECORDED meridian,
	// so every face bound, the area slack and the occupied-volume bound carry
	// the distance from it to the meridian the record denotes. Zero for every
	// payload no construction displaced.
	section revolveSectionCharge
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
	w := p.resolved[r.loop].Walks[r.walk]
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
// builder used (revolveaxis.ResolveLoop), then measures the count-independent
// coordinate ceilings §8 spends before the split.
func resolveRevolve(ctx context.Context, rp revolvePayload) (*revolveResolution, error) {
	ideal, ok := revolvemesh.IdealBasis(rp.frame, rp.ax.aU, rp.ax.aV, rp.ax.dU, rp.ax.dV)
	if !ok {
		return nil, fmt.Errorf(`%w: this revolve's axis basis holds a coordinate that cannot be enclosed, so the mesh can state no construction bound`, ErrUnsupported)
	}

	// One resolution of every loop, shared with the builder (revolveaxis.ResolveLoop),
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
		r, err := revolveaxis.ResolveLoop(ctx, loop, work, "revolve tessellation",
			rp.chargedWalk, rp.ax.snapTol)
		if err != nil {
			return nil, err
		}
		js, gap, err := revolvesampling.MeridianJunctions(rp.lift(), r)
		if err != nil {
			return nil, err
		}
		junctionGap = math.Max(junctionGap, gap)
		resolved[li], junctions[li] = r, js
	}
	if err := revolvesampling.RequireAxisIncidence(resolved, junctions); err != nil {
		return nil, err
	}

	rhoMax, zAbsMax, err := revolveproof.Extents(revolveWalkView(resolved))
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
	// The section displacement is reserved out of the tolerance before the two
	// coordinate stages, on the prism tessellator's own terms (tessellate.go):
	// the mesh chords the recorded meridian, and the meridian it denotes sits
	// up to sectionDelta from it. Both downward nudges are proven margin for
	// the subtraction and for the upward-rounded sum the bound is published
	// through.
	budgetChord := chord
	section, err := revolveSectionChargeOf(rp, freeform.NewFreeformWork())
	if err != nil {
		return nil, err
	}
	if rp.sectionDelta > 0 {
		budgetChord = freeform.DownRound(freeform.DownRound(chord - rp.sectionDelta))
		if budgetChord <= 0 {
			requested, displacement := units.Millimeters(chord), units.Millimeters(rp.sectionDelta)
			return nil, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s; retry with a tolerance greater than %s`, ErrUnsupported, requested, displacement, displacement)
		}
	}
	available, err := revolvemesh.RevolveBudget(budgetChord, deltaCPrior, deltaRPrior)
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
		counts[li] = make([]int, len(r.Walks))
		sags[li] = make([]float64, len(r.Walks))
		for k, w := range r.Walks {
			counts[li][k] = 1
			if !w.IsCircular() {
				continue
			}
			n, sag, err := chordCount(w.SegmentWalk, meridian, revolvesampling.MeridianMin(w.SegmentWalk))
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
		section: section, verify: verify,
	}, nil
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
	meridianSamples := make([][]revolvemesh.RevMeridian, len(p.resolved))
	sampleGap := 0.0
	for li := range p.resolved {
		samples, gap, err := revolvesampling.MeridianSamples(
			rp.lift(), p.loops[li], p.resolved[li], p.junctions[li], p.counts[li], p.sags[li])
		if err != nil {
			return nil, err
		}
		sampleGap = math.Max(sampleGap, gap)
		loopMesh[li] = revLoopMesh{resolved: p.resolved[li], samples: samples}
		meridianSamples[li] = samples
	}
	if sampleGap > p.samplePrior {
		return nil, fmt.Errorf(`%w: a revolve meridian sample sits farther from the axis coordinates its record denotes than the tolerance split reserved for it`, ErrUnsupported)
	}
	if li, k, erased := revolvesampling.OffAxisWalk(p.resolved, meridianSamples); erased {
		return nil, &revolveRefineError{
			err:   fmt.Errorf(`%w: a circular revolve generator chords to a polyline lying entirely on the axis, which sweeps no face`, ErrUnsupported),
			retry: revolveRefine{loop: li, walk: k},
		}
	}

	// docs/tessellation-design.md §9's meridian section proof, run for a full
	// and a partial sweep alike and BEFORE any three-dimensional cell is
	// formed: every loop simple and correctly nested, and every non-adjacent
	// chord pair — across loops and WITHIN one loop — clear of the two sagitta
	// tubes the analytic-to-chord homotopy moves inside.
	sectionPts, sectionLoops, sectionSag := revolvesampling.SectionPoints(meridianSamples)
	if err := requireLoopClearance(ctx, sectionPts, sectionLoops, revolveproof.LoopMaxSagitta(sectionSag)); err != nil {
		return nil, revolveSectionRetry(loopMesh, err)
	}
	if err := requireWalkClearance(ctx, sectionPts, sectionLoops, sectionSag); err != nil {
		return nil, revolveSectionRetry(loopMesh, err)
	}

	if err := revolvePreflightFacets(loopMesh, p.nPhi, rp.full, sheet, p.verify >= VerifyBoundary, p.work); err != nil {
		return nil, err
	}
	angular, err := revolvemesh.AngularSequence(revolvemesh.AngularInput{
		Phi0: rp.phi0, Phi1: rp.phi1, Full: rp.full, Den: rp.den,
	}, p.nPhi)
	if err != nil {
		return nil, err
	}

	// Rings share their meridian samples with the cell builder, so the
	// emitted vertex indices stay attached to the same samples.
	budget := proofbound.NewWorkBudget(ctx)
	vertices, deltaC, deltaR, err := revolvemesh.EmitRings(
		meridianSamples, angular, p.basis, p.ideal, rp.xform, rp.full, budget)
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
			if lm.resolved.Kinds[k] == wallAxis {
				continue
			}
			if lo.OnAxis && hi.OnAxis {
				return nil, fmt.Errorf(`%w: a revolve generator with both ends on the axis sweeps no face, yet the recorded walk is not an axis line`, ErrUnsupported)
			}
			face, err := p.faceOf(fmt.Sprintf("side(%d,%d)", li, lm.resolved.Walks[k].Segs[0]))
			if err != nil {
				return nil, err
			}
			revolvemesh.EmitCellTriangles(lo, hi, p.nPhi, func(tri [3]int) {
				mesh.addTriangle(tri, face)
			})
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
		// revolvemesh.EmitCapTriangles' triangulation call is what would
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
		capStart, err := p.faceOf(roleCapStart)
		if err != nil {
			return nil, err
		}
		capEnd, err := p.faceOf(roleCapEnd)
		if err != nil {
			return nil, err
		}
		if err := revolvemesh.EmitCapTriangles(ctx, meridianSamples, sectionPts, sectionLoops,
			angular.Samples-1, func(end, start [3]int) {
				mesh.addTriangle(end, capEnd)
				mesh.addTriangle(start, capStart)
			}); err != nil {
			return nil, err
		}
		if proofs {
			cellSlack = proofbound.AbsSumUpper(cellSlack,
				proofbound.ProductUpper(2, revolvesampling.CapSegmentArea(p.resolved, p.counts)))
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
	// net (tessellation.RequireVertexLinks, §9); a BodySheet runs
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
		if err := tessellation.RequireVertexLinks(ctx, len(mesh.vertices), mesh.triangles); err != nil {
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
			if lm.resolved.Walks[k].IsCircular() {
				return &revolveRefineError{err: err, retry: revolveRefine{loop: named.loop, walk: k}}
			}
		}
	}
	for li, lm := range loops {
		for k, w := range lm.resolved.Walks {
			if w.IsCircular() {
				return &revolveRefineError{err: err, retry: revolveRefine{loop: li, walk: k}}
			}
		}
	}
	return err
}

type revolveWalkView []revolveWalks

func (loops revolveWalkView) Len() int                        { return len(loops) }
func (loops revolveWalkView) Walks(i int) []survey2d.SideWalk { return loops[i].Walks }

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
	return loops[i].resolved.Kinds[walk] == wallAxis
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
	// moved is the section displacement every face reads beside its chording
	// and coordinate stages (revolvePlan.section); zero folds nothing.
	moved := func(bound float64) float64 {
		if p.rp.sectionDelta > 0 {
			return proofbound.AbsSumUpper(bound, p.rp.sectionDelta)
		}
		return bound
	}
	if proofbound.UpRound(moved(proofbound.AbsSumUpper(p.deltaM, p.deltaPhi, coord))) > p.chord {
		// The chording component must stay inside the requested tolerance, and
		// revolvemesh.RevolveBudget already reserved both coordinate stages out of it
		// before the counts were chosen. Reaching here would mean the
		// reservation did not hold, which is a proof failure rather than a
		// coarser mesh (docs/tessellation-design.md §8).
		return fmt.Errorf(`%w: this revolve mesh's chording exceeds the tolerance its own budget reserved for it`, ErrUnsupported)
	}
	for f, ext := range faceCells {
		bound := proofbound.UpRound(moved(proofbound.AbsSumUpper(ext.sag, chordSagitta(ext.rho, p.sweep, p.nPhi), coord)))
		if proofbound.IsNonFinite(bound) {
			return fmt.Errorf(`%w: a revolve wall face's composed displacement is not finite`, ErrUnsupported)
		}
		m.setFaceBound(f, bound)
	}
	capBound := proofbound.UpRound(moved(proofbound.AbsSumUpper(p.deltaM, coord)))
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
	slack := proofbound.AbsSumUpper(cellSlack, revolvemesh.CoordinateAreaAllow(m.vertices, m.triangles, coord))
	if allow := revolveaxis.SectionAreaAllow(p.rp.sectionDelta, p.sweep, p.rp.sweep().Bound,
		p.resolved, !p.rp.full && !p.rp.surfaceResult, p.section); allow > 0 {
		slack = proofbound.AbsSumUpper(slack, allow)
	}
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
	if allow := revolveaxis.SectionVolumeAllow(p.rp.sectionDelta, p.sweep, p.rp.sweep().Bound, p.section); allow > 0 {
		sym = proofbound.AbsSumUpper(sym, allow)
	}
	m.volSymDiff = sym
	m.symDiffOK = true
	return nil
}

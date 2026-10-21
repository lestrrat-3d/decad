package verifyrun

import (
	"context"
	"fmt"
	"math"
	"sync"

	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	vocab "github.com/lestrrat-3d/decad/report"

	"github.com/lestrrat-3d/decad/internal/boundaryaudit"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/clearance/pairkernel"
	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// This file is Verify's pair partition (agent-docs/interference-design.md §2):
// the per-pair proof walk. Every pair's proof reads only its two operands
// and caches each body fills the same way whoever asks first, so a pair's
// outcome does not depend on which other pairs ran before it or beside it.
// Verify runs the jobs through internal/orderedwork and folds outcomes in
// pair order.
// verifyPairJob is one unordered pair, in Document.Bodies() order, that box
// separation alone does not finish.
type verifyPairJob struct {
	a, b      *topo.Body
	boxProven bool
}

// verifyPairOutcome is what one pair adds to the report, in the order the
// pair walk appends it.
type verifyPairOutcome struct {
	interferences []Interference
	clearances    []Clearance
	diagnostics   []Diagnostic
	undecided     bool
}

// verifyPairJobs lists the pairs of proven-valid bodies that need more than
// box separation, in i < j order. A pair it leaves out adds nothing to the
// report: separated boxes settle it and no gap was asked for, or both
// operands are sheets with separated boxes. The context is polled once per
// pair, as the pair walk always has.
func verifyPairJobs(ctx context.Context, bodies []*BodyReport, cfg verifyConfig) ([]verifyPairJob, error) {
	var jobs []verifyPairJob
	for i := range bodies {
		for j := i + 1; j < len(bodies); j++ {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			a, b := bodies[i].Body, bodies[j].Body
			boxProven := measurement.BoxesDisjoint(bodies[i].Bounds.Box, bodies[j].Bounds.Box)
			if boxProven {
				sheetPair := a.Kind == topo.BodySheet && b.Kind == topo.BodySheet
				if sheetPair || !cfg.Clearances {
					continue
				}
			}
			jobs = append(jobs, verifyPairJob{a: a, b: b, boxProven: boxProven})
		}
	}
	return jobs, nil
}

// proveVerifyPair runs agent-docs/interference-design.md §2's walk over one pair:
// a pair holding a sheet operand takes agent-docs/surface-design.md §9.3's
// procedure; a solid pair takes the analytic pair kernel, then containment,
// equality and read-only intersection measurement. geomCache is nil unless
// clearances were asked; it and meshes must be safe for concurrent use.
func proveVerifyPair(ctx context.Context, job verifyPairJob, cfg verifyConfig, geomCache *pairkernel.Cache, meshes booleaneval.OperandMesher, mesher verifiedmesh.Mesher, alloc topo.Alloc) (verifyPairOutcome, error) {
	var out verifyPairOutcome
	a, b, boxProven := job.a, job.b, job.boxProven

	// A sheet operand encloses no region, so the interference relation §1
	// decides is not the question for this pair (agent-docs/surface-design.md
	// §9.3). A sheet-sheet pair offers no closed boundary to cast against, so
	// it takes only box separation: separated boxes contribute nothing (the
	// job list already dropped them), and boxes that meet stay
	// DiagUnsupportedPairSheet regardless of WithClearances(). A
	// sheet-against-solid pair is decided by pairkernel.SheetSolid
	// into a proven crossing, a proven containment or separation with a
	// measured gap, or the undecided code when the kernel cannot settle it —
	// run whenever the boxes meet (crossing must always be checked, asked or
	// not), and also when the boxes are separated but a gap was requested,
	// exactly as the solid-solid path below runs pairkernel.Pair in that same
	// second case.
	if a.Kind == topo.BodySheet || b.Kind == topo.BodySheet {
		if a.Kind == topo.BodySheet && b.Kind == topo.BodySheet {
			out.diagnostics = append(out.diagnostics,
				PairDiagNone(a, b, vocab.DiagUnsupportedPairSheet,
					"both operands are sheet bodies, and neither offers a closed boundary to cast the other against"))
			out.undecided = true
			return out, nil
		}
		sheet, solid := a, b
		if b.Kind == topo.BodySheet {
			sheet, solid = b, a
		}
		sres, err := pairkernel.SheetSolid(ctx, sheet, solid, boxProven, geomCache)
		if err != nil {
			return verifyPairOutcome{}, err
		}
		switch sres.Verdict {
		case boundaryaudit.SheetSolidCrossing:
			out.diagnostics = append(out.diagnostics, Diagnostic{
				Code:    vocab.DiagSheetSolidCrossing,
				Status:  vocab.Interfering,
				Pair:    &DiagnosticPair{A: a, B: b},
				Reading: vocab.ReadingNone,
				Message: "the sheet crosses the solid's boundary; no Interference row is emitted because a sheet encloses no region and there is no overlap volume to report",
			})
		case boundaryaudit.SheetSolidContained, boundaryaudit.SheetSolidOutside:
			if cfg.Clearances {
				pr := pairkernel.Result{Lo: sres.Lo, Hi: sres.Hi, Exact: sres.Exact, Diam: sres.Diam}
				out.appendClearance(a, b, pr, cfg.Rel)
			}
		case boundaryaudit.SheetSolidUndecided:
			if boxProven {
				// Box separation already proves the sheet lies outside the
				// solid; only the requested gap itself is unmeasured, the
				// same shape of gap as an unmeasured solid-solid clearance
				// below.
				out.diagnostics = append(out.diagnostics,
					PairDiagNone(a, b, vocab.DiagUndecidedClearance,
						"the pair is proven disjoint but the requested clearance gap is unmeasured"))
			} else {
				out.diagnostics = append(out.diagnostics,
					PairDiagNone(a, b, vocab.DiagUnsupportedPairSheet,
						"the clearance kernel could not settle this sheet-against-solid pair"))
			}
			out.undecided = true
		}
		return out, nil
	}

	res, fast := pairkernel.AxisBoxes(a, b)
	if !fast {
		var err error
		res, err = pairkernel.Pair(ctx, a, b, boxProven, geomCache)
		if err != nil {
			return verifyPairOutcome{}, err
		}
	}
	if res.Verdict == boundaryaudit.PairUndecided {
		// The exact planar relation (internal/verifyrun/clearance.go) answers what the
		// analytic kernel left open — a pair an operand has no carrier model
		// in, or one its enumeration could not settle — and only ever adds a
		// verdict, never replaces one (interference design §3.2).
		planar, ok, err := planarPairVerdict(ctx, mesher, a, b)
		if err != nil {
			return verifyPairOutcome{}, err
		}
		if ok {
			res = planar
		}
	}
	if err := ctx.Err(); err != nil {
		return verifyPairOutcome{}, err
	}

	// Box separation already proves the partition. The analytic kernel runs
	// only to supply an asked gap; failure to measure that gap is Suspect
	// (DiagUndecidedClearance) but never sends a proven-disjoint pair to
	// intersection.
	if boxProven {
		if res.Verdict == boundaryaudit.PairDisjoint || res.Verdict == boundaryaudit.PairTouching {
			out.appendClearance(a, b, res, cfg.Rel)
			return out, nil
		}
		out.diagnostics = append(out.diagnostics,
			PairDiagNone(a, b, vocab.DiagUndecidedClearance,
				"the pair is proven disjoint but the requested clearance gap is unmeasured"))
		out.undecided = true
		return out, nil
	}

	if res.Verdict == boundaryaudit.PairDisjoint || res.Verdict == boundaryaudit.PairTouching {
		if cfg.Clearances {
			out.appendClearance(a, b, res, cfg.Rel)
		}
		return out, nil
	}

	volume, outcome, err := MeasuredInterference(ctx, alloc, a, b, res, meshes)
	if err != nil {
		return verifyPairOutcome{}, err
	}
	if outcome != boundaryaudit.InterferenceMeasured {
		out.diagnostics = append(out.diagnostics, UndecidedPairDiag(a, b, a, b, res.Verdict, outcome))
		out.undecided = true
		return out, nil
	}
	out.interferences = append(out.interferences, Interference{A: a, B: b, Volume: volume})
	pairD, err := pairkernel.PairGateDiameter(ctx, a, b, nil)
	if err != nil {
		return verifyPairOutcome{}, err
	}
	obs := volume
	out.diagnostics = append(out.diagnostics, Diagnostic{
		Code:     vocab.DiagInterference,
		Status:   vocab.Interfering,
		Pair:     &DiagnosticPair{A: a, B: b},
		Reading:  vocab.ReadingOverlapVolume,
		Observed: &obs,
		Message:  "the pair is proven to overlap",
	})
	pass, ref, haveRef := InterferenceToleranceRef(volume, a, b, pairD, cfg.Rel)
	if !pass {
		beyond := Diagnostic{
			Code:     vocab.DiagMeasurementBeyondTolerance,
			Status:   vocab.Suspect,
			Pair:     &DiagnosticPair{A: a, B: b},
			Reading:  vocab.ReadingOverlapVolume,
			Observed: &obs,
			Message:  fmt.Sprintf("the overlap-volume reading's bound %s is beyond the relative tolerance", volume.Bound),
		}
		if haveRef {
			beyond.Required = tolerance.RequiredThreshold(cfg.Rel*ref, volume.Value)
		}
		out.diagnostics = append(out.diagnostics, beyond)
		out.undecided = true
	}
	return out, nil
}

// appendClearance records the pair's Clearance row and, when the gap misses
// the tolerance gate, its diagnostic (verify.go's appendClearance).
func (o *verifyPairOutcome) appendClearance(a, b *topo.Body, res pairkernel.Result, rel float64) {
	scratch := Report{}
	if d := appendClearance(&scratch, a, b, res, rel); d != nil {
		o.diagnostics = append(o.diagnostics, *d)
		o.undecided = true
	}
	o.clearances = append(o.clearances, scratch.Clearances...)
}

// verifyMeshCache meshes each body once per Verify call, at one chord for
// every pair it takes part in (agent-docs/interference-design.md §5.3). The chord is
// the tightest one any of the body's candidate pairs would have asked for:
// the least booloperand.PairChord over the solid pairs whose boxes meet, which
// are the only pairs that can reach the mesh path. The entries are fixed
// before the worker pool starts and only read after it, and each entry's mesh
// is built under its own sync.Once, so one Verify call meshes and audits a
// body once however many workers ask for it at the same time.
//
// A body whose tessellation restates a held mesh (a faceted Boolean result,
// a mitred sweep; booloperand.HeldFloor) chords nothing, so there is nothing to share:
// it has no entry, and every pair meshes it as the public booleans do, at the
// pair's own tolerance raised to its held floor, and gates the facets that
// pair touches at the pair's own tolerance (agent-docs/api-design.md §8 "The chain
// depth", agent-docs/faceted-vertex-bounds-design.md §5).
type verifyMeshCache struct {
	entries map[*topo.Body]*verifyMeshEntry
	mesher  verifiedmesh.Mesher
	operand booleaneval.OperandMesher
}

// verifyMeshEntry is one body's shared mesh: the chord it is meshed at, and
// the outcome of the one tessellation that builds it.
type verifyMeshEntry struct {
	chord float64
	once  sync.Once
	mesh  *topo.Mesh
	err   error
}

// newVerifyMeshCache fixes every body's chord from the pair jobs. A pair with
// no chord tolerance contributes nothing here: if it reaches the mesh path
// anyway, booloperand.PairChord refuses it there exactly as it would without
// the cache. The context is polled once per job.
func newVerifyMeshCache(ctx context.Context, h Hooks, jobs []verifyPairJob) (*verifyMeshCache, error) {
	cache := &verifyMeshCache{entries: map[*topo.Body]*verifyMeshEntry{}, mesher: h.Mesher, operand: h.Operand}
	shares := map[*topo.Body]meshbool.ChordOperand{}
	share := func(b *topo.Body) meshbool.ChordOperand {
		s, ok := shares[b]
		if !ok {
			s = booloperand.ChordShare(ctx, b)
			shares[b] = s
		}
		return s
	}
	for _, job := range jobs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if job.boxProven || job.a.Kind == topo.BodySheet || job.b.Kind == topo.BodySheet {
			continue
		}
		tol, _, ok := meshbool.PairChordFrom(share(job.a), share(job.b))
		if !ok {
			continue
		}
		for _, b := range [2]*topo.Body{job.a, job.b} {
			// A body that restates a held mesh chords nothing, so one mesh at
			// one chord cannot serve its pairs.
			if _, restating := booloperand.HeldFloor(b); restating {
				continue
			}
			e, ok := cache.entries[b]
			if !ok {
				cache.entries[b] = &verifyMeshEntry{chord: tol}
				continue
			}
			e.chord = math.Min(e.chord, tol)
		}
	}
	return cache, nil
}

// operandMesh returns b's shared mesh, building it on the first call. A body
// with no entry is meshed as pairMeshes meshes it. A body with an entry
// restates no held mesh, so its pairs never gate it. A failed build is kept
// like a mesh: the build is deterministic, so every pair would fail the same
// way, and each pair maps the error to its own operand index.
func (c *verifyMeshCache) TessellateOperand(ctx context.Context, node *topo.Body, pairTol float64) (*topo.Mesh, bool, error) {
	e, ok := c.entries[node]
	if !ok {
		return c.operand.TessellateOperand(ctx, node, pairTol)
	}
	e.once.Do(func() {
		e.mesh, e.err = c.mesher.VerifyAll(ctx, node, e.chord)
	})
	if e.err != nil {
		return nil, false, e.err
	}
	return e.mesh, false, nil
}

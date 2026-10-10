package decad

import (
	"context"
	"fmt"
	"math"
	"sync"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/units"
)

// This file is Verify's pair partition (docs/interference-design.md §2):
// the per-pair proof walk. Every pair's proof reads only its two operands
// and caches each body fills the same way whoever asks first, so a pair's
// outcome does not depend on which other pairs ran before it or beside it.
// Verify runs the jobs through internal/orderedwork and folds outcomes in
// pair order.
// verifyPairJob is one unordered pair, in Document.Bodies() order, that box
// separation alone does not finish.
type verifyPairJob struct {
	a, b      *Body
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
			boxProven := boxesDisjoint(bodies[i].Bounds.Box, bodies[j].Bounds.Box)
			if boxProven {
				sheetPair := a.Kind() == BodySheet && b.Kind() == BodySheet
				if sheetPair || !cfg.Clearances {
					continue
				}
			}
			jobs = append(jobs, verifyPairJob{a: a, b: b, boxProven: boxProven})
		}
	}
	return jobs, nil
}

// proveVerifyPair runs docs/interference-design.md §2's walk over one pair:
// a pair holding a sheet operand takes docs/surface-design.md §9.3's
// procedure; a solid pair takes the analytic pair kernel, then containment,
// equality and read-only intersection measurement. geomCache is nil unless
// clearances were asked; it and meshes must be safe for concurrent use.
func proveVerifyPair(ctx context.Context, job verifyPairJob, cfg verifyConfig, geomCache *bodyGeomCache, meshes operandMeshes) (verifyPairOutcome, error) {
	var out verifyPairOutcome
	a, b, boxProven := job.a, job.b, job.boxProven

	// A sheet operand encloses no region, so the interference relation §1
	// decides is not the question for this pair (docs/surface-design.md
	// §9.3). A sheet-sheet pair offers no closed boundary to cast against, so
	// it takes only box separation: separated boxes contribute nothing (the
	// job list already dropped them), and boxes that meet stay
	// DiagUnsupportedPairSheet regardless of WithClearances(). A
	// sheet-against-solid pair is decided by sheetSolidPair (clearance.go)
	// into a proven crossing, a proven containment or separation with a
	// measured gap, or the undecided code when the kernel cannot settle it —
	// run whenever the boxes meet (crossing must always be checked, asked or
	// not), and also when the boxes are separated but a gap was requested,
	// exactly as the solid-solid path below runs clearancePair in that same
	// second case.
	if a.Kind() == BodySheet || b.Kind() == BodySheet {
		if a.Kind() == BodySheet && b.Kind() == BodySheet {
			out.diagnostics = append(out.diagnostics,
				pairDiagNone(a, b, DiagUnsupportedPairSheet,
					"both operands are sheet bodies, and neither offers a closed boundary to cast the other against"))
			out.undecided = true
			return out, nil
		}
		sheet, solid := a, b
		if b.Kind() == BodySheet {
			sheet, solid = b, a
		}
		sres, err := sheetSolidPair(ctx, sheet, solid, boxProven, geomCache)
		if err != nil {
			return verifyPairOutcome{}, err
		}
		switch sres.verdict {
		case sheetSolidCrossing:
			out.diagnostics = append(out.diagnostics, Diagnostic{
				Code:    DiagSheetSolidCrossing,
				Status:  Interfering,
				Pair:    &DiagnosticPair{A: a, B: b},
				Reading: ReadingNone,
				Message: "the sheet crosses the solid's boundary; no Interference row is emitted because a sheet encloses no region and there is no overlap volume to report",
			})
		case sheetSolidContained, sheetSolidOutside:
			if cfg.Clearances {
				pr := pairResult{lo: sres.lo, hi: sres.hi, exact: sres.exact, diam: sres.diam}
				out.appendClearance(a, b, pr, cfg.Rel)
			}
		case sheetSolidUndecided:
			if boxProven {
				// Box separation already proves the sheet lies outside the
				// solid; only the requested gap itself is unmeasured, the
				// same shape of gap as an unmeasured solid-solid clearance
				// below.
				out.diagnostics = append(out.diagnostics,
					pairDiagNone(a, b, DiagUndecidedClearance,
						"the pair is proven disjoint but the requested clearance gap is unmeasured"))
			} else {
				out.diagnostics = append(out.diagnostics,
					pairDiagNone(a, b, DiagUnsupportedPairSheet,
						"the clearance kernel could not settle this sheet-against-solid pair"))
			}
			out.undecided = true
		}
		return out, nil
	}

	res, fast := clearanceAxisBoxes(a, b)
	if !fast {
		var err error
		res, err = clearancePairCached(ctx, a, b, boxProven, geomCache)
		if err != nil {
			return verifyPairOutcome{}, err
		}
	}
	if res.verdict == pairUndecided {
		// The exact planar relation (clearance_planar.go) answers what the
		// analytic kernel left open — a pair an operand has no carrier model
		// in, or one its enumeration could not settle — and only ever adds a
		// verdict, never replaces one (interference design §3.2).
		planar, ok, err := planarPairVerdict(ctx, a, b)
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
		if res.verdict == pairDisjoint || res.verdict == pairTouching {
			out.appendClearance(a, b, res, cfg.Rel)
			return out, nil
		}
		out.diagnostics = append(out.diagnostics,
			pairDiagNone(a, b, DiagUndecidedClearance,
				"the pair is proven disjoint but the requested clearance gap is unmeasured"))
		out.undecided = true
		return out, nil
	}

	if res.verdict == pairDisjoint || res.verdict == pairTouching {
		if cfg.Clearances {
			out.appendClearance(a, b, res, cfg.Rel)
		}
		return out, nil
	}

	volume, outcome, err := measuredInterference(ctx, a, b, res, meshes)
	if err != nil {
		return verifyPairOutcome{}, err
	}
	if outcome != interferenceMeasured {
		out.diagnostics = append(out.diagnostics, undecidedPairDiag(a, b, res.verdict, outcome))
		out.undecided = true
		return out, nil
	}
	out.interferences = append(out.interferences, Interference{A: a, B: b, Volume: volume})
	pairD, err := interferencePairDiameter(ctx, a, b)
	if err != nil {
		return verifyPairOutcome{}, err
	}
	obs := volume
	out.diagnostics = append(out.diagnostics, Diagnostic{
		Code:     DiagInterference,
		Status:   Interfering,
		Pair:     &DiagnosticPair{A: a, B: b},
		Reading:  ReadingOverlapVolume,
		Observed: &obs,
		Message:  "the pair is proven to overlap",
	})
	pass, ref, haveRef := interferenceToleranceRef(volume, a, b, pairD, cfg.Rel)
	if !pass {
		beyond := Diagnostic{
			Code:     DiagMeasurementBeyondTolerance,
			Status:   Suspect,
			Pair:     &DiagnosticPair{A: a, B: b},
			Reading:  ReadingOverlapVolume,
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
func (o *verifyPairOutcome) appendClearance(a, b *Body, res pairResult, rel float64) {
	scratch := Report{}
	if d := appendClearance(&scratch, a, b, res, rel); d != nil {
		o.diagnostics = append(o.diagnostics, *d)
		o.undecided = true
	}
	o.clearances = append(o.clearances, scratch.Clearances...)
}

// verifyMeshCache meshes each body once per Verify call, at one chord for
// every pair it takes part in (docs/interference-design.md §5.3). The chord is
// the tightest one any of the body's candidate pairs would have asked for:
// the least pairChordTolerance over the solid pairs whose boxes meet, which
// are the only pairs that can reach the mesh path. The entries are fixed
// before the worker pool starts and only read after it, and each entry's mesh
// is built under its own sync.Once, so one Verify call meshes and audits a
// body once however many workers ask for it at the same time.
//
// A body whose tessellation restates a held mesh (a faceted Boolean result,
// a mitred sweep; heldFloorOf) chords nothing, so there is nothing to share:
// it has no entry, and every pair meshes it as the public booleans do, at the
// pair's own tolerance raised to its held floor, and gates the facets that
// pair touches at the pair's own tolerance (docs/api-design.md §8 "The chain
// depth", docs/faceted-vertex-bounds-design.md §5).
type verifyMeshCache struct {
	entries map[*Body]*verifyMeshEntry
}

// verifyMeshEntry is one body's shared mesh: the chord it is meshed at, and
// the outcome of the one tessellation that builds it.
type verifyMeshEntry struct {
	chord float64
	once  sync.Once
	mesh  *Mesh
	err   error
}

// newVerifyMeshCache fixes every body's chord from the pair jobs. A body or a
// pair whose chord share cannot be read contributes nothing here: a pair that
// reaches the mesh path anyway reads its own pairChordTolerance there and
// fails with that error exactly as it would without the cache. The context is
// polled once per job.
func newVerifyMeshCache(ctx context.Context, jobs []verifyPairJob) (*verifyMeshCache, error) {
	cache := &verifyMeshCache{entries: map[*Body]*verifyMeshEntry{}}
	shares := map[*Body]meshbool.ChordOperand{}
	unreadable := map[*Body]struct{}{}
	share := func(b *Body) (meshbool.ChordOperand, bool) {
		if s, ok := shares[b]; ok {
			return s, true
		}
		if _, ok := unreadable[b]; ok {
			return meshbool.ChordOperand{}, false
		}
		s, err := chordOperandOf(ctx, b)
		if err != nil {
			unreadable[b] = struct{}{}
			return meshbool.ChordOperand{}, false
		}
		shares[b] = s
		return s, true
	}
	for _, job := range jobs {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		if job.boxProven || job.a.Kind() == BodySheet || job.b.Kind() == BodySheet {
			continue
		}
		sa, ok := share(job.a)
		if !ok {
			continue
		}
		sb, ok := share(job.b)
		if !ok {
			continue
		}
		tol, _, ok := meshbool.PairChordFrom(sa, sb)
		if !ok {
			continue
		}
		for _, b := range [2]*Body{job.a, job.b} {
			if !sharesVerifyMesh(b) {
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

// sharesVerifyMesh reports whether b's tessellation chords its boundary, so
// that one mesh at one chord can serve every pair b takes part in: whether b
// restates no held mesh.
func sharesVerifyMesh(b *Body) bool {
	_, restating := heldFloorOf(b)
	return !restating
}

// operandMesh returns b's shared mesh, building it on the first call. A body
// with no entry is meshed as pairMeshes meshes it. A body with an entry
// restates no held mesh, so its pairs never gate it. A failed build is kept
// like a mesh: the build is deterministic, so every pair would fail the same
// way, and each pair maps the error to its own operand index.
func (c *verifyMeshCache) operandMesh(ctx context.Context, b *Body, pairTol float64) (*Mesh, bool, error) {
	e, ok := c.entries[b]
	if !ok {
		return pairMeshes{}.operandMesh(ctx, b, pairTol)
	}
	e.once.Do(func() {
		e.mesh, e.err = tessellateContext(ctx, b, units.Millimeters(e.chord), VerifyAll)
	})
	return e.mesh, false, e.err
}

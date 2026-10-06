package decad

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/meshbool"
)

// This file is Verify's pair partition (docs/interference-design.md §2): the
// per-pair proof walk, and the ordered executor that runs the pairs needing
// kernel work on a bounded worker pool. Every pair's proof reads only its two
// operands and caches each body fills the same way whoever asks first, so a
// pair's outcome does not depend on which other pairs ran before it or
// beside it. The executor stores each outcome in its own slot and Verify
// folds the slots in pair order, so rows, diagnostics and the returned error
// are the ones a walk of the pairs one at a time produces.

type verifyWorkerContextKey struct{}

// withVerifyWorkers is an internal test hook. It sets how many pairs one
// Verify call proves at once, without touching package state.
func withVerifyWorkers(ctx context.Context, workers int) context.Context {
	if workers < 1 {
		workers = 1
	}
	return context.WithValue(ctx, verifyWorkerContextKey{}, workers)
}

// verifyWorkers is the pair pool size: the test hook's value, or the same
// GOMAXPROCS-derived cap the boolean's contact batches use.
func verifyWorkers(ctx context.Context) int {
	if workers, ok := ctx.Value(verifyWorkerContextKey{}).(int); ok && workers > 0 {
		return workers
	}
	return meshbool.DefaultContactWorkers()
}

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
				if sheetPair || !cfg.clearances {
					continue
				}
			}
			jobs = append(jobs, verifyPairJob{a: a, b: b, boxProven: boxProven})
		}
	}
	return jobs, nil
}

// runVerifyPairs proves every job and returns the outcomes in job order. With
// one worker it walks the jobs in order on the calling goroutine and stops at
// the first error. With more, workers take jobs in order and each writes only
// its own slot; once a job fails, no job after it starts, and every job
// before it still runs, so the error returned is the one at the lowest
// failing index — the error the one-at-a-time walk would have stopped on.
// The context is polled before each job.
func runVerifyPairs(ctx context.Context, jobs []verifyPairJob, workers int, prove func(context.Context, verifyPairJob) (verifyPairOutcome, error)) ([]verifyPairOutcome, error) {
	out := make([]verifyPairOutcome, len(jobs))
	if workers > len(jobs) {
		workers = len(jobs)
	}
	if workers <= 1 {
		for i, job := range jobs {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
			res, err := prove(ctx, job)
			if err != nil {
				return nil, err
			}
			out[i] = res
		}
		return out, nil
	}

	errs := make([]error, len(jobs))
	var next atomic.Int64
	var firstErr atomic.Int64
	firstErr.Store(int64(len(jobs)))
	fail := func(i int, err error) {
		errs[i] = err
		for {
			cur := firstErr.Load()
			if int64(i) >= cur || firstErr.CompareAndSwap(cur, int64(i)) {
				return
			}
		}
	}
	var wg sync.WaitGroup
	wg.Add(workers)
	for range workers {
		go func() {
			defer wg.Done()
			for {
				i := int(next.Add(1) - 1)
				if i >= len(jobs) || int64(i) > firstErr.Load() {
					return
				}
				if err := ctx.Err(); err != nil {
					fail(i, err)
					return
				}
				res, err := prove(ctx, jobs[i])
				if err != nil {
					fail(i, err)
					continue
				}
				out[i] = res
			}
		}()
	}
	wg.Wait()
	if i := firstErr.Load(); i < int64(len(jobs)) {
		return nil, errs[i]
	}
	return out, nil
}

// proveVerifyPair runs docs/interference-design.md §2's walk over one pair:
// a pair holding a sheet operand takes docs/surface-design.md §9.3's
// procedure; a solid pair takes the analytic pair kernel, then containment,
// equality and read-only intersection measurement. geomCache is nil unless
// clearances were asked, and must be safe for concurrent use.
func proveVerifyPair(ctx context.Context, job verifyPairJob, cfg verifyConfig, geomCache *bodyGeomCache) (verifyPairOutcome, error) {
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
			if cfg.clearances {
				pr := pairResult{lo: sres.lo, hi: sres.hi, exact: sres.exact, diam: sres.diam}
				out.appendClearance(a, b, pr, cfg.rel)
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
			out.appendClearance(a, b, res, cfg.rel)
			return out, nil
		}
		out.diagnostics = append(out.diagnostics,
			pairDiagNone(a, b, DiagUndecidedClearance,
				"the pair is proven disjoint but the requested clearance gap is unmeasured"))
		out.undecided = true
		return out, nil
	}

	if res.verdict == pairDisjoint || res.verdict == pairTouching {
		if cfg.clearances {
			out.appendClearance(a, b, res, cfg.rel)
		}
		return out, nil
	}

	volume, outcome, err := measuredInterference(ctx, a, b, res)
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
	pass, ref, haveRef := interferenceToleranceRef(volume, a, b, pairD, cfg.rel)
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
			beyond.Required = requiredThreshold(cfg.rel*ref, volume.Value)
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

package parts

import (
	"cmp"
	"context"
	"fmt"
	"math"
	"runtime"
	"slices"
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/jointcheck"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// witnessSample is how many mesh vertices of each part G2 tries.
const witnessSample = 64

// witness is G2's answer: the least proven upper end of a pair's gap among the
// sampled points, and the point it was read at.
type witness struct {
	upper float64
	point r3.Vec
	ok    bool // a point was measured
}

// distancesReadable reports whether point distances to both parts are
// available: each was meshed at VerifyAll (G0).
func distancesReadable(a, b *bodyFaces) bool { return a.verified && b.verified }

// findWitness samples up to witnessSample mesh vertices of a nearest b's box and
// of b nearest a's box, and returns the least sum of the two bodies' distances to
// a point, each taken at its upper end (the triangle inequality: the gap is at
// most that sum). It stops early at a point whose sum is below stopBelow. It finds
// a witness when one exists among the sampled points; finding none proves
// nothing.
func findWitness(ctx context.Context, a, b *bodyFaces, ba, bb *decad.Body, stopBelow float64) (witness, error) {
	best := witness{upper: math.Inf(1)}
	tolA, tolB := units.Millimeters(a.meshTol), units.Millimeters(b.meshTol)
	try := func(p r3.Vec) (bool, error) {
		da, err := ba.DistanceToPoint(ctx, p, tolA)
		if err != nil {
			return false, err
		}
		db, err := bb.DistanceToPoint(ctx, p, tolB)
		if err != nil {
			return false, err
		}
		upper := da.Value.Base() + da.Bound.Base() + db.Value.Base() + db.Bound.Base()
		best.ok = true
		if upper < best.upper {
			best.upper, best.point = upper, p
		}
		return best.upper < stopBelow, nil
	}
	points := append(nearestVertices(a.verts, partBox(b)), nearestVertices(b.verts, partBox(a))...)
	for _, p := range points {
		if err := ctx.Err(); err != nil {
			return witness{}, err
		}
		done, err := try(p)
		if err != nil {
			return witness{}, err
		}
		if done {
			break
		}
	}
	return best, nil
}

// nearestVertices returns up to witnessSample of verts nearest the box.
func nearestVertices(verts []r3.Vec, box jointcheck.Box) []r3.Vec {
	type vd struct {
		v r3.Vec
		d float64
	}
	all := make([]vd, len(verts))
	for i, v := range verts {
		all[i] = vd{v, box.PointDistance(v)}
	}
	slices.SortStableFunc(all, func(x, y vd) int { return cmp.Compare(x.d, y.d) })
	out := make([]r3.Vec, 0, min(witnessSample, len(all)))
	for _, e := range all[:min(witnessSample, len(all))] {
		out = append(out, e.v)
	}
	return out
}

// interval is the measurement of the gap interval [lo, hi]: its midpoint and
// half width.
func interval(lo, hi float64) *decad.Measurement {
	return &decad.Measurement{Value: units.Millimeters((lo + hi) / 2), Exactness: decad.Approximate,
		Bound: units.Millimeters((hi - lo) / 2)}
}

// checkContact is J4.
func checkContact(ctx context.Context, r *JointResult, a, b *bodyFaces, ba, bb *decad.Body, c config) error {
	low := boxGapLow(a, b)
	high := jointcheck.GapHigh(partBox(a), partBox(b))
	r.Gap = interval(low, high)
	switch {
	case low > c.contactTol:
		r.settlef(Contradicted, CauseApart, "the parts' boxes are at least %.3g mm apart, more than the %.3g mm contact tolerance",
			low, c.contactTol)
		return nil
	case !distancesReadable(a, b):
		r.settlef(Undecided, CauseUnreadFace, "point distances are unavailable: a body would not mesh at VerifyAll")
		return nil
	case a.meshBound+b.meshBound > c.contactTol:
		r.settlef(Undecided, CauseNoWitness, "below resolution: the parts' mesh bounds of %.3g mm exceed the %.3g mm contact tolerance",
			a.meshBound+b.meshBound, c.contactTol)
		return nil
	}
	w, err := findWitness(ctx, a, b, ba, bb, c.contactTol)
	if err != nil {
		return err
	}
	if w.ok && w.upper <= c.contactTol {
		pt := w.point
		r.Witness, r.Gap = &pt, interval(low, math.Max(low, w.upper))
		r.Status, r.Cause, r.Reason = Confirmed, CauseNone, ""
		return nil
	}
	r.settlef(Undecided, CauseNoWitness, "no sampled point proves the parts within %.3g mm of each other; the least sum is %.3g mm",
		c.contactTol, w.upper)
	return nil
}

// checkKeepout is J5.
func checkKeepout(ctx context.Context, r *JointResult, a, b *bodyFaces, ba, bb *decad.Body) error {
	minGap := r.Joint.MinGap.Base()
	low := boxGapLow(a, b)
	high := jointcheck.GapHigh(partBox(a), partBox(b))
	r.Gap = interval(low, high)
	if low >= minGap {
		r.Status, r.Cause, r.Reason = Confirmed, CauseNone, ""
		return nil
	}
	if distancesReadable(a, b) {
		w, err := findWitness(ctx, a, b, ba, bb, minGap)
		if err != nil {
			return err
		}
		if w.ok && w.upper < minGap {
			pt := w.point
			r.Witness, r.Gap = &pt, interval(low, math.Max(low, w.upper))
			r.settlef(Contradicted, CauseTooClose, "the parts are within %.3g mm of each other, closer than the %.3g mm minimum", w.upper, minGap)
			return nil
		}
	}
	r.settlef(Undecided, CauseNeedsPairProof, "the boxes come within %.3g mm and no sampled point shows the parts closer; only a pair proof decides it",
		minGap)
	return nil
}

// parallel runs f(0..n-1) on up to GOMAXPROCS goroutines and returns the first
// error. f must only read shared state and write to its own index.
func parallel(ctx context.Context, n int, f func(i int) error) error {
	workers := min(runtime.GOMAXPROCS(0), n)
	var (
		next     atomic.Int64
		wg       sync.WaitGroup
		mu       sync.Mutex
		firstErr error
	)
	for range workers {
		wg.Go(func() {
			for {
				i := int(next.Add(1)) - 1
				if i >= n {
					return
				}
				mu.Lock()
				stop := firstErr != nil
				mu.Unlock()
				if stop {
					return
				}
				if err := ctx.Err(); err != nil {
					mu.Lock()
					if firstErr == nil {
						firstErr = err
					}
					mu.Unlock()
					return
				}
				if err := f(i); err != nil {
					mu.Lock()
					if firstErr == nil {
						firstErr = fmt.Errorf("parts: %w", err)
					}
					mu.Unlock()
					return
				}
			}
		})
	}
	wg.Wait()
	return firstErr
}

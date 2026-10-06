package decad

import (
	"context"
	"errors"
	"math"
	"strconv"
	"sync"
	"testing"
	"time"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/stretchr/testify/require"
)

// pairJobsByIndex builds n jobs whose first operand identifies the job, so a
// prove function can tell which job it was handed.
func pairJobsByIndex(n int) ([]verifyPairJob, map[*Body]int) {
	jobs := make([]verifyPairJob, n)
	index := make(map[*Body]int, n)
	for i := range jobs {
		a := &Body{}
		jobs[i] = verifyPairJob{a: a}
		index[a] = i
	}
	return jobs, index
}

// TestRunVerifyPairsKeepsJobOrder finishes later jobs first and requires the
// outcomes back in job order, whatever order the workers complete in.
func TestRunVerifyPairsKeepsJobOrder(t *testing.T) {
	t.Parallel()
	const n = 24
	jobs, index := pairJobsByIndex(n)
	prove := func(_ context.Context, job verifyPairJob) (verifyPairOutcome, error) {
		i := index[job.a]
		time.Sleep(time.Duration(n-i) * time.Millisecond)
		return verifyPairOutcome{diagnostics: []Diagnostic{{Message: strconv.Itoa(i)}}, undecided: i%2 == 0}, nil
	}
	for _, workers := range []int{1, 2, 8, 64} {
		out, err := runVerifyPairs(t.Context(), jobs, workers, prove)
		require.NoError(t, err, `%d workers`, workers)
		require.Len(t, out, n)
		for i, o := range out {
			require.Equal(t, strconv.Itoa(i), o.diagnostics[0].Message, `%d workers, slot %d`, workers, i)
			require.Equal(t, i%2 == 0, o.undecided)
		}
	}
}

// TestRunVerifyPairsReturnsLowestFailingError fails two jobs, the later one
// first, and requires the error of the earlier one: the error a walk of the
// jobs in order stops on. Every job before it must still have run.
func TestRunVerifyPairsReturnsLowestFailingError(t *testing.T) {
	t.Parallel()
	const n = 20
	errEarly := errors.New("early job failed")
	errLate := errors.New("late job failed")
	for _, workers := range []int{1, 3, 8} {
		jobs, index := pairJobsByIndex(n)
		ran := make([]bool, n)
		prove := func(_ context.Context, job verifyPairJob) (verifyPairOutcome, error) {
			i := index[job.a]
			ran[i] = true
			switch i {
			case 5:
				time.Sleep(30 * time.Millisecond)
				return verifyPairOutcome{}, errEarly
			case 12:
				return verifyPairOutcome{}, errLate
			}
			return verifyPairOutcome{}, nil
		}
		out, err := runVerifyPairs(t.Context(), jobs, workers, prove)
		require.ErrorIs(t, err, errEarly, `%d workers`, workers)
		require.Nil(t, out)
		for i := range 5 {
			require.True(t, ran[i], `%d workers: job %d precedes the failure and must run`, workers, i)
		}
	}

	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	jobs, _ := pairJobsByIndex(n)
	_, err := runVerifyPairs(ctx, jobs, 4, func(context.Context, verifyPairJob) (verifyPairOutcome, error) {
		return verifyPairOutcome{}, nil
	})
	require.ErrorIs(t, err, context.Canceled)
}

// internalFrustumBody revolves the outline (0,0) (0,ra) (L,rb) (L,0) about
// the sketch's U axis and places U along a → b.
func internalFrustumBody(t *testing.T, doc *Document, a, b r3.Vec, ra, rb float64) *Body {
	t.Helper()
	l := b.Sub(a).Len()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pts := [][2]float64{{0, 0}, {0, ra}, {l, rb}, {l, 0}}
	ps := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		ps[i] = s.CreatePoint(p[0], p[1])
		s.Fix(ps[i])
	}
	for i := range ps {
		s.CreateLine(ps[i], ps[(i+1)%len(ps)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := doc.Revolve(s, s.Profiles()[0], SketchLine{End: Point2{U: 1}}, FullRevolution{})
	require.NoError(t, err)
	d, ok := b.Sub(a).Normalize()
	require.True(t, ok)
	ref := r3.NewVec(0, 1, 0)
	if math.Abs(d.Y) > 0.9 {
		ref = r3.NewVec(1, 0, 0)
	}
	p, ok := d.Cross(ref).Normalize()
	require.True(t, ok)
	place, err := r3.FromBasis(r3.Basis{EX: d, EY: p.Cross(d), EZ: p}, a)
	require.NoError(t, err)
	body, err = body.Placed(t.Context(), place)
	require.NoError(t, err)
	return body
}

// TestVerifyPairWorkersMatchOneWorker verifies one document with one worker
// and with several, with and without clearances, and requires the two reports
// equal field for field. The document holds every pair path: overlapping
// revolves measured through the mesh intersection, an overlapping box pair
// measured analytically, disjoint boxes whose gap is measured, sheet against
// solid, and sheet against sheet.
func TestVerifyPairWorkersMatchOneWorker(t *testing.T) {
	t.Parallel()
	doc := New()
	internalFrustumBody(t, doc, r3.NewVec(0, 0, 0), r3.NewVec(0, 0, 6), 1.5, 1.2)
	internalFrustumBody(t, doc, r3.NewVec(-3, 0, 2), r3.NewVec(3, 0.4, 5), 1.2, 1.0)
	internalBoxBody(t, doc, 10, 0, 14, 4, 3)
	internalBoxBody(t, doc, 12, 2, 16, 6, 3)
	internalBoxBody(t, doc, 20, 0, 22, 2, 2)
	internalSheetBody(t, doc, 13, 1, 15, 3, 5)
	internalSheetBody(t, doc, 14, 2, 18, 5, 1)
	for _, opts := range [][]VerifyOption{nil, {WithClearances()}} {
		one, err := doc.Verify(withVerifyWorkers(t.Context(), 1), opts...)
		require.NoError(t, err)
		many, err := doc.Verify(withVerifyWorkers(t.Context(), 6), opts...)
		require.NoError(t, err)
		require.NotEmpty(t, one.Interferences)
		require.Equal(t, one, many)
	}
}

// meshOnceCylinders places three cylinders on the three axes through the
// origin. Their radii differ, so no two surfaces touch tangentially, and
// their lengths differ, so each body's two pairs ask for different chord
// tolerances: the shared mesh of every body is finer than at least one of
// its pairs would have chorded it alone. Every pair crosses transversally and
// reaches the read-only mesh intersection.
func meshOnceCylinders(t *testing.T) (*Document, [3]*Body, [3]float64) {
	t.Helper()
	doc := New()
	radii := [3]float64{3, 2, 1.5}
	half := [3]float64{20, 12, 6}
	var bodies [3]*Body
	for i := range bodies {
		var axis r3.Vec
		switch i {
		case 0:
			axis = r3.NewVec(1, 0, 0)
		case 1:
			axis = r3.NewVec(0, 1, 0)
		default:
			axis = r3.NewVec(0, 0, 1)
		}
		bodies[i] = internalFrustumBody(t, doc, axis.Scale(-half[i]), axis.Scale(half[i]), radii[i], radii[i])
	}
	return doc, bodies, radii
}

// bicylinderVolume is the volume two cylinders of radii big ≥ small share
// when their axes cross at right angles: integrating the big cylinder's chord
// length 2·√(big² − z²) over the small one's disc gives
// 4·small²·∫ cos²t·√(big² − small²·sin²t) dt over t in [−π/2, π/2]. The
// integrand is smooth, and composite Simpson over 4096 panels is accurate far
// below any bound the mesh path publishes.
func bicylinderVolume(big, small float64) float64 {
	const n = 4096
	lo, hi := -math.Pi/2, math.Pi/2
	h := (hi - lo) / n
	g := func(t float64) float64 {
		c, s := math.Cos(t), math.Sin(t)
		return c * c * math.Sqrt(big*big-small*small*s*s)
	}
	sum := g(lo) + g(hi)
	for k := 1; k < n; k++ {
		w := 2.0
		if k%2 == 1 {
			w = 4
		}
		sum += w * g(lo+float64(k)*h)
	}
	return 4 * small * small * sum * h / 3
}

// TestVerifyMeshesEachBodyOnce verifies three mutually crossing cylinders and
// requires every body tessellated exactly once, although each one takes part
// in two mesh-path pairs whose own chord tolerances differ. It also requires
// the chord each body is meshed at to be the least of its pairs' tolerances.
func TestVerifyMeshesEachBodyOnce(t *testing.T) {
	t.Parallel()
	doc, bodies, _ := meshOnceCylinders(t)

	jobs := []verifyPairJob{{a: bodies[0], b: bodies[1]}, {a: bodies[0], b: bodies[2]}, {a: bodies[1], b: bodies[2]}}
	cache, err := newVerifyMeshCache(t.Context(), jobs)
	require.NoError(t, err)
	finer := 0
	for _, b := range bodies {
		entry, ok := cache.entries[b]
		require.True(t, ok)
		least := math.Inf(1)
		for _, job := range jobs {
			if job.a != b && job.b != b {
				continue
			}
			tol, _, err := pairChordTolerance(t.Context(), job.a, job.b)
			require.NoError(t, err)
			if tol > entry.chord {
				finer++
			}
			least = math.Min(least, tol)
		}
		require.Equal(t, least, entry.chord)
	}
	require.Positive(t, finer, `the fixture must give some body a chord finer than one of its pairs' own`)

	count := &tessellationCount{}
	report, err := doc.Verify(withTessellationCount(withVerifyWorkers(t.Context(), 3), count))
	require.NoError(t, err)
	require.Len(t, report.Interferences, 3)
	for i, b := range bodies {
		require.Equal(t, 1, count.of(b), `body %d`, i)
	}
}

// TestVerifyOverlapEnclosesBicylinderVolume requires every overlap volume
// Verify measures through shared meshes to enclose the pair's true volume,
// computed independently by bicylinderVolume, and to prove it positive.
func TestVerifyOverlapEnclosesBicylinderVolume(t *testing.T) {
	t.Parallel()
	doc, bodies, radii := meshOnceCylinders(t)
	index := map[*Body]int{}
	for i, b := range bodies {
		index[b] = i
	}
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Len(t, report.Interferences, 3)
	for _, row := range report.Interferences {
		ra, rb := radii[index[row.A]], radii[index[row.B]]
		want := bicylinderVolume(math.Max(ra, rb), math.Min(ra, rb))
		v, e := row.Volume.Value.Base(), row.Volume.Bound.Base()
		require.Positive(t, v-e, `radii %g and %g`, ra, rb)
		require.LessOrEqual(t, math.Abs(v-want), e, `radii %g and %g: %g ± %g against %g`, ra, rb, v, e, want)
	}
}

// TestVerifyMeshCacheBuildsOnceUnderContention asks the cache for one body
// from many goroutines at once and requires a single build whose mesh every
// caller receives. Run under -race, it also guards the cache's own memory
// safety.
func TestVerifyMeshCacheBuildsOnceUnderContention(t *testing.T) {
	t.Parallel()
	_, bodies, _ := meshOnceCylinders(t)
	jobs := []verifyPairJob{{a: bodies[0], b: bodies[1]}, {a: bodies[0], b: bodies[2]}}
	cache, err := newVerifyMeshCache(t.Context(), jobs)
	require.NoError(t, err)
	count := &tessellationCount{}
	ctx := withTessellationCount(t.Context(), count)
	const callers = 8
	meshes := make([]*Mesh, callers)
	errs := make([]error, callers)
	var wg sync.WaitGroup
	start := make(chan struct{})
	for i := range callers {
		wg.Go(func() {
			<-start
			meshes[i], _, errs[i] = cache.operandMesh(ctx, bodies[0], 1)
		})
	}
	close(start)
	wg.Wait()
	for i := range callers {
		require.NoError(t, errs[i])
		require.Same(t, meshes[0], meshes[i])
	}
	require.Equal(t, 1, count.of(bodies[0]))
}

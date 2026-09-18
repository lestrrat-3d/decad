package decad_test

import (
	"context"
	"io"
	"math"
	"os"
	"runtime"
	"slices"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

const cacheTolerance = 0.5

func cacheTorusBody() *decad.Body {
	return benchRevolve(func(s *sketch.Sketch) {
		center := s.CreatePoint(0, 10)
		s.Fix(center)
		s.CreateCircle(center, 3)
	}, decad.FullRevolution{})
}

func TestTessellationCacheKeysAndReplacement(t *testing.T) {
	t.Parallel()
	body := holedPlateBody(t)
	tol := units.Millimeters(cacheTolerance)
	first, err := body.Tessellate(tol)
	require.NoError(t, err)
	hit, err := body.Tessellate(tol)
	require.NoError(t, err)
	require.Same(t, first, hit, "an identical normalized key returns the cached mesh")

	metric := units.Millimeters(25.4)
	metricMesh, err := body.Tessellate(metric)
	require.NoError(t, err)
	imperialMesh, err := body.Tessellate(units.Inches(1))
	require.NoError(t, err)
	require.Same(t, metricMesh, imperialMesh, "exactly equal normalized units share a key")

	nearby, err := body.Tessellate(units.Millimeters(math.Nextafter(25.4, math.Inf(1))))
	require.NoError(t, err)
	require.NotSame(t, metricMesh, nearby, "nearby unequal normalized tolerances do not share a key")

	replacement, err := body.Tessellate(units.Millimeters(1))
	require.NoError(t, err)
	require.NotSame(t, nearby, replacement)
	switchedBack, err := body.Tessellate(metric)
	require.NoError(t, err)
	require.NotSame(t, metricMesh, switchedBack, "the one-entry cache evicts the previous tolerance")
	finalHit, err := body.Tessellate(metric)
	require.NoError(t, err)
	require.Same(t, switchedBack, finalHit)
}

func TestTessellationCacheErrorsCancellationAndAccessorCopies(t *testing.T) {
	t.Parallel()
	body := holedPlateBody(t)
	tol := units.Millimeters(cacheTolerance)
	warm, err := body.Tessellate(tol)
	require.NoError(t, err)

	bad, err := body.Tessellate(units.Millimeters(1e-20))
	require.Nil(t, bad)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	afterFailure, err := body.Tessellate(tol)
	require.NoError(t, err)
	require.Same(t, warm, afterFailure, "a refusal does not evict a successful entry")

	canceled, cancel := context.WithCancel(t.Context())
	cancel()
	got, err := body.TessellateContext(canceled, tol)
	require.Nil(t, got)
	require.ErrorIs(t, err, context.Canceled)
	afterCancel, err := body.Tessellate(tol)
	require.NoError(t, err)
	require.Same(t, warm, afterCancel, "a canceled hit does not evict a successful entry")

	vertices := warm.Vertices()
	triangles := warm.Triangles()
	sources := warm.SourceFaces()
	wantVertex, wantTriangle, wantSource := vertices[0], triangles[0], sources[0]
	vertices[0].X++
	triangles[0][0]++
	sources[0] = nil
	unchanged, err := body.Tessellate(tol)
	require.NoError(t, err)
	require.Equal(t, wantVertex, unchanged.Vertices()[0])
	require.Equal(t, wantTriangle, unchanged.Triangles()[0])
	require.Same(t, wantSource, unchanged.SourceFaces()[0])

	fresh := holedPlateBody(t)
	preCanceled, cancel := context.WithCancel(t.Context())
	cancel()
	got, err = fresh.TessellateContext(preCanceled, tol)
	require.Nil(t, got)
	require.ErrorIs(t, err, context.Canceled)
	uncanceled, err := fresh.Tessellate(tol)
	require.NoError(t, err)
	require.Same(t, uncanceled, mustTessellate(t, fresh, tol))
}

func mustTessellate(t *testing.T, body *decad.Body, tol units.Value) *decad.Mesh {
	t.Helper()
	mesh, err := body.Tessellate(tol)
	require.NoError(t, err)
	return mesh
}

func TestTessellationCachePayloadClasses(t *testing.T) {
	t.Parallel()
	fixtures := []struct {
		name string
		body func(*testing.T) *decad.Body
	}{
		{name: "prism", body: holedPlateBody},
		{name: "cup", body: func(t *testing.T) *decad.Body {
			_, box := shellBox(t)
			body, err := box.Shell(topCap(box), units.Millimeters(5))
			require.NoError(t, err)
			return body
		}},
		{name: "loft", body: func(t *testing.T) *decad.Body {
			doc := decad.New()
			return loftBoxAt(t, doc, 0, 10)
		}},
		{name: "faceted", body: func(t *testing.T) *decad.Body {
			doc := decad.New()
			plate := boxBody(t, doc, 0, 0, 20, 20, 8)
			tool := translated(t, diskBody(t, doc, 10, 10, 2), 0, 0, -6)
			body, err := decad.Cut(plate, tool)
			require.NoError(t, err)
			return body
		}},
		{name: "revolve", body: func(t *testing.T) *decad.Body {
			s, p := solidSketch(t)
			body, err := decad.New().Revolve(s, p, uAxis, decad.FullRevolution{})
			require.NoError(t, err)
			return body
		}},
		{name: "cap blend", body: func(t *testing.T) *decad.Body {
			_, box := capBlendBox(t)
			body, err := box.Chamfer(capLoopEdges(box), units.Millimeters(5))
			require.NoError(t, err)
			return body
		}},
	}
	for _, fixture := range fixtures {
		t.Run(fixture.name, func(t *testing.T) {
			body := fixture.body(t)
			first := mustTessellate(t, body, units.Millimeters(1))
			hit := mustTessellate(t, body, units.Millimeters(1))
			require.Same(t, first, hit)
			require.Len(t, first.SourceFaces(), len(first.Triangles()))
			live := map[*decad.Face]struct{}{}
			for _, face := range body.Faces() {
				live[face] = struct{}{}
			}
			for _, face := range first.SourceFaces() {
				_, ok := live[face]
				require.True(t, ok, "cache preserves live SourceFaces identity")
			}
		})
	}
}

func TestTessellationCacheCopiesDoNotInheritEntries(t *testing.T) {
	t.Parallel()
	tol := units.Millimeters(cacheTolerance)
	base := holedPlateBody(t)
	baseMesh := mustTessellate(t, base, tol)

	duplicate, err := base.Duplicate()
	require.NoError(t, err)
	duplicateMesh := mustTessellate(t, duplicate, tol)
	require.NotSame(t, baseMesh, duplicateMesh)
	require.NotSame(t, base.Faces()[0], duplicate.Faces()[0])
	require.Same(t, duplicateMesh, mustTessellate(t, duplicate, tol))

	copied, err := base.PlacedCopy(mustTransform(t, r3.NewVec(5, 0, 0)))
	require.NoError(t, err)
	copyMesh := mustTessellate(t, copied, tol)
	require.NotSame(t, baseMesh, copyMesh)
	require.NotSame(t, base.Faces()[0], copied.Faces()[0])

	placedSource := holedPlateBody(t)
	placedSourceMesh := mustTessellate(t, placedSource, tol)
	placed, err := placedSource.Placed(mustTransform(t, r3.NewVec(5, 0, 0)))
	require.NoError(t, err)
	placedMesh := mustTessellate(t, placed, tol)
	require.NotSame(t, placedSourceMesh, placedMesh)
	require.NotSame(t, placedSource.Faces()[0], placed.Faces()[0])
}

func mustTransform(t testing.TB, v r3.Vec) r3.Transform {
	t.Helper()
	transform, err := r3.Translation(v)
	require.NoError(t, err)
	return transform
}

func TestTessellationCacheConcurrentColdAndHit(t *testing.T) {
	t.Parallel()
	body := holedPlateBody(t)
	const callers = 4
	start := make(chan struct{})
	meshes := make([]*decad.Mesh, callers)
	errs := make([]error, callers)
	var wg sync.WaitGroup
	wg.Add(callers)
	for i := range callers {
		go func() {
			defer wg.Done()
			<-start
			meshes[i], errs[i] = body.TessellateContext(t.Context(), units.Millimeters(cacheTolerance))
		}()
	}
	close(start)
	wg.Wait()
	for i := range callers {
		require.NoError(t, errs[i])
		require.NotNil(t, meshes[i])
	}
	hit := mustTessellate(t, body, units.Millimeters(cacheTolerance))
	require.True(t, slices.Contains(meshes, hit), "the last completed cold success is the cached hit")
}

func BenchmarkTessellationCacheCold(b *testing.B) {
	for range b.N {
		body := benchRevolve(func(s *sketch.Sketch) {
			c := s.CreatePoint(0, 10)
			s.Fix(c)
			s.CreateCircle(c, 3)
		}, decad.FullRevolution{})
		if _, err := body.Tessellate(units.Millimeters(cacheTolerance)); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkTessellationCacheRepeated(b *testing.B) {
	body := benchRevolve(func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 10)
		s.Fix(c)
		s.CreateCircle(c, 3)
	}, decad.FullRevolution{})
	if _, err := body.Tessellate(units.Millimeters(cacheTolerance)); err != nil {
		b.Fatal(err)
	}
	b.ResetTimer()
	for range b.N {
		if _, err := body.Tessellate(units.Millimeters(cacheTolerance)); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkTessellationCacheAlternatingTolerance(b *testing.B) {
	body := benchRevolve(func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 10)
		s.Fix(c)
		s.CreateCircle(c, 3)
	}, decad.FullRevolution{})
	tols := [2]units.Value{units.Millimeters(cacheTolerance), units.Millimeters(1)}
	for i := range b.N {
		if _, err := body.Tessellate(tols[i%len(tols)]); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkTessellationCacheOBJ(b *testing.B) {
	body := benchRevolve(func(s *sketch.Sketch) {
		c := s.CreatePoint(0, 10)
		s.Fix(c)
		s.CreateCircle(c, 3)
	}, decad.FullRevolution{})
	opts := []decad.OBJOption{decad.WithChordTolerance(units.Millimeters(cacheTolerance))}
	if err := body.OBJ(io.Discard, opts...); err != nil {
		b.Fatal(err)
	}
	b.ResetTimer()
	for range b.N {
		if err := body.OBJ(io.Discard, opts...); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkTessellationCacheLoft(b *testing.B) {
	body := benchLoft()
	if _, err := body.Tessellate(units.Millimeters(cacheTolerance)); err != nil {
		b.Fatal(err)
	}
	b.ResetTimer()
	for range b.N {
		if _, err := body.Tessellate(units.Millimeters(cacheTolerance)); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkTessellationCacheVerify(b *testing.B) {
	doc := decad.New()
	a := benchRodBody(b, doc, 0, 0, 4)
	c := benchRodBody(b, doc, 0, 0, 3)
	placed, err := c.Placed(mustTransform(b, r3.NewVec(0, -4, 2)))
	if err != nil {
		b.Fatal(err)
	}
	_ = placed
	if _, err := doc.Verify(b.Context()); err != nil {
		b.Fatal(err)
	}
	_ = a
	b.ResetTimer()
	for range b.N {
		if _, err := doc.Verify(b.Context()); err != nil {
			b.Fatal(err)
		}
	}
}

var retainedCacheBodies []*decad.Body

func TestTessellationCacheRetainedMemory(t *testing.T) {
	if os.Getenv("DECAD_CACHE_MEM_EXPERIMENT") == "" {
		t.Skip("set DECAD_CACHE_MEM_EXPERIMENT to measure retained cache memory")
	}
	runtime.GC()
	var before runtime.MemStats
	runtime.ReadMemStats(&before)
	const count = 16
	retainedCacheBodies = make([]*decad.Body, count)
	for i := range retainedCacheBodies {
		retainedCacheBodies[i] = cacheTorusBody()
		mustTessellate(t, retainedCacheBodies[i], units.Millimeters(cacheTolerance))
	}
	runtime.GC()
	var after runtime.MemStats
	runtime.ReadMemStats(&after)
	retained := after.HeapAlloc - before.HeapAlloc
	runtime.KeepAlive(retainedCacheBodies)
	t.Logf("bodies=%d heap_alloc_before=%d heap_alloc_after=%d retained_delta=%d retained_per_body=%d heap_inuse=%d", count, before.HeapAlloc, after.HeapAlloc, retained, retained/uint64(count), after.HeapInuse)
}

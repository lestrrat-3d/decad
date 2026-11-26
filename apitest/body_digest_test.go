package apitest_test

import (
	"context"
	"crypto/sha256"
	"encoding/binary"
	"math"
	"strconv"
	"sync"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// digestFiller adds n unrelated bodies to doc, which shifts every producer, level and curve id of what is built
// after it. One filler at least keeps the next operation off producer 0, which the digest keeps raw.
func digestFiller(t *testing.T, doc *decad.Document, n int) {
	t.Helper()
	for i := range n {
		boxBody(t, doc, 50+float64(i)*20, 50, 60+float64(i)*20, 60, 3)
	}
}

// digestPair builds a plate with a through hole and a block that overlaps it, in doc, and returns them.
func digestPair(t *testing.T, doc *decad.Document, width float64) (*decad.Body, *decad.Body) {
	t.Helper()
	plate := boxBody(t, doc, 0, 0, width, 10, 5)
	hole := discBodySymmetric(t, doc, 4, 2, 8)
	drilled, err := decad.Cut(t.Context(), plate, hole)
	require.NoError(t, err)
	block := boxBodyAtZ(t, doc, 6, 2, 9, 8, 2, 6)
	return drilled, block
}

func requireVolumeBits(t *testing.T, want, got *decad.Body) {
	t.Helper()
	w, err := want.Volume()
	require.NoError(t, err)
	g, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, math.Float64bits(w.Value.Base()), math.Float64bits(g.Value.Base()))
}

// TestBodyDigestStableAfterCaches requires a digest unchanged by every call that fills a result-neutral cache:
// Tessellate, Verify, LocatePoint, ContactPair, and an Intersect that commits nothing.
func TestBodyDigestStableAfterCaches(t *testing.T) {
	t.Parallel()
	doc := decad.New()
	digestFiller(t, doc, 1)
	drilled, block := digestPair(t, doc, 20)
	far := boxBodyAtZ(t, doc, 100, 100, 110, 110, 40, 2)

	ctx := t.Context()
	body, err := drilled.Digest(ctx)
	require.NoError(t, err)
	other, err := block.Digest(ctx)
	require.NoError(t, err)
	pair, err := doc.PairDigest(ctx, drilled, block)
	require.NoError(t, err)

	// A fresh walk, not the stored digest, is what the later reads compare against.
	fresh := decad.New()
	digestFiller(t, fresh, 1)
	fd, fb := digestPair(t, fresh, 20)
	freshBody, err := fd.Digest(ctx)
	require.NoError(t, err)
	require.Equal(t, body, freshBody)

	_, err = drilled.Tessellate(ctx, units.Millimeters(0.05))
	require.NoError(t, err)
	_, err = block.Tessellate(ctx, units.Millimeters(0.2))
	require.NoError(t, err)
	_, err = doc.Verify(ctx)
	require.NoError(t, err)
	_, err = drilled.LocatePoint(ctx, r3.NewVec(1, 1, 1), units.Millimeters(0.05))
	require.NoError(t, err)
	_, err = doc.ContactPair(ctx, drilled, far, r3.Identity(), r3.Identity(), contactRequest())
	require.NoError(t, err)
	_, err = decad.Intersect(ctx, drilled, far)
	require.Error(t, err, "the far block does not meet the plate")

	after, err := fd.Digest(ctx)
	require.NoError(t, err)
	require.Equal(t, body, after)
	afterOther, err := fb.Digest(ctx)
	require.NoError(t, err)
	require.Equal(t, other, afterOther)
	// Digest memoizes, so build the same body once more after the caches were filled.
	again := decad.New()
	digestFiller(t, again, 1)
	ad, ab := digestPair(t, again, 20)
	_, err = ad.Tessellate(ctx, units.Millimeters(0.05))
	require.NoError(t, err)
	_, err = decad.Intersect(ctx, ad, boxBodyAtZ(t, again, 100, 100, 110, 110, 40, 2))
	require.Error(t, err)
	cached, err := ad.Digest(ctx)
	require.NoError(t, err)
	require.Equal(t, body, cached, "a digest taken after the caches filled equals one taken before")
	cachedPair, err := again.PairDigest(ctx, ad, ab)
	require.NoError(t, err)
	require.Equal(t, pair, cachedPair)
}

// TestPairDigestRenumberedIDs requires equal digests for one construction built after 1, 2 and 7 filler bodies,
// which renumbers every producer, level and curve id, and the same Intersect outcome for each. A changed size and a
// different gap tolerance give different digests.
func TestPairDigestRenumberedIDs(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	build := func(fillers int, width float64, opts ...decad.DocumentOption) (*decad.Document, *decad.Body, *decad.Body) {
		doc := decad.New(opts...)
		digestFiller(t, doc, fillers)
		a, b := digestPair(t, doc, width)
		return doc, a, b
	}
	baseDoc, baseA, baseB := build(1, 20)
	wantBody, err := baseA.Digest(ctx)
	require.NoError(t, err)
	wantPair, err := baseDoc.PairDigest(ctx, baseA, baseB)
	require.NoError(t, err)
	wantVolume, err := decad.Intersect(ctx, baseA, baseB)
	require.NoError(t, err)

	for _, fillers := range []int{2, 7} {
		doc, a, b := build(fillers, 20)
		gotBody, err := a.Digest(ctx)
		require.NoError(t, err)
		require.Equal(t, wantBody, gotBody, "fillers=%d", fillers)
		gotPair, err := doc.PairDigest(ctx, a, b)
		require.NoError(t, err)
		require.Equal(t, wantPair, gotPair, "fillers=%d", fillers)
		got, err := decad.Intersect(ctx, a, b)
		require.NoError(t, err)
		requireVolumeBits(t, wantVolume, got)
	}

	wide, wa, wb := build(1, 21)
	widePair, err := wide.PairDigest(ctx, wa, wb)
	require.NoError(t, err)
	require.NotEqual(t, wantPair, widePair, "a changed size")

	loose, la, lb := build(1, 20, decad.WithGapTolerance(units.Millimeters(0.5)))
	loosePair, err := loose.PairDigest(ctx, la, lb)
	require.NoError(t, err)
	require.NotEqual(t, wantPair, loosePair, "a different gap tolerance")
	looseBody, err := la.Digest(ctx)
	require.NoError(t, err)
	require.NotEqual(t, wantBody, looseBody, "a different gap tolerance")

	require.Equal(t, 64, len(wantPair.String()))
	reversed, err := baseDoc.PairDigest(ctx, baseB, baseA)
	require.NoError(t, err)
	require.NotEqual(t, wantPair, reversed, "operand order")
}

// TestPairDigestSharedSource requires the pair digest to include what two operands share. Two copies of one source
// and two copies of two identical sources have equal body digests, and the pair digests differ.
func TestPairDigestSharedSource(t *testing.T) {
	t.Parallel()
	ctx := t.Context()

	one := decad.New()
	digestFiller(t, one, 1)
	source := boxBody(t, one, 0, 0, 10, 10, 5)
	c1, err := decad.Copy(ctx, source)
	require.NoError(t, err)
	c2, err := decad.Copy(ctx, source)
	require.NoError(t, err)

	two := decad.New()
	digestFiller(t, two, 1)
	s1 := boxBody(t, two, 0, 0, 10, 10, 5)
	s2 := boxBody(t, two, 0, 0, 10, 10, 5)
	u1, err := decad.Copy(ctx, s1)
	require.NoError(t, err)
	u2, err := decad.Copy(ctx, s2)
	require.NoError(t, err)

	d1, err := c1.Digest(ctx)
	require.NoError(t, err)
	d2, err := c2.Digest(ctx)
	require.NoError(t, err)
	e1, err := u1.Digest(ctx)
	require.NoError(t, err)
	e2, err := u2.Digest(ctx)
	require.NoError(t, err)
	require.Equal(t, d1, e1)
	require.Equal(t, d2, e2)
	require.Equal(t, d1, e2)

	shared, err := one.PairDigest(ctx, c1, c2)
	require.NoError(t, err)
	apart, err := two.PairDigest(ctx, u1, u2)
	require.NoError(t, err)
	require.NotEqual(t, shared, apart)

	// Equal pair structure still gives equal digests: the same shared source in a second document.
	three := decad.New()
	digestFiller(t, three, 3)
	source3 := boxBody(t, three, 0, 0, 10, 10, 5)
	k1, err := decad.Copy(ctx, source3)
	require.NoError(t, err)
	k2, err := decad.Copy(ctx, source3)
	require.NoError(t, err)
	again, err := three.PairDigest(ctx, k1, k2)
	require.NoError(t, err)
	require.Equal(t, shared, again)
}

// meshFingerprint hashes the vertex bits and triangles of b's tessellation at a fixed chord.
func meshFingerprint(t *testing.T, b *decad.Body) [32]byte {
	t.Helper()
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	h := sha256.New()
	var buf [8]byte
	for _, v := range mesh.Vertices() {
		for _, x := range [3]float64{v.X, v.Y, v.Z} {
			binary.LittleEndian.PutUint64(buf[:], math.Float64bits(x))
			h.Write(buf[:])
		}
	}
	for _, tri := range mesh.Triangles() {
		for _, i := range tri {
			binary.LittleEndian.PutUint64(buf[:], uint64(i))
			h.Write(buf[:])
		}
	}
	var out [32]byte
	h.Sum(out[:0])
	return out
}

// TestBodyDigestMatchesMeshFingerprint builds bodies of the kinds a multi-part assembly repeats (drilled plates,
// unions, placed copies, rotated copies) at several history positions and sizes. Two bodies have equal digests exactly
// when their tessellations are bit-identical.
func TestBodyDigestMatchesMeshFingerprint(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	type built struct {
		name string
		body *decad.Body
	}
	var all []built
	for _, fillers := range []int{1, 2, 4} {
		for _, width := range []float64{20, 24} {
			doc := decad.New()
			digestFiller(t, doc, fillers)
			drilled, block := digestPair(t, doc, width)
			joined, err := decad.Union(ctx, boxBody(t, doc, 0, 20, 8, 28, 4), boxBodyAtZ(t, doc, 4, 22, 12, 30, 2, 6))
			require.NoError(t, err)
			moved := translated(t, boxBody(t, doc, 0, 0, 5, 5, 5), 7, 3, 1)
			turned := roundTrip(t, boxBody(t, doc, 0, 0, 6, 6, 6))
			all = append(all,
				built{"drilled/" + strconv.FormatFloat(width, 'f', -1, 64), drilled}, built{"block", block}, built{"joined boxes", joined},
				built{"moved", moved}, built{"turned", turned})
		}
	}
	type key struct {
		digest decad.Digest
		print  [32]byte
	}
	byDigest := map[decad.Digest][32]byte{}
	byPrint := map[[32]byte]decad.Digest{}
	classes := map[key]struct{}{}
	for _, b := range all {
		d, err := b.body.Digest(ctx)
		require.NoError(t, err, b.name)
		p := meshFingerprint(t, b.body)
		if want, ok := byDigest[d]; ok {
			require.Equal(t, want, p, "%s: equal digests, different meshes", b.name)
		}
		if want, ok := byPrint[p]; ok {
			require.Equal(t, want, d, "%s: equal meshes, different digests", b.name)
		}
		byDigest[d], byPrint[p] = p, d
		classes[key{d, p}] = struct{}{}
	}
	// 3 history positions x 2 widths x 5 kinds built 30 bodies in 6 classes: the drilled plate of each width,
	// plus block, union, moved and turned, each the same at every position and width.
	require.Len(t, all, 30)
	require.Len(t, classes, 6)
}

// TestBodyDigestGates covers the refusals of Digest and PairDigest.
func TestBodyDigestGates(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc := decad.New()
	a := boxBody(t, doc, 0, 0, 10, 10, 5)
	b := boxBody(t, doc, 20, 0, 30, 10, 5)
	other := decad.New()
	foreign := boxBody(t, other, 0, 0, 10, 10, 5)

	var nilBody *decad.Body
	_, err := nilBody.Digest(ctx)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.PairDigest(ctx, a, nil)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.PairDigest(ctx, a, a)
	require.ErrorIs(t, err, decad.ErrDegenerate)
	_, err = doc.PairDigest(ctx, a, foreign)
	require.ErrorIs(t, err, decad.ErrForeignBody)
	_, err = doc.PairDigest(ctx, foreign, a)
	require.ErrorIs(t, err, decad.ErrForeignBody)

	canceled, cancel := context.WithCancel(ctx)
	cancel()
	_, err = b.Digest(canceled)
	require.ErrorIs(t, err, canceled.Err())

	// A retired body keeps a digest.
	moved := translated(t, a, 1, 0, 0)
	_, err = a.Digest(ctx)
	require.NoError(t, err)
	_, err = doc.PairDigest(ctx, a, moved)
	require.NoError(t, err)
}

// TestBodyDigestIncludesHistory requires bodies of equal geometry and different history to differ: the history
// routes of Intersect read the operations that produced a body.
func TestBodyDigestIncludesHistory(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc := decad.New()
	digestFiller(t, doc, 1)
	source := boxBody(t, doc, 0, 0, 10, 10, 5)
	copied, err := decad.Copy(ctx, source)
	require.NoError(t, err)
	fresh := boxBody(t, doc, 0, 0, 10, 10, 5)

	require.Equal(t, meshFingerprint(t, copied), meshFingerprint(t, fresh), "the fixture needs equal geometry")
	dc, err := copied.Digest(ctx)
	require.NoError(t, err)
	df, err := fresh.Digest(ctx)
	require.NoError(t, err)
	require.NotEqual(t, dc, df)
}

// TestBodyDigestConcurrent runs Digest and PairDigest on one pair from several goroutines while Tessellate and
// Verify fill the caches the digest skips. Every goroutine reads the same digest. Run under -race.
func TestBodyDigestConcurrent(t *testing.T) {
	t.Parallel()
	ctx := t.Context()
	doc := decad.New()
	digestFiller(t, doc, 1)
	a, b := digestPair(t, doc, 20)

	const workers = 8
	bodies := make([]decad.Digest, workers)
	pairs := make([]decad.Digest, workers)
	errs := make([]error, workers)
	var wg sync.WaitGroup
	for i := range workers {
		wg.Add(1)
		go func() {
			defer wg.Done()
			switch i % 4 {
			case 0:
				_, errs[i] = a.Tessellate(ctx, units.Millimeters(0.05))
			case 1:
				_, errs[i] = doc.Verify(ctx)
			}
			if errs[i] != nil {
				return
			}
			bodies[i], errs[i] = a.Digest(ctx)
			if errs[i] != nil {
				return
			}
			pairs[i], errs[i] = doc.PairDigest(ctx, a, b)
		}()
	}
	wg.Wait()
	for i := range workers {
		require.NoError(t, errs[i])
		require.Equal(t, bodies[0], bodies[i])
		require.Equal(t, pairs[0], pairs[i])
	}
}

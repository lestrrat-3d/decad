package revolvemesh

import (
	"math/rand/v2"
	"testing"

	"github.com/stretchr/testify/require"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// legacyAxisCache and the legacy* functions below are the by-value
// implementations the pointer signatures replaced, copied verbatim apart from
// their names. They are the reference the pointer forms must match exactly.
type legacyAxisCache struct {
	normal   []RevolveSepAxis
	normalOK []bool
	corners  [][3]revolveCornerAxes
	cornerOK [][3]bool
}

func newLegacyAxisCache(n int) *legacyAxisCache {
	return &legacyAxisCache{
		normal:   make([]RevolveSepAxis, n),
		normalOK: make([]bool, n),
		corners:  make([][3]revolveCornerAxes, n),
		cornerOK: make([][3]bool, n),
	}
}

func (c *legacyAxisCache) normalAxis(i int, a RevolveAuditTri, delta float64) RevolveSepAxis {
	if !c.normalOK[i] {
		c.normal[i] = RevolveNormalAxis(a, delta)
		c.normalOK[i] = true
	}
	return c.normal[i]
}

func (c *legacyAxisCache) corner(i int, a RevolveAuditTri, at int, delta float64) *revolveCornerAxes {
	if !c.cornerOK[i][at] {
		off := RevolveCornerOffsets(a, at)
		out := &c.corners[i][at]
		out.fan[0] = RevolveEdgeFanAxis(a, off[0], delta)
		out.fan[1] = RevolveEdgeFanAxis(a, off[1], delta)
		chord := RevolveOffsetOf(proofarith.DvSub(off[0].V, off[1].V), RevIvSubVec(off[0].F, off[1].F))
		out.chord = RevolveEdgeFanAxis(a, chord, delta)
		c.cornerOK[i][at] = true
	}
	return &c.corners[i][at]
}

func legacyVertexIsolated(a RevolveAuditTri, triA [3]int, b RevolveAuditTri, triB [3]int, shared int, delta float64,
	cache *legacyAxisCache, ia int) bool {
	e := proofbound.ProductUpper(2, delta)
	ai := TriangleVertexSlot(triA, shared)
	bi := TriangleVertexSlot(triB, shared)
	if ai < 0 || bi < 0 {
		return false
	}
	aOff := RevolveCornerOffsets(a, ai)
	bOff := RevolveCornerOffsets(b, bi)

	// signed reads the common sign of the listed offsets, or reports that this
	// candidate cannot sign them all.
	signed := func(ax RevolveSepAxis, offs []RevolveOffset) (int, bool) {
		side := 0
		for _, o := range offs {
			s, ok := ax.SideOf(o, e)
			if !ok {
				return 0, false
			}
			if side == 0 {
				side = s
			} else if side != s {
				return 0, false
			}
		}
		return side, true
	}
	try := func(ax RevolveSepAxis, check []RevolveOffset) bool {
		side, ok := signed(ax, check)
		if !ok {
			return false
		}
		want, ok := signed(ax, bOff[:])
		if !ok || want == 0 {
			return false
		}
		return side == 0 || side != want
	}
	normal := func() RevolveSepAxis {
		if cache != nil {
			return cache.normalAxis(ia, a, delta)
		}
		return RevolveNormalAxis(a, delta)
	}
	if try(normal(), nil) {
		return true
	}
	var corner *revolveCornerAxes
	if cache != nil {
		corner = cache.corner(ia, a, ai, delta)
	}
	for k := range 2 {
		var ax RevolveSepAxis
		if corner != nil {
			ax = corner.fan[k]
		} else {
			ax = RevolveEdgeFanAxis(a, aOff[k], delta)
		}
		if try(ax, []RevolveOffset{aOff[1-k]}) {
			return true
		}
	}
	var chordAxis RevolveSepAxis
	if corner != nil {
		chordAxis = corner.chord
	} else {
		chord := RevolveOffsetOf(proofarith.DvSub(aOff[0].V, aOff[1].V), RevIvSubVec(aOff[0].F, aOff[1].F))
		chordAxis = RevolveEdgeFanAxis(a, chord, delta)
	}
	if try(chordAxis, aOff[:]) {
		return true
	}
	// The edge-pair family. g = eA × eB zeroes both eA and eB identically, so
	// the plane holds one edge of each triangle for the WHOLE family rather
	// than at the stored coordinates alone. That leaves a with two corners on
	// the plane and one strictly off it, and b likewise, so a sits in one
	// closed half-space and b in the other and their intersection lies in the
	// plane. A triangle with two corners on a plane and its third strictly off
	// meets that plane in exactly the closed segment between the two, so the
	// intersection is contained in eA ∩ eB — two segments from the shared
	// corner that a non-zero g makes non-parallel, and which therefore meet
	// only at that corner.
	//
	// g stays non-zero over the whole family without a separate test: a member
	// whose g vanished would read zero on both remaining corners, and sideOf
	// has already proven each of them strictly outside its own perturbation
	// allowance, which bounds exactly how far that reading can move. length
	// bounds |eA × eB| by the product of the two factor lengths and drift is
	// PerturbBilinearAllow over the two factors sliding by e, so the charge is
	// the one every other candidate here makes. A reading inside its allowance
	// stays UNDECIDED and the candidate is skipped, never admitted.
	for k := range 2 {
		for m := range 2 {
			g := proofarith.DvCross(aOff[k].V, bOff[m].V)
			if proofarith.DvIsZero(g) {
				continue
			}
			ax := RevolveSepAxis{
				G:      g,
				F:      RevIvCross(aOff[k].F, bOff[m].F),
				Length: proofbound.ProductUpper(aOff[k].Length, bOff[m].Length),
				Drift:  PerturbBilinearAllow(aOff[k].Length, bOff[m].Length, e, e),
			}
			sa, okA := ax.SideOf(aOff[1-k], e)
			sb, okB := ax.SideOf(bOff[1-m], e)
			if okA && okB && sa != sb {
				return true
			}
		}
	}
	return false
}

// pointerEquivTris builds a random triangle pair meeting at vertex 0 of a
// small vertex set. Coordinates come from a coarse grid half the time so
// coplanar and degenerate pairs occur, and from the continuum otherwise.
func pointerEquivTris(t *testing.T, rng *rand.Rand) (a, b RevolveAuditTri, triA, triB [3]int, shared int) {
	t.Helper()
	coord := func(coarse bool) float64 {
		if coarse {
			return float64(rng.IntN(5)-2) * 0.5
		}
		return rng.Float64()*2 - 1
	}
	coarse := rng.IntN(2) == 0
	verts := make([]r3.Vec, 5)
	for i := range verts {
		verts[i] = r3.Vec{X: coord(coarse), Y: coord(coarse), Z: coord(coarse)}
	}
	// Rotating a triangle's slots moves the shared corner to every slot.
	rot := func(tri [3]int, k int) [3]int {
		return [3]int{tri[k%3], tri[(k+1)%3], tri[(k+2)%3]}
	}
	triA = rot([3]int{0, 1, 2}, rng.IntN(3))
	triB = rot([3]int{0, 3, 4}, rng.IntN(3))
	var okA, okB bool
	a, okA = NewRevolveAuditTri(verts, triA)
	b, okB = NewRevolveAuditTri(verts, triB)
	require.True(t, okA && okB)
	return a, b, triA, triB, 0
}

func TestRevolveVertexIsolatedPointerFormMatchesByValue(t *testing.T) {
	rng := rand.New(rand.NewPCG(7, 11))
	var trues, falses int
	for iter := range 400 {
		a, b, triA, triB, shared := pointerEquivTris(t, rng)
		for _, delta := range []float64{0, 1e-9, 0.25} {
			want := legacyVertexIsolated(a, triA, b, triB, shared, delta, nil, 0)
			require.Equal(t, want, revolveVertexIsolated(&a, triA, &b, triB, shared, delta, nil, 0), "iter %d", iter)
			require.Equal(t, want, RevolveVertexIsolated(a, triA, b, triB, shared, delta), "iter %d", iter)
			if want {
				trues++
			} else {
				falses++
			}

			// Both orders through one cache per implementation, as the audit does.
			oldCache, newCache := newLegacyAxisCache(2), newRevolveAxisCache(2)
			for round := range 2 {
				require.Equal(t,
					legacyVertexIsolated(a, triA, b, triB, shared, delta, oldCache, 0),
					revolveVertexIsolated(&a, triA, &b, triB, shared, delta, newCache, 0), "iter %d round %d", iter, round)
				require.Equal(t,
					legacyVertexIsolated(b, triB, a, triA, shared, delta, oldCache, 1),
					revolveVertexIsolated(&b, triB, &a, triA, shared, delta, newCache, 1), "iter %d round %d", iter, round)
			}
			for at := range 3 {
				require.Equal(t, oldCache.normalAxis(0, a, delta), newCache.normalAxis(0, &a, delta))
				require.Equal(t, *oldCache.corner(0, a, at, delta), *newCache.corner(0, &a, at, delta))
			}
		}
	}
	require.Positive(t, trues)
	require.Positive(t, falses)
}

// A pair passing the same triangle for both operands, as one pointer twice,
// must read like two copies of one value: the functions only read it.
func TestRevolveVertexIsolatedPointerFormSameOperand(t *testing.T) {
	rng := rand.New(rand.NewPCG(3, 5))
	for iter := range 200 {
		a, _, triA, _, shared := pointerEquivTris(t, rng)
		for _, delta := range []float64{0, 1e-9, 0.25} {
			before := a
			want := legacyVertexIsolated(a, triA, a, triA, shared, delta, nil, 0)
			require.Equal(t, want, revolveVertexIsolated(&a, triA, &a, triA, shared, delta, nil, 0), "iter %d", iter)
			require.Equal(t, before, a, "iter %d: operand mutated", iter)
		}
	}
}

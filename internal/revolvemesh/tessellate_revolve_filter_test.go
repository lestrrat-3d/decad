package revolvemesh

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type filterOracle struct {
	readings int
	settled  int
}

func (o *filterOracle) sides(t *testing.T, axes []RevolveSepAxis, offs []RevolveOffset, drift float64) {
	t.Helper()
	for ai, ax := range axes {
		for oi, off := range offs {
			o.readings++
			allow := PerturbBilinearAllow(ax.Length, off.Length, ax.Drift, drift)
			if _, ok := RevIvSide(RevIvDot(ax.F, off.F), allow); ok {
				o.settled++
			}
			gotSide, gotOK := ax.SideOf(off, drift)
			wantSide, wantOK := ax.SideOfExact(off.V, off.Length, drift)
			require.Equal(t, wantOK, gotOK, `axis %d, offset %d`, ai, oi)
			require.Equal(t, wantSide, gotSide, `axis %d, offset %d`, ai, oi)
		}
	}
}

func (o *filterOracle) pair(t *testing.T, data []RevolveAuditTri, tris [][3]int, i, j int, delta float64) {
	t.Helper()
	a, b := data[i], data[j]
	shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
	if count == 0 {
		o.readings++
		if RevolveSeparatedFloat(a, b, delta) {
			o.settled++
		}
		require.Equal(t, RevolveSeparatedExact(a, b, delta), RevolveSeparated(a, b, delta), `facets %d and %d`, i, j)
		return
	}
	e := proofbound.ProductUpper(2, delta)
	ai := TriangleVertexSlot(tris[i], shared[0])
	bi := TriangleVertexSlot(tris[j], shared[0])
	aOff, bOff := RevolveCornerOffsets(a, ai), RevolveCornerOffsets(b, bi)
	chord := RevolveOffsetOf(proof.DvSub(aOff[0].V, aOff[1].V), RevIvSubVec(aOff[0].F, aOff[1].F))
	axes := []RevolveSepAxis{
		RevolveNormalAxis(a, delta), RevolveNormalAxis(b, delta),
		RevolveEdgeFanAxis(a, aOff[0], delta), RevolveEdgeFanAxis(a, aOff[1], delta),
		RevolveEdgeFanAxis(a, chord, delta), RevolveRejectionAxis(aOff[0], aOff[1], delta),
	}
	for k := range 2 {
		for m := range 2 {
			axes = append(axes, RevolveSepAxis{
				G: proof.DvCross(aOff[k].V, bOff[m].V), F: RevIvCross(aOff[k].F, bOff[m].F),
				Length: proofbound.ProductUpper(aOff[k].Length, bOff[m].Length),
				Drift:  PerturbBilinearAllow(aOff[k].Length, bOff[m].Length, e, e),
			})
		}
	}
	o.sides(t, axes, append(append([]RevolveOffset{chord}, aOff[:]...), bOff[:]...), e)
}

func TestRevolveAuditPreTestMatchesExact(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1, 2))
	jitter := func(scale float64) r3.Vec {
		return r3.NewVec((rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale)
	}
	var oracle filterOracle
	for range 4000 {
		base := jitter(20)
		verts := []r3.Vec{base, base.Add(jitter(2)), base.Add(jitter(2)), base, base.Add(jitter(2)), base.Add(jitter(2))}
		if rng.IntN(2) == 0 {
			verts[3] = base.Add(jitter(0.01))
		}
		tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
		if verts[3] == verts[0] {
			tris[1] = [3]int{0, 4, 5}
		}
		a, okA := NewRevolveAuditTri(verts, tris[0])
		b, okB := NewRevolveAuditTri(verts, tris[1])
		if !okA || !okB {
			continue
		}
		delta := 1e-6 * float64(rng.IntN(20000))
		oracle.pair(t, []RevolveAuditTri{a, b}, tris, 0, 1, delta)
	}
	require.Greater(t, oracle.settled, 0, `the float pre-test settles some randomized readings`)
}

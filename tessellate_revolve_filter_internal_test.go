package decad

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// filterOracle counts how often the float pre-test settled a reading, so a
// test can show it ran rather than always deferring to the exact arithmetic.
type filterOracle struct {
	readings, settled int
}

// sides compares sideOf against sideOfExact for every axis and offset pair.
func (o *filterOracle) sides(t *testing.T, axes []revolveSepAxis, offs []revolveOffset, drift float64) {
	t.Helper()
	for ai, ax := range axes {
		for oi, off := range offs {
			o.readings++
			allow := perturbBilinearAllow(ax.length, off.length, ax.drift, drift)
			if _, ok := revIvSide(revIvDot(ax.f, off.f), allow); ok {
				o.settled++
			}
			gotSide, gotOK := ax.sideOf(off, drift)
			wantSide, wantOK := ax.sideOfExact(off.v, off.length, drift)
			require.Equal(t, wantOK, gotOK, `axis %d, offset %d`, ai, oi)
			require.Equal(t, wantSide, gotSide, `axis %d, offset %d`, ai, oi)
		}
	}
}

// pair runs every candidate axis the vertex and edge isolation proofs build
// for triangles i and j, plus the separating-axis walk, through both the
// pre-tested and the exact predicates.
func (o *filterOracle) pair(t *testing.T, data []revolveAuditTri, tris [][3]int, i, j int, delta float64) {
	t.Helper()
	a, b := data[i], data[j]
	shared, count := sharedVertexIndices(tris[i], tris[j])
	if count == 0 {
		o.readings++
		if revolveSeparatedFloat(a, b, delta) {
			o.settled++
		}
		require.Equal(t, revolveSeparatedExact(a, b, delta), revolveSeparated(a, b, delta), `facets %d and %d`, i, j)
		return
	}
	e := proofbound.ProductUpper(2, delta)
	ai := triangleVertexSlot(tris[i], shared[0])
	bi := triangleVertexSlot(tris[j], shared[0])
	aOff, bOff := revolveCornerOffsets(a, ai), revolveCornerOffsets(b, bi)
	chord := revolveOffsetOf(proofarith.DvSub(aOff[0].v, aOff[1].v), revIvSubVec(aOff[0].f, aOff[1].f))
	axes := []revolveSepAxis{
		revolveNormalAxis(a, delta), revolveNormalAxis(b, delta),
		revolveEdgeFanAxis(a, aOff[0], delta), revolveEdgeFanAxis(a, aOff[1], delta),
		revolveEdgeFanAxis(a, chord, delta),
		revolveRejectionAxis(aOff[0], aOff[1], delta),
	}
	for k := range 2 {
		for m := range 2 {
			axes = append(axes, revolveSepAxis{
				g:      proofarith.DvCross(aOff[k].v, bOff[m].v),
				f:      revIvCross(aOff[k].f, bOff[m].f),
				length: proofbound.ProductUpper(aOff[k].length, bOff[m].length),
				drift:  perturbBilinearAllow(aOff[k].length, bOff[m].length, e, e),
			})
		}
	}
	o.sides(t, axes, append(append([]revolveOffset{chord}, aOff[:]...), bOff[:]...), e)
}

// TestRevolveAuditPreTestMatchesExact runs every facet pair of a real revolve
// mesh, and a set of random triangle pairs placed near the readings'
// thresholds, through the float pre-test and the exact predicates, at
// displacements from zero to far past the facets' own size. Every reading
// must agree. The pre-test must also settle most readings, or it is not doing
// its job.
func TestRevolveAuditPreTestMatchesExact(t *testing.T) {
	t.Parallel()
	doc := New()
	frustum := internalFrustumBody(t, doc, r3.NewVec(0.3, -0.7, 0.1), r3.NewVec(4, 2, 5), 1.5, 0.6)
	m, err := tessellateContext(t.Context(), frustum, units.Millimeters(0.05), VerifyNone)
	require.NoError(t, err)
	var o filterOracle
	for _, delta := range []float64{0, 1e-15, 1e-9, 1e-4, 0.05, 0.5} {
		data, err := requireRevolveFacetAreas(proofbound.NewWorkBudget(t.Context()), m.vertices, m.triangles, 0)
		require.NoError(t, err)
		for i := range data {
			for j := i + 1; j < len(data); j++ {
				o.pair(t, data, m.triangles, i, j, delta)
			}
		}
	}
	require.Greater(t, o.settled*10, o.readings*9, `the pre-test settles %d of %d readings`, o.settled, o.readings)

	// Random triangles sharing a vertex, or sharing none, a short way apart,
	// read at displacements on both sides of their gaps.
	rng := rand.New(rand.NewPCG(1, 2))
	jitter := func(scale float64) r3.Vec {
		return r3.NewVec((rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale, (rng.Float64()-0.5)*scale)
	}
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
		a, okA := newRevolveAuditTri(verts, tris[0])
		b, okB := newRevolveAuditTri(verts, tris[1])
		if !okA || !okB {
			continue
		}
		delta := 1e-6 * float64(rng.IntN(20000))
		o.pair(t, []revolveAuditTri{a, b}, tris, 0, 1, delta)
	}
}

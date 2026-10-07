package decad

import (
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/tessellation"

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
func (o *filterOracle) sides(t *testing.T, axes []revolvemesh.RevolveSepAxis, offs []revolvemesh.RevolveOffset, drift float64) {
	t.Helper()
	for ai, ax := range axes {
		for oi, off := range offs {
			o.readings++
			allow := revolvemesh.PerturbBilinearAllow(ax.Length, off.Length, ax.Drift, drift)
			if _, ok := revolvemesh.RevIvSide(revolvemesh.RevIvDot(ax.F, off.F), allow); ok {
				o.settled++
			}
			gotSide, gotOK := ax.SideOf(off, drift)
			wantSide, wantOK := ax.SideOfExact(off.V, off.Length, drift)
			require.Equal(t, wantOK, gotOK, `axis %d, offset %d`, ai, oi)
			require.Equal(t, wantSide, gotSide, `axis %d, offset %d`, ai, oi)
		}
	}
}

// pair runs every candidate axis the vertex and edge isolation proofs build
// for triangles i and j, plus the separating-axis walk, through both the
// pre-tested and the exact predicates.
func (o *filterOracle) pair(t *testing.T, data []revolvemesh.RevolveAuditTri, tris [][3]int, i, j int, delta float64) {
	t.Helper()
	a, b := data[i], data[j]
	shared, count := tessellation.SharedVertexIndices(tris[i], tris[j])
	if count == 0 {
		o.readings++
		if revolvemesh.RevolveSeparatedFloat(a, b, delta) {
			o.settled++
		}
		require.Equal(t, revolvemesh.RevolveSeparatedExact(a, b, delta), revolvemesh.RevolveSeparated(a, b, delta), `facets %d and %d`, i, j)
		return
	}
	e := proofbound.ProductUpper(2, delta)
	ai := revolvemesh.TriangleVertexSlot(tris[i], shared[0])
	bi := revolvemesh.TriangleVertexSlot(tris[j], shared[0])
	aOff, bOff := revolvemesh.RevolveCornerOffsets(a, ai), revolvemesh.RevolveCornerOffsets(b, bi)
	chord := revolvemesh.RevolveOffsetOf(proofarith.DvSub(aOff[0].V, aOff[1].V), revolvemesh.RevIvSubVec(aOff[0].F, aOff[1].F))
	axes := []revolvemesh.RevolveSepAxis{
		revolvemesh.RevolveNormalAxis(a, delta), revolvemesh.RevolveNormalAxis(b, delta),
		revolvemesh.RevolveEdgeFanAxis(a, aOff[0], delta), revolvemesh.RevolveEdgeFanAxis(a, aOff[1], delta),
		revolvemesh.RevolveEdgeFanAxis(a, chord, delta),
		revolvemesh.RevolveRejectionAxis(aOff[0], aOff[1], delta),
	}
	for k := range 2 {
		for m := range 2 {
			axes = append(axes, revolvemesh.RevolveSepAxis{
				G:      proofarith.DvCross(aOff[k].V, bOff[m].V),
				F:      revolvemesh.RevIvCross(aOff[k].F, bOff[m].F),
				Length: proofbound.ProductUpper(aOff[k].Length, bOff[m].Length),
				Drift:  revolvemesh.PerturbBilinearAllow(aOff[k].Length, bOff[m].Length, e, e),
			})
		}
	}
	o.sides(t, axes, append(append([]revolvemesh.RevolveOffset{chord}, aOff[:]...), bOff[:]...), e)
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
		data, err := revolvemesh.RequireRevolveFacetAreas(proofbound.NewWorkBudget(t.Context()), m.vertices, m.triangles, 0)
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
		a, okA := revolvemesh.NewRevolveAuditTri(verts, tris[0])
		b, okB := revolvemesh.NewRevolveAuditTri(verts, tris[1])
		if !okA || !okB {
			continue
		}
		delta := 1e-6 * float64(rng.IntN(20000))
		o.pair(t, []revolvemesh.RevolveAuditTri{a, b}, tris, 0, 1, delta)
	}
}

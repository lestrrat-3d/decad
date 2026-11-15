package retainedblend

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The band of flatBand lies in the plane z = 0: 0 ≤ x ≤ 1 and 0 ≤ y (the end
// plane), every held vertex bounded by 0.001 and the rounding 0.001, so a point
// within 0.002 of a plane is undecided on it. A point within that of an end
// plane that no other plane drops is read by the sign of its exact offset, and
// the offset is charged into its bound (retained-edge-blend §4.3).

// charged is bound grown by the offset d, as signRead charges it where the wall meets
// the end plane square and the slack is zero.
func charged(bound, d float64) float64 { return proofbound.AbsSumUpper(bound, d) }

// TestSignReadChargesHeldVertex reads a held vertex within its bound of the end
// plane by its sign and raises its bound by the offset. Omit the charge and the
// bound assertions fail.
func TestSignReadChargesHeldVertex(t *testing.T) {
	t.Parallel()

	for _, c := range []struct {
		name string
		off  float64
	}{
		{"inside by 0.0015", 0.0015},
		{"outside by 0.0015", -0.0015},
		{"inside by a float step's worth", 3e-16},
	} {
		off := c.off
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			// Vertex 0 lies off the end plane y = 0 by off and inside x ∈ [0, 1]: no
			// other plane drops it, so no order of the planes decides it.
			w, b, f := flatBand(r3.NewVec(0.5, off, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
			cuts, err := w.cutBand(f, b)
			require.NoError(t, err)

			want := charged(0.001, math.Abs(off))
			require.GreaterOrEqual(t, w.beta[0], want, "the offset is charged into the held bound")
			require.InDelta(t, want, w.beta[0], 1e-14)
			require.Greater(t, w.beta[0], 0.001)
			require.Equal(t, w.beta[0], w.raised, "a raised held bound is reported")
			require.Equal(t, 0.001, w.beta[1], "a vertex that no reading undecided keeps its bound")
			require.Equal(t, 0.001, w.beta[2])

			if off > 0 {
				// Inside the band on the side of its offset: the whole facet is removed.
				require.Equal(t, map[int]bool{0: true, 1: true, 2: true}, cuts.removed)
				require.Empty(t, w.out)
				return
			}
			// Outside it, on the other side: the vertex stays, and so does the sliver
			// of the facet outside y ≥ 0 that it cuts off.
			require.Equal(t, map[int]bool{1: true, 2: true}, cuts.removed)
			require.NotEmpty(t, w.out)
		})
	}
}

// TestSignReadChargesCutPoint reads a cut point within its bound of the end plane
// by its sign and charges the offset into the point's own bound: the cuts of x = 0
// and x = 1 on a facet edge 0.0015 off the end plane.
func TestSignReadChargesCutPoint(t *testing.T) {
	t.Parallel()

	w, b, f := flatBand(r3.NewVec(-0.5, 0.0015, 0), r3.NewVec(1.5, 0.0015, 0), r3.NewVec(0.5, 3, 0))
	cuts, err := w.cutBand(f, b)
	require.NoError(t, err)
	require.Equal(t, map[int]bool{2: true}, cuts.removed)

	// The cuts the plane made: facet bound 0.001 plus the rounding 0 plus the
	// grid's 0.001, then the offset.
	made := 0
	for id := 3; id < len(w.verts); id++ {
		p := w.verts[id]
		if math.Abs(p.Y-0.0015) > 1e-12 {
			continue
		}
		made++
		require.InDelta(t, 0.0035, w.beta[id], 1e-12, "the cut at x = %v", p.X)
	}
	require.Equal(t, 2, made)
	require.Equal(t, 0.001, w.beta[0], "the corners outside the band keep their bound")
	require.Equal(t, 0.001, w.beta[1])
}

// TestSignReadOnlyWhereReadingRefuses reads nothing by sign that another plane or
// the plain reading decides: a corner another plane drops, a corner clear of the
// plane and a corner on it by provenance keep their bounds, and so do the cuts
// that need no sign.
func TestSignReadOnlyWhereReadingRefuses(t *testing.T) {
	t.Parallel()

	t.Run("a corner another plane drops is not charged", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(-0.5, 0, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.6, 4, 0))
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		for id, beta := range w.beta {
			require.Equal(t, 0.001, math.Min(beta, 0.001), "vertex %d", id)
			if id < 3 {
				require.Equal(t, 0.001, beta, "held vertex %d", id)
			}
		}
		require.Zero(t, w.raised)
	})
	t.Run("a corner on the plane by provenance is not charged", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(0.5, 0, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		b.ends[0].on = map[int]bool{0: true}
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		require.Equal(t, []float64{0.001, 0.001, 0.001}, w.beta)
		require.Zero(t, w.raised)
	})
	t.Run("a corner clear of the plane is not charged", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(0.5, 0.01, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		_, err := w.cutBand(f, b)
		require.NoError(t, err)
		require.Equal(t, []float64{0.001, 0.001, 0.001}, w.beta)
		require.Zero(t, w.raised)
	})
}

// TestSignReadRefusals keeps every refusal that is not an end plane's: a point
// within its bound of the main plane or of a further plane is undecided still,
// and so is any point of an angle-main band, which has no exact main plane.
func TestSignReadRefusals(t *testing.T) {
	t.Parallel()

	t.Run("a corner two planes leave undecided", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(0.0005, 0.0005, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		_, err := w.cutBand(f, b)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "a vertex of the wall lies on an end face's plane")
	})
	t.Run("a corner within its bound of the main plane", func(t *testing.T) {
		t.Parallel()
		w, b, f := flatBand(r3.NewVec(1.0015, 1, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		_, err := w.cutBand(f, b)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Equal(t, 0.001, w.beta[0], "nothing is charged for a plane that is not an end plane's")
	})
	t.Run("a band without an exact main plane reads no sign", func(t *testing.T) {
		t.Parallel()
		w, b, _ := flatBand(r3.NewVec(0.5, 0.0015, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
		b.main = nil
		b.w = w
		b.side = make([]map[int]int, bandOther+len(b.others))
		for q := range b.side {
			b.side[q] = map[int]int{}
		}
		poly := []bandPt{w.heldPt(0), w.heldPt(1), w.heldPt(2)}
		b.sign = true
		_, err := b.clip(poly, bandEnd0, 0.001)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Equal(t, 0.001, w.beta[0])
		b.active = []int{bandEnd0}
		_, ok := b.clipBySign(poly, b.active, 0.001)
		require.Equal(t, -1, ok)
	})
}

// TestChargeShared makes a vertex that an earlier cut of the same held edge made
// carry the larger bound of the two copies of the point.
func TestChargeShared(t *testing.T) {
	t.Parallel()

	w := &work{verts: []r3.Vec{{}}, beta: []float64{0.002}}
	w.chargeShared(0, &bandPt{bound: 0.0035})
	require.Equal(t, 0.002, w.beta[0], "a point no sign read charged changes nothing")
	w.chargeShared(0, &bandPt{bound: 0.0035, signed: [2]int8{3, 0}})
	require.Equal(t, 0.0035, w.beta[0])
	w.chargeShared(0, &bandPt{bound: 0.003, signed: [2]int8{0, 1}})
	require.Equal(t, 0.0035, w.beta[0], "the bound never shrinks")
}

// TestSignReadChargesSlack charges the route's arithmetic slack with the offset,
// which covers the rounding of the offset itself.
func TestSignReadChargesSlack(t *testing.T) {
	t.Parallel()

	w, b, f := flatBand(r3.NewVec(0.5, 0.0015, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
	b.slack = 0.0005
	_, err := w.cutBand(f, b)
	require.NoError(t, err)
	require.InDelta(t, 0.001+0.0015+0.0005, w.beta[0], 1e-12)
}

// TestCutOfHeldEdgeReadsInputBounds makes the cut of an edge by the end plane take its
// ends' input bounds, not the raises a sign reading left in the work. The edge's two
// corners lie 0.0015 either side of the plane and are first read by it in the crossing,
// after the cuts of x = 0 and x = 1 made points on the edge that are read first. The
// corners carry the charge (0.0025), the cut on y = 0 only its facet's input bound and
// the grid's 0.001: one charge, made on its own offset, and none inherited from the
// corners. Take the work's bounds in the crossing and the cut reads 0.0035.
func TestCutOfHeldEdgeReadsInputBounds(t *testing.T) {
	t.Parallel()

	w, b, f := flatBand(r3.NewVec(-0.5, -0.0015, 0), r3.NewVec(1.5, 0.0015, 0), r3.NewVec(0.5, 3, 0))
	_, err := w.cutBand(f, b)
	require.NoError(t, err)

	require.InDelta(t, 0.0025, w.beta[0], 1e-12, "the corners are charged")
	require.InDelta(t, 0.0025, w.beta[1], 1e-12)
	found := false
	for id := 3; id < len(w.verts); id++ {
		if p := w.verts[id]; math.Abs(p.Y) > 1e-12 || math.Abs(p.X-0.5) > 1e-12 {
			continue
		}
		found = true
		require.InDelta(t, 0.002, w.beta[id], 1e-12, "the cut of the edge by the end plane")
	}
	require.True(t, found)
}

// TestCutFacetBoundReadsInputBounds makes the cut of a second facet take its facet
// bound from the input bounds of its corners although the first facet's reading raised
// the corner they share. Facet 0 is wholly inside the band and charges vertex 0, which
// lies 0.0015 from the end plane; facet 1 shares vertex 0 and is cut by the main plane
// x = 1. Take the work's bounds for the facet and the cut reads 0.0035.
func TestCutFacetBoundReadsInputBounds(t *testing.T) {
	t.Parallel()

	w, b, f := flatBand(r3.NewVec(0.5, 0.0015, 0), r3.NewVec(0.6, 3, 0), r3.NewVec(0.7, 4, 0), r3.NewVec(1.5, 3, 0))
	w.fp.Tris = [][3]int{{0, 1, 2}, {0, 2, 3}}
	w.fp.FaceOf = []int{0, 0}
	f.facets = []int{0, 1}
	_, err := w.cutBand(f, b)
	require.NoError(t, err)
	require.InDelta(t, 0.0025, w.beta[0], 1e-12, "facet 0 charges the shared corner")

	cuts := 0
	for id := 4; id < len(w.verts); id++ {
		if math.Abs(w.verts[id].X-1) > 1e-12 {
			continue
		}
		cuts++
		require.InDelta(t, 0.002, w.beta[id], 1e-12, "the cut of the main plane at y = %v", w.verts[id].Y)
	}
	require.Positive(t, cuts)
}

// TestInputBeta reads a held vertex's input bound, and a vertex the work made with the
// bound it was made with.
func TestInputBeta(t *testing.T) {
	t.Parallel()

	w := &work{fp: facetedrecord.Payload{VertexBound: []float64{0.001, 0.002}}, beta: []float64{0.005, 0.002, 0.003}}
	require.Equal(t, 0.001, w.inputBeta(0), "a raise is no part of the input claim")
	require.Equal(t, 0.002, w.inputBeta(1))
	require.Equal(t, 0.003, w.inputBeta(2), "a vertex the work made")
	require.Equal(t, 0.002, w.inputFacetBound([3]int{0, 1, 0}))
	require.Equal(t, 0.003, w.inputFacetBound([3]int{0, 1, 2}))
}

// tiltedBand is flatBand with the end plane turned about x so that the wall z = 0
// meets it at the angle whose sine is s: n = (0, s, √(1−s²)).
func tiltedBand(s float64, verts ...r3.Vec) (*work, *wallBand, faceLoops) {
	w, b, f := flatBand(verts...)
	b.ends[0] = &bandEnd{g: plane{n: r3.NewVec(0, s, math.Sqrt(1-s*s)), o: r3.Vec{}}, sign: 1,
		sine: func(float64) float64 { return s }, planarWall: true}
	return w, b, f
}

// keptDistance is the distance from p to the kept part {n·x ≤ 0} of the plane
// z = h, a true wall that lies h above the held facet plane z = 0 (below it for h
// negative). Every held bound of 0.001 allows h = ±0.001, so each is a world the
// held claims permit.
func keptDistance(p r3.Vec, s, h float64) float64 {
	c := math.Sqrt(1 - s*s)
	q := r3.NewVec(p.X, p.Y, h)
	if s*q.Y+c*q.Z <= 0 {
		return math.Abs(p.Z - h)
	}
	ys := -c * h / s
	return math.Hypot(p.Y-ys, p.Z-h)
}

// TestSignReadBoundCoversSlant publishes, for a kept vertex read by sign beside an
// end plane that the wall meets at a slant, a bound that reaches the nearest point
// of the true boundary measured along the wall: c/s for c = β + |v| + slack, not
// c. The vertex's own true wall lies on either side of the held facet (the foot
// of the vertex on the wall falls on the band's side, or not). Drop the slant
// term and every slanted row fails.
func TestSignReadBoundCoversSlant(t *testing.T) {
	t.Parallel()

	const beta = 0.001
	for _, s := range []float64{1, 0.866, 0.66, 0.4, 0.2} {
		for _, eps := range []float64{0, 0.0001, 0.0005, 0.0015} {
			for _, h := range []float64{beta, -beta} {
				w, b, f := tiltedBand(s, r3.NewVec(0.5, -eps/s, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
				cuts, err := w.cutBand(f, b)
				require.NoError(t, err, "s %v eps %v", s, eps)
				require.False(t, cuts.removed[0], "the vertex is read outside and kept")
				truth := keptDistance(w.verts[0], s, h)
				require.GreaterOrEqual(t, w.beta[0], truth, "s %v eps %v h %v", s, eps, h)
				if s < 1 && eps == 0 && h == beta {
					// The distance to the plane alone does not reach it.
					require.Less(t, beta+eps, truth, "the unslanted charge falls short at s %v", s)
				}
			}
		}
	}
}

// TestSignReadRefusesBelowMinSine refuses a sign reading where the wall meets the
// end plane at a sine below minSine, or where the end plane has no proven sine.
func TestSignReadRefusesBelowMinSine(t *testing.T) {
	t.Parallel()

	for name, sine := range map[string]func(float64) float64{
		"below minSine": func(float64) float64 { return minSine / 2 },
		"no bound":      nil,
		"proof fails at the reach": func(reach float64) float64 {
			if reach > 0 {
				return 0
			}
			return 1
		},
	} {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			w, b, f := flatBand(r3.NewVec(0.5, 0.0015, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
			b.ends[0].sine = sine
			_, err := w.cutBand(f, b)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.Equal(t, 0.001, w.beta[0], "nothing is charged")
		})
	}
}

// setbackBand is tiltedBand with the wall's lean to the main plane's normal set, as a
// route sets it from the wall's tag (wallTilt). The main plane is x = 1, the wall z = 0.
func setbackBand(s, lean float64, verts ...r3.Vec) (*work, *wallBand, faceLoops) {
	w, b, f := tiltedBand(s, verts...)
	b.lean, b.leanOK = lean, true
	return w, b, f
}

// TestSetbackBoundFormula states setbackBound for a point d from the setback plane,
// β from its true wall: β + (d + λβ + slack)/√(1 − λ²). Drop the offset d, the β
// term, the lean term, the slack, or the square root and a row fails.
func TestSetbackBoundFormula(t *testing.T) {
	t.Parallel()

	for _, c := range []struct {
		name                       string
		d, beta, lean, slack, want float64
	}{
		{name: "on the plane", d: 0, beta: 0.001, want: 0.001},
		{name: "off the plane", d: 0.003, beta: 0.001, want: 0.004},
		{name: "on the far side", d: -0.003, beta: 0.001, want: 0.004},
		{name: "slack", d: 0.003, beta: 0.001, slack: 0.0005, want: 0.0045},
		{name: "lean", d: 0, beta: 0.001, lean: 0.1, want: 0.001 + 0.1*0.001/math.Sqrt(1-0.01)},
		{name: "lean and offset", d: 0.003, beta: 0.001, lean: 0.1, want: 0.001 + (0.003+0.1*0.001)/math.Sqrt(1-0.01)},
	} {
		t.Run(c.name, func(t *testing.T) {
			t.Parallel()
			_, b, _ := setbackBand(1, c.lean)
			b.slack = c.slack
			got, ok := b.setbackBound(r3.NewVec(1-c.d, 0, 0), c.beta)
			require.True(t, ok)
			require.GreaterOrEqual(t, got, c.want-1e-15, "an upper bound never falls below the exact value")
			require.InDelta(t, c.want, got, 1e-12)
		})
	}
	t.Run("no lean from the tags, no bound", func(t *testing.T) {
		t.Parallel()
		_, b, _ := setbackBand(1, 0)
		b.leanOK = false
		_, ok := b.setbackBound(r3.NewVec(1, 0, 0), 0.001)
		require.False(t, ok)
		b.leanOK, b.main = true, nil
		_, ok = b.setbackBound(r3.NewVec(1, 0, 0), 0.001)
		require.False(t, ok, "a band without an exact main plane has no setback plane")
	})
}

// lineDistance is the distance from p to the line through q along the unit vector d.
func lineDistance(p, q, d r3.Vec) float64 { return p.Sub(q).Cross(d).Len() }

// TestSetbackBoundCoversSetbackLine reads setbackBound against the true setback line in
// every world the held claims allow: the true wall is a plane within β of the held point
// whose generators lean from the main plane's normal by the proven lean (every
// orientation of the lean about the normal), the true main plane lies off its tag by
// the slack either way. The distance from the held point to the true line where that
// wall meets that plane never exceeds the bound; the worst ratio is at most 1 and
// reaches 0.99, so the bound is tight. Each term of the formula is needed: a point on
// the plane (d = 0) with the wall leaning needs the lean term; an offset needs d.
func TestSetbackBoundCoversSetbackLine(t *testing.T) {
	t.Parallel()

	const beta = 0.001
	worst := 0.0
	for _, lean := range []float64{0, 0.017, 0.1} {
		for _, slack := range []float64{0, 0.0003} {
			for _, d := range []float64{0, 1e-5, 0.0004, 0.003, -0.003, 0.02} {
				_, b, _ := setbackBand(1, lean)
				b.slack = slack
				x := r3.NewVec(1-d, 0.3, 0)
				bound, ok := b.setbackBound(x, beta)
				require.True(t, ok)
				for _, rot := range []float64{0, 0.5, 1.2, 2.5, math.Pi, 4.1, 5.5} {
					for _, tt := range []float64{-beta, 0, beta} {
						for _, delta := range []float64{-slack, 0, slack} {
							// The wall's unit normal makes the angle asin(sin lean) with the
							// plane's: m·x̂ = sin(lean), the rest of m turned by rot about x̂.
							sl, cl := math.Sin(lean), math.Cos(lean)
							m := r3.NewVec(sl, cl*math.Cos(rot), cl*math.Sin(rot))
							// The wall passes tt from x along m.
							cw := m.Dot(x) - tt
							// The line m·p = cw, p.x = 1 + delta.
							mx := m.X
							ry := cw - mx*(1+delta)
							den := m.Y*m.Y + m.Z*m.Z
							q := r3.NewVec(1+delta, m.Y*ry/den, m.Z*ry/den)
							dir, okd := m.Cross(r3.NewVec(1, 0, 0)).Normalize()
							require.True(t, okd)
							dist := lineDistance(x, q, dir)
							ratio := dist / bound
							require.LessOrEqual(t, ratio, 1+1e-12, "lean %v slack %v d %v rot %v t %v delta %v", lean, slack, d, rot, tt, delta)
							worst = math.Max(worst, ratio)
						}
					}
				}
			}
		}
	}
	t.Logf("worst ratio %.6f", worst)
	require.GreaterOrEqual(t, worst, 0.99, "the bound is tight")
}

// TestSignReadKeepsSlantBoundForHeldVertices reads a held vertex 0.0015 from an end plane
// the wall meets at sine 0.3, d from the setback plane. A held vertex keeps its facets
// down the wall, whose points lie farther from the true result than the vertex does
// from the setback line, so it publishes the slant bound c/s however close it is to the
// plane, with or without a lean from the tags.
func TestSignReadKeepsSlantBoundForHeldVertices(t *testing.T) {
	t.Parallel()

	const s, beta, off = 0.3, 0.001, 0.0015
	slant := (beta + off) / s
	for _, d := range []float64{0.003, 0.02} {
		for _, lean := range []bool{true, false} {
			w, b, f := setbackBand(s, 0, r3.NewVec(1-d, off/s, 0), r3.NewVec(0.6, 2, 0), r3.NewVec(0.7, 4, 0))
			b.leanOK = lean
			cuts, err := w.cutBand(f, b)
			require.NoError(t, err)
			require.True(t, cuts.removed[0], "the vertex is read inside the band on the side of its offset")
			require.InDelta(t, slant, w.beta[0], 1e-9, "d %v lean %v", d, lean)
		}
	}
}

// TestCutBoundCoversTrueSetbackLineAfterSignRead reads the cuts of the main plane that a
// facet makes after a sign reading raised one of its corners. The cuts carry the input
// bounds alone (inputBeta), so they lie on the setback plane within β + the grid of the
// true line where the true wall meets it, in every world the corners' input bounds
// allow: the true wall is the plane through the three corners moved along the wall's
// normal by up to β each. Nothing the raise of the corner at the end plane states
// enters it. The ratio of the true distance to the published bound is at most 1 on
// every row, whichever way the end plane is tilted.
func TestCutBoundCoversTrueSetbackLineAfterSignRead(t *testing.T) {
	t.Parallel()

	const beta = 0.001
	worst, rows := 0.0, 0
	for _, s := range []float64{1, 0.66, 0.4} {
		for _, off := range []float64{0.0015, -0.0015, 0.0004} {
			v0 := r3.NewVec(0.5, off/s, 0)
			v1, v2 := r3.NewVec(1.5, 3, 0), r3.NewVec(0.7, 4, 0)
			var w *work
			var f faceLoops
			if s == 1 {
				var b *wallBand
				w, b, f = flatBand(v0, v1, v2)
				_, err := w.cutBand(f, b)
				require.NoError(t, err)
			} else {
				var b *wallBand
				w, b, f = tiltedBand(s, v0, v1, v2)
				_, err := w.cutBand(f, b)
				require.NoError(t, err)
			}
			require.Greater(t, w.beta[0], beta, "the corner at the end plane is raised")
			corners := [3]r3.Vec{v0, v1, v2}
			for id := 3; id < len(w.verts); id++ {
				p := w.verts[id]
				if math.Abs(p.X-1) > 1e-9 || p.Z != 0 {
					continue
				}
				for _, d := range [][3]float64{{beta, beta, beta}, {-beta, -beta, -beta}, {beta, -beta, 0}, {-beta, beta, beta},
					{beta, beta, -beta}, {-beta, 0, beta}, {0, -beta, beta}} {
					// z = a·x + b·y + c through the corners moved along z by d.
					q := [3]r3.Vec{{X: corners[0].X, Y: corners[0].Y, Z: d[0]}, {X: corners[1].X, Y: corners[1].Y, Z: d[1]},
						{X: corners[2].X, Y: corners[2].Y, Z: d[2]}}
					n := q[1].Sub(q[0]).Cross(q[2].Sub(q[0]))
					a, bb := -n.X/n.Z, -n.Y/n.Z
					c := q[0].Z - a*q[0].X - bb*q[0].Y
					// In the plane x = 1 the wall is the line z = a + bb·y + c.
					dist := math.Abs(a+bb*p.Y+c) / math.Sqrt(1+bb*bb)
					rows++
					worst = math.Max(worst, dist/w.beta[id])
					require.LessOrEqual(t, dist, w.beta[id], "s %v off %v cut at y %v world %v", s, off, p.Y, d)
				}
			}
		}
	}
	require.Positive(t, rows)
	require.Greater(t, worst, 0.1)
	t.Logf("rows %d worst ratio %.4f", rows, worst)
}

// TestCutTakesTheFacetsLargestInputBound gives the corner opposite the cut edge a larger
// input bound: the cut carries the facet's bound, the largest of its three corners'
// input bounds, not just its edge's two. Drop the facet term and it reads 0.002.
func TestCutTakesTheFacetsLargestInputBound(t *testing.T) {
	t.Parallel()

	w, b, f := flatBand(r3.NewVec(0.5, 1, 0), r3.NewVec(1.5, 1, 0), r3.NewVec(0.7, 4, 0))
	w.beta[2], w.fp.VertexBound[2] = 0.003, 0.003
	_, err := w.cutBand(f, b)
	require.NoError(t, err)
	found := false
	for id := 3; id < len(w.verts); id++ {
		if p := w.verts[id]; math.Abs(p.X-1) < 1e-12 && math.Abs(p.Y-1) < 1e-12 {
			found = true
			require.InDelta(t, 0.004, w.beta[id], 1e-12)
		}
	}
	require.True(t, found)
}

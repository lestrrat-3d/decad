package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// These fixtures assert docs/helix-design.md §13's PR 3 rows against the
// production path. Bound legs shown to fail by deleting them and watching
// the named assertion go red, then restoring them:
//
//   - the arc chord's sagitta in β (coilshell.Build's analytic leg):
//     TestCoilArcFacetBoundHoldsTheSurface went red on every fixture;
//   - the arc leg of coil.CellProof.Swept (the departure's sagitta):
//     TestCoilArcMeshProofsEncloseTheBody's volume check went red;
//   - the arc leg of coil.CellProof.Density: the same test's area check went
//     red;
//   - the caps' circular-segment allowance in areaSlack, about 0.06 mm² on
//     the round wire, is below that check's slack, so deleting it left the
//     test green.

// coilWallRole is a one-segment loop's wall role.
const coilWallRole = "side(0,0)"

// coilRoundWire is §13's round wire: a circle of radius r about (3, 0).
func coilRoundWire(t *testing.T, r float64) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(3, 0)
	s.Fix(c)
	s.CreateCircle(c, r)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// coilSlot is a stadium between (2.5, 0.5) and (3.5, 0.5) of radius 0.4:
// ρ ∈ [2.1, 3.9], ζ ∈ [0.1, 0.9], two lines and two semicircular arcs.
func coilSlot(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	slot, err := s.CreateSlot(2.5, 0.5, 3.5, 0.5, 0.4)
	require.NoError(t, err)
	s.Fix(slot.C1)
	s.Fix(slot.C2)
	for _, l := range []*sketch.Line{slot.L1, slot.L2} {
		s.Fix(l.Start)
		s.Fix(l.End)
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// circleWallIntegral is Θ·∫√h dα over a whole circle of radius r about
// (cu, 0) beside the V axis, h = r²·(cu + r·cos α)² + k²·r²·sin²α, by the
// trapezoid rule over 8192 points: the integrand is periodic and analytic, so
// the rule's own error is far below float64 rounding, which the caller
// charges at 1e-14 relative.
func circleWallIntegral(cu, r, pitch, turns float64) float64 {
	k := pitch / (2 * math.Pi)
	const n = 8192
	sum := new(big.Float).SetPrec(256)
	for i := range n {
		a := 2 * math.Pi * float64(i) / n
		rho := cu + r*math.Cos(a)
		s := math.Sin(a)
		sum.Add(sum, big.NewFloat(math.Sqrt(r*r*rho*rho+k*k*r*r*s*s)))
	}
	f, _ := sum.Float64()
	return 2 * math.Pi * turns * f * 2 * math.Pi / n
}

func TestCoilRoundWireSpring(t *testing.T) {
	s, p := coilRoundWire(t, 0.5)
	doc := New()
	b, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(5))
	require.NoError(t, err)
	pi := bigPi()

	t.Run("volume encloses 7.5π²", func(t *testing.T) {
		vol, err := b.Volume()
		require.NoError(t, err)
		// 2π·5 turns · (3 mm · π·0.25 mm²).
		exact := new(big.Float).SetPrec(512).Mul(pi, pi)
		exact.Mul(exact, big.NewFloat(7.5))
		requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(), exact, "volume")
		require.Less(t, vol.Bound.Base(), 1e-12*vol.Value.Base())
	})
	t.Run("area encloses the wall integral", func(t *testing.T) {
		wall := circleWallIntegral(3, 0.5, 1.5, 5)
		face, err := coilFace(t, b, coilWallRole).Area()
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(face.Value.Base()-wall), face.Bound.Base()+1e-14*wall)
		require.Less(t, face.Bound.Base(), 1e-9*wall)
		area, err := b.Area()
		require.NoError(t, err)
		// Two caps of π·0.25 mm² beside the wall.
		require.LessOrEqual(t, math.Abs(area.Value.Base()-(wall+0.5*math.Pi)), area.Bound.Base()+1e-14*wall)
		require.Less(t, area.Bound.Base(), 1e-9*area.Value.Base())
	})
	t.Run("centroid sits on the axis", func(t *testing.T) {
		c, err := b.Centroid()
		require.NoError(t, err)
		// M/Q = 0 for the section centred on ζ = 0, plus pitch·turns/2.
		require.LessOrEqual(t, c.Value.Sub(r3.NewVec(0, 3.75, 0)).Len(), c.Bound.Base())
	})
	t.Run("topology", func(t *testing.T) {
		require.Len(t, b.Faces(), 3)
		require.Len(t, b.Edges(), 2)
		require.Len(t, b.Vertices(), 2)
		for _, e := range b.Edges() {
			require.Len(t, e.Faces(), 2)
			circle, ok := e.Curve().(Circle3)
			require.True(t, ok, "a whole circle's rim is a closed Circle3")
			require.Equal(t, e.Start(), e.End())
			require.InDelta(t, 0.5, circle.Radius.Base(), 1e-15)
			require.True(t, e.IsConvex())
			length, err := e.Length()
			require.NoError(t, err)
			require.InDelta(t, math.Pi, length.Value.Base(), length.Bound.Base()+1e-15)
		}
		start := coilFace(t, b, roleCapStart).Loops()[0].CoEdges()[0].Edge().Curve().(Circle3)
		require.InDelta(t, 0, start.Center.Sub(r3.NewVec(3, 0, 0)).Len(), 1e-15)
		require.InDelta(t, 1, math.Abs(start.Axis.Z), 1e-15)
		// After five whole turns the end rim sits 7.5 mm up the axis in the
		// start rim's own plane.
		end := coilFace(t, b, roleCapEnd).Loops()[0].CoEdges()[0].Edge().Curve().(Circle3)
		require.InDelta(t, 0, end.Center.Sub(r3.NewVec(3, 7.5, 0)).Len(), 1e-12)
		require.InDelta(t, 0, end.Axis.Sub(start.Axis).Len(), 1e-12)
	})
	t.Run("bounds within δ", func(t *testing.T) {
		box, err := b.Bounds()
		require.NoError(t, err)
		k := 1.5 / (2 * math.Pi)
		lo, hi := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1)), r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
		const slop = 1e-12
		for i := range 4097 {
			th := 2 * math.Pi * 5 * float64(i) / 4096
			for a := range 64 {
				al := 2 * math.Pi * float64(a) / 64
				rho, zeta := 3+0.5*math.Cos(al), 0.5*math.Sin(al)
				q := r3.NewVec(rho*math.Cos(th), zeta+k*th, -rho*math.Sin(th))
				require.True(t, q.X >= box.Min.X-slop && q.X <= box.Max.X+slop)
				require.True(t, q.Y >= box.Min.Y-slop && q.Y <= box.Max.Y+slop)
				require.True(t, q.Z >= box.Min.Z-slop && q.Z <= box.Max.Z+slop)
				lo = r3.NewVec(math.Min(lo.X, q.X), math.Min(lo.Y, q.Y), math.Min(lo.Z, q.Z))
				hi = r3.NewVec(math.Max(hi.X, q.X), math.Max(hi.Y, q.Y), math.Max(hi.Z, q.Z))
			}
		}
		bound := box.Bound.Base() + slop
		for _, gap := range []float64{lo.X - box.Min.X, lo.Y - box.Min.Y, lo.Z - box.Min.Z, box.Max.X - hi.X, box.Max.Y - hi.Y, box.Max.Z - hi.Z} {
			require.LessOrEqual(t, gap, bound)
		}
		require.LessOrEqual(t, b.payload.(coilPayload).delta, box.Bound.Base())
	})
	t.Run("verify is sound", func(t *testing.T) {
		rep, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, ValidityValid, rep.Bodies[0].Validity.Outcome)
		require.Equal(t, Sound, rep.Status)
	})
}

func TestCoilSlotSpring(t *testing.T) {
	s, p := coilSlot(t)
	doc := New()
	b, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(3))
	require.NoError(t, err)

	// Q = 3·(0.8 + 0.16π): the section is symmetric about ρ = 3.
	vol, err := b.Volume()
	require.NoError(t, err)
	pi := bigPi()
	q := new(big.Float).SetPrec(512).Mul(pi, big.NewFloat(0.16))
	q.Add(q, big.NewFloat(0.8))
	q.Mul(q, big.NewFloat(3))
	exact := new(big.Float).SetPrec(512).Mul(q, pi)
	exact.Mul(exact, big.NewFloat(6))
	requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base()+1e-13, exact, "volume")

	require.Len(t, b.Faces(), 6)
	require.Len(t, b.Edges(), 12)
	require.Len(t, b.Vertices(), 8)
	arcs, lines, helices := 0, 0, 0
	for _, e := range b.Edges() {
		require.Len(t, e.Faces(), 2)
		switch c := e.Curve().(type) {
		case Arc3:
			arcs++
			require.InDelta(t, 0.4, c.Radius.Base(), 1e-15)
			require.True(t, e.IsConvex())
			length, err := e.Length()
			require.NoError(t, err)
			require.InDelta(t, 0.4*math.Pi, length.Value.Base(), length.Bound.Base()+1e-15)
		case Line3:
			lines++
		case FacetedCurve:
			helices++
			// The slot's junctions are tangent: no turn, so no convex edge.
			require.False(t, e.IsConvex())
		}
	}
	require.Equal(t, 4, arcs)
	require.Equal(t, 4, lines)
	require.Equal(t, 4, helices)
	// Every arc rim runs counter-clockwise about its axis from its start to
	// its end vertex: the midpoint of that sweep lies on the arc.
	for _, e := range b.Edges() {
		c, ok := e.Curve().(Arc3)
		if !ok {
			continue
		}
		a := e.Start().Position().Value.Sub(c.Center)
		mid := c.Center.Add(c.Axis.Cross(a))
		require.InDelta(t, 0.4, mid.Sub(c.Center).Len(), 1e-12)
		d := math.Inf(1)
		for _, v := range b.payload.(coilPayload).verts {
			d = math.Min(d, v.Sub(mid).Len())
		}
		require.Less(t, d, 1e-9, "the counter-clockwise quarter point is a held station")
	}

	rot, err := r3.Rotation(r3.NewVec(1, 2, 2), units.Degrees(37))
	require.NoError(t, err)
	placed, err := b.Placed(t.Context(), rot)
	require.NoError(t, err)
	pv, err := placed.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(pv.Value.Base()-vol.Value.Base()), pv.Bound.Base()+vol.Bound.Base())
	area, err := b.Area()
	require.NoError(t, err)
	pa, err := placed.Area()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(pa.Value.Base()-area.Value.Base()), pa.Bound.Base()+area.Bound.Base())

	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	for _, body := range rep.Bodies {
		require.Equal(t, ValidityValid, body.Validity.Outcome)
	}
}

func TestCoilArcRefusals(t *testing.T) {
	t.Run("CS6 at a wire diameter equal to the pitch", func(t *testing.T) {
		s, p := coilRoundWire(t, 0.75)
		doc := New()
		_, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(1))
		require.ErrorIs(t, err, ErrUnsupported)
		require.ErrorContains(t, err, "at or past the 1.5 mm pitch")
		require.Empty(t, doc.Bodies())
		// Below one turn the same wire builds.
		b, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(0.75))
		require.NoError(t, err)
		vol, err := b.Volume()
		require.NoError(t, err)
		exact := new(big.Float).SetPrec(512).Mul(bigPi(), bigPi())
		exact.Mul(exact, big.NewFloat(2*0.75*3*0.5625))
		requireEnclosesBig(t, vol.Value.Base(), vol.Bound.Base(), exact, "three-quarter turn")
	})
	t.Run("CS5 a circle reaching the axis", func(t *testing.T) {
		s, p := coilRoundWire(t, 3)
		doc := New()
		_, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(7), units.Scalar(1))
		require.ErrorIs(t, err, ErrDegenerate)
		require.Empty(t, doc.Bodies())
	})
}

// coilBite is the square ρ ∈ [2, 3.2], ζ ∈ [0, 1] whose right side is a
// concave arc about (3.6, 0.5), radius √0.41, through (2.96, 0.5): the arc
// runs counter-clockwise from (3.2, 1) to (3.2, 0), and the outer loop walks
// it the other way.
func coilBite(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	pt := func(u, v float64) *sketch.Point {
		p := s.CreatePoint(u, v)
		s.Fix(p)
		return p
	}
	a, b, c, d := pt(2, 0), pt(3.2, 0), pt(3.2, 1), pt(2, 1)
	s.CreateLine(a, b)
	s.CreateArc(pt(3.6, 0.5), c, b)
	s.CreateLine(c, d)
	s.CreateLine(d, a)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// TestCoilArcFacetBoundHoldsTheSurface samples the true wall of every chord
// cell — Φ of the recorded curve, evaluated in float64 from §3's formula —
// and asserts each sample lies within the cell's facet bound (the largest β
// over its four corners) of the cell's two held triangles. It is a falsifier
// of docs/helix-design.md §5.4's β over an arc chord, never its proof.
func TestCoilArcFacetBoundHoldsTheSurface(t *testing.T) {
	for _, c := range []struct {
		name    string
		profile func(*testing.T) (*sketch.Sketch, *sketch.Profile)
		turns   float64
	}{
		{"round wire", func(t *testing.T) (*sketch.Sketch, *sketch.Profile) { return coilRoundWire(t, 0.5) }, 2},
		{"thick wire", func(t *testing.T) (*sketch.Sketch, *sketch.Profile) { return coilRoundWire(t, 1.2) }, 0.5},
		{"concave bite", coilBite, 1},
	} {
		t.Run(c.name, func(t *testing.T) {
			s, p := c.profile(t)
			b, err := New().Coil(t.Context(), s, p, coilAxisV, units.Millimeters(3), units.Scalar(c.turns))
			require.NoError(t, err)
			cp := b.payload.(coilPayload)
			rec, err := coilRecordOfPayload(cp)
			require.NoError(t, err)
			stride, n := len(rec.Pts), rec.N
			idx := rec.LoopIdx[0]
			if c.name == "concave bite" {
				// The outer loop walks the arc against its counter-clockwise
				// sense, and its rims are concave.
				reversed := 0
				for _, seg := range rec.Profile.Segments {
					if seg.IsArc() && seg.Reversed {
						reversed++
					}
				}
				require.Equal(t, 1, reversed)
				for _, e := range b.Edges() {
					if _, ok := e.Curve().(Arc3); ok {
						require.False(t, e.IsConvex())
					}
				}
			}
			// along(v, w, l) is the recorded curve's point a fraction l of
			// the way along chord v → w, at a matched parameter.
			along := func(v, w int, l float64) (float64, float64) {
				pv, pw := rec.Pts[v], rec.Pts[w]
				seg := rec.Profile.Segments[rec.Profile.Chord[v].Segment]
				if !seg.IsArc() {
					return pv.U + l*(pw.U-pv.U), pv.V + l*(pw.V-pv.V)
				}
				cu, cv := coilshell.Centre(seg)
				fu, _ := cu.Float64()
				fv, _ := cv.Float64()
				r, _ := seg.Radius.Lo.Float64()
				a0 := math.Atan2(pv.V-fv, pv.U-fu)
				a1 := math.Atan2(pw.V-fv, pw.U-fu)
				if a1-a0 > math.Pi {
					a1 -= 2 * math.Pi
				}
				if a0-a1 > math.Pi {
					a1 += 2 * math.Pi
				}
				a := a0 + l*(a1-a0)
				return fu + r*math.Cos(a), fv + r*math.Sin(a)
			}
			k := 3 / (2 * math.Pi)
			theta := 2 * math.Pi * c.turns
			worst := 0.0
			for j := range n {
				for kk := range stride {
					v, w := idx[kk], idx[(kk+1)%stride]
					corners := []int{int(j)*stride + v, int(j)*stride + w, int(j+1)*stride + v, int(j+1)*stride + w}
					facet := 0.0
					for _, q := range corners {
						facet = math.Max(facet, cp.vertexBound[q])
					}
					cell := 2 * (int(j)*stride + kk)
					tris := [][3]int{cp.tris[cell], cp.tris[cell+1]}
					for _, l := range []float64{0, 0.25, 0.5, 0.75, 1} {
						rho, zeta := along(v, w, l)
						for _, sf := range []float64{0, 0.2, 0.5, 0.8, 1} {
							th := theta * (float64(j) + sf) / float64(n)
							q := r3.NewVec(rho*math.Cos(th), zeta+k*th, -rho*math.Sin(th))
							d := math.Inf(1)
							for _, tri := range tris {
								d = math.Min(d, closestOnTriangle(q, cp.verts[tri[0]], cp.verts[tri[1]], cp.verts[tri[2]]))
							}
							require.LessOrEqual(t, d, facet+1e-12, "cell %d of chord %d at (%v, %v)", j, v, l, sf)
							worst = math.Max(worst, d/facet)
						}
					}
				}
			}
			// The arc's own sagitta is most of an arc cell's facet bound.
			require.Greater(t, worst, 0.5)
		})
	}
}

// TestCoilArcMeshProofsEncloseTheBody is TestCoilMeshProofsEncloseTheBody
// over arc profiles: the held mesh's signed volume lies within volSymDiff of
// Θ·Q and its area within areaSlack of the published Area. Both are
// falsifiers; the proofs are docs/helix-design.md §8.1 and §8.2.
func TestCoilArcMeshProofsEncloseTheBody(t *testing.T) {
	s, p := coilRoundWire(t, 0.5)
	b, err := New().Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2))
	require.NoError(t, err)
	mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	held := meshSignedVolume(mesh.Vertices(), mesh.Triangles())
	// 2π·2 turns · 3 · π·0.25 = 3π².
	exact := new(big.Float).SetPrec(512).Mul(bigPi(), bigPi())
	exact.Mul(exact, big.NewFloat(3))
	requireEnclosesBig(t, held, mesh.volSymDiff+1e-12, exact, "held volume")
	area, err := b.Area()
	require.NoError(t, err)
	gap := math.Abs(meshArea(mesh.Vertices(), mesh.Triangles()) - area.Value.Base())
	require.LessOrEqual(t, gap, mesh.areaSlack+area.Bound.Base()+1e-9)

	for k := range mesh.Triangles() {
		require.Contains(t, []string{roleCapStart, roleCapEnd, coilWallRole}, mesh.source[k].Origins()[0].Role)
	}
}

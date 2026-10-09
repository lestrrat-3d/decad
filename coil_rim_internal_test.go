package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coil"
	"github.com/lestrrat-3d/decad/internal/coilshell"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// coilFullTurnArc is an ArcSeg whose End is its Start: a whole turn of radius
// 0.5 about (3, 0), alone in its loop.
func coilFullTurnArc(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(3, 0)
	s.Fix(c)
	p := s.CreatePoint(3.5, 0)
	s.Fix(p)
	s.CreateArc(c, p, p)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)
	return s, s.Profiles()[0]
}

// TestCoilFullTurnArcClosesLikeACircle coils an arc that ends where it
// starts. Its wall is a band bounded by its two rim circles, the whole
// circle's topology: no helix edge closes the wall on itself, and the body
// reads what the same circle recorded as a CircleSeg reads.
func TestCoilFullTurnArcClosesLikeACircle(t *testing.T) {
	s, p := coilFullTurnArc(t)
	rec, _, _, err := recordProfile(s, p)
	require.NoError(t, err)
	require.Len(t, rec.Outer.Segments, 1)
	arc, ok := rec.Outer.Segments[0].(sectionrecord.ArcSeg)
	require.True(t, ok, "premise: the profile records one ArcSeg")
	require.Equal(t, arc.Start, arc.End)

	doc := New()
	b, err := doc.Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2.3))
	require.NoError(t, err)
	require.Len(t, b.Faces(), 3)
	require.Len(t, b.Edges(), 2)
	require.Len(t, b.Vertices(), 2)
	for _, e := range b.Edges() {
		_, ok := e.Curve().(Circle3)
		require.True(t, ok, "a closed rim is a Circle3")
		require.Equal(t, e.Start(), e.End())
		require.Len(t, e.Faces(), 2)
		require.NotEqual(t, e.Faces()[0], e.Faces()[1], "no face holds an edge twice")
	}
	wall := coilFace(t, b, coilWallRole)
	require.Len(t, wall.Loops(), 2)
	for _, l := range wall.Loops() {
		require.Len(t, l.CoEdges(), 1)
	}

	sc, pc := coilRoundWire(t, 0.5)
	circle, err := New().Coil(t.Context(), sc, pc, coilAxisV, units.Millimeters(1.5), units.Scalar(2.3))
	require.NoError(t, err)
	for _, read := range []func(*Body) (Measurement, error){(*Body).Volume, (*Body).Area} {
		got, err := read(b)
		require.NoError(t, err)
		want, err := read(circle)
		require.NoError(t, err)
		require.LessOrEqual(t, math.Abs(got.Value.Base()-want.Value.Base()), got.Bound.Base()+want.Bound.Base())
	}
	rep, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, ValidityValid, rep.Bodies[0].Validity.Outcome)
}

// bigCircleGap is the distance, in 512-bit floating point, from the world
// point p to the circle of radius r about c normal to axis.
func bigCircleGap(p [3]*big.Float, c, axis r3.Vec, r float64) *big.Float {
	const prec = 512
	f := func(x float64) *big.Float { return new(big.Float).SetPrec(prec).SetFloat64(x) }
	cv, av := [3]float64{c.X, c.Y, c.Z}, [3]float64{axis.X, axis.Y, axis.Z}
	var v [3]*big.Float
	vv, va, aa := new(big.Float).SetPrec(prec), new(big.Float).SetPrec(prec), new(big.Float).SetPrec(prec)
	for k := range 3 {
		v[k] = new(big.Float).SetPrec(prec).Sub(p[k], f(cv[k]))
		vv.Add(vv, new(big.Float).SetPrec(prec).Mul(v[k], v[k]))
		va.Add(va, new(big.Float).SetPrec(prec).Mul(v[k], f(av[k])))
		aa.Add(aa, new(big.Float).SetPrec(prec).Mul(f(av[k]), f(av[k])))
	}
	a2 := new(big.Float).SetPrec(prec).Quo(new(big.Float).SetPrec(prec).Mul(va, va), aa)
	perp := new(big.Float).SetPrec(prec).Sub(vv, a2)
	if perp.Sign() < 0 {
		perp.SetInt64(0)
	}
	perp.Sqrt(perp)
	radial := new(big.Float).SetPrec(prec).Sub(perp, f(r))
	out := new(big.Float).SetPrec(prec).Add(a2, new(big.Float).SetPrec(prec).Mul(radial, radial))
	return out.Sqrt(out)
}

// denotedRimPoints returns points of the circle a coil rim denotes: the
// recorded circle of centre (cu, cv) and radius r, carried through the
// screw motion at the given turn and the record's exact world map. Each is
// exact up to the 2⁻²⁰⁰ trig enclosure and the 512-bit radius: the unit
// circle is sampled at rational points ((1 − s²), 2s)/(1 + s²).
func denotedRimPoints(t *testing.T, rec coilshell.Record, cu, cv float64, r *big.Float, turn *big.Rat) [][3]*big.Float {
	t.Helper()
	mid := func(x coil.Iv) *big.Rat { return coil.Mid(x) }
	sinT, cosT := coil.TurnSinCos(turn)
	sn, cs := mid(sinT), mid(cosT)
	erU, erV := rec.Axis.Radial()
	pu, pv := mid(erU), mid(erV)
	au, av := mid(rec.Axis.AU), mid(rec.Axis.AV)
	du, dv := mid(rec.Axis.DU), mid(rec.Axis.DV)
	slide := new(big.Rat).Mul(rec.Pitch, turn)
	tilt := big.NewRat(int64(rec.Sigma*rec.Axis.Side), 1)
	rr, _ := r.Rat(nil)
	var out [][3]*big.Float
	for i := -40; i <= 40; i++ {
		s := big.NewRat(int64(i), 8)
		den := new(big.Rat).Add(big.NewRat(1, 1), new(big.Rat).Mul(s, s))
		c := new(big.Rat).Quo(new(big.Rat).Sub(big.NewRat(1, 1), new(big.Rat).Mul(s, s)), den)
		sv := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), s), den)
		for _, flip := range []int64{1, -1} {
			u := new(big.Rat).Add(new(big.Rat).SetFloat64(cu), new(big.Rat).Mul(rr, c))
			v := new(big.Rat).Add(new(big.Rat).SetFloat64(cv), new(big.Rat).Mul(rr, new(big.Rat).Mul(sv, big.NewRat(flip, 1))))
			rho := new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).Sub(u, au), pu), new(big.Rat).Mul(new(big.Rat).Sub(v, av), pv))
			cm1 := new(big.Rat).Sub(cs, big.NewRat(1, 1))
			x := new(big.Rat).Add(u, new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).Mul(cm1, rho), pu), new(big.Rat).Mul(slide, du)))
			y := new(big.Rat).Add(v, new(big.Rat).Add(new(big.Rat).Mul(new(big.Rat).Mul(cm1, rho), pv), new(big.Rat).Mul(slide, dv)))
			z := new(big.Rat).Mul(new(big.Rat).Mul(sn, rho), tilt)
			w := rec.World.Point(coil.Point(x), coil.Point(y), coil.Point(z))
			var pt [3]*big.Float
			for k := range 3 {
				pt[k] = new(big.Float).SetPrec(512).SetRat(w[k].Lo)
			}
			out = append(out, pt)
		}
	}
	return out
}

// TestCoilRimCurveBoundHoldsTheDenotedCircle samples the circle every coil
// rim denotes — the recorded circle under the screw motion and the record's
// exact world map — and asserts each sample lies within the rim edge's
// curveBound of its held circle. It is a falsifier of coilRimCurveBound,
// never its proof. At 2.3 turns the end rim's centre and axis round, and
// under a rotation the map's own defect enters too. Legs shown to fail by
// deleting them and watching this test go red, then restoring them: the
// centre's own gap |D| (the round wire and the placed slot), and the axial
// term 2·ax (the slot). The radius gap |r − heldRadius| and the map's
// orthonormality term r·e sit below the other legs on these fixtures, so
// deleting either left the test green.
func TestCoilRimCurveBoundHoldsTheDenotedCircle(t *testing.T) {
	rot, err := r3.Rotation(r3.NewVec(1, 2, 2), units.Degrees(37))
	require.NoError(t, err)
	for _, c := range []struct {
		name    string
		profile func(*testing.T) (*sketch.Sketch, *sketch.Profile)
		placed  bool
	}{
		{"round wire", func(t *testing.T) (*sketch.Sketch, *sketch.Profile) { return coilRoundWire(t, 0.5) }, false},
		{"round wire placed", func(t *testing.T) (*sketch.Sketch, *sketch.Profile) { return coilRoundWire(t, 0.5) }, true},
		{"slot", coilSlot, false},
		{"slot placed", coilSlot, true},
	} {
		t.Run(c.name, func(t *testing.T) {
			s, p := c.profile(t)
			b, err := New().Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2.3))
			require.NoError(t, err)
			if c.placed {
				b, err = b.Placed(t.Context(), rot)
				require.NoError(t, err)
			}
			rec, err := coilRecordOfPayload(b.payload.(coilPayload))
			require.NoError(t, err)
			rims, worst := 0, 0.0
			for _, e := range b.Edges() {
				var centre, axis r3.Vec
				var radius float64
				switch cv := e.Curve().(type) {
				case Circle3:
					centre, axis, radius = cv.Center, cv.Axis, cv.Radius.Base()
				case Arc3:
					centre, axis, radius = cv.Center, cv.Axis, cv.Radius.Base()
				default:
					continue
				}
				rims++
				require.True(t, e.curveBounded, "a coil rim proves its curve bound")
				require.Less(t, e.curveBound, 1e-12)
				turn := new(big.Rat)
				for _, f := range e.Faces() {
					if f.Origins()[0].Role == roleCapEnd {
						turn = rec.Turns
					}
				}
				// The rim's recorded circle is the one whose start-cap lift
				// sits nearest the held centre at the rim's own turn.
				best := math.Inf(1)
				for _, seg := range rec.Profile.Segments {
					if !seg.IsArc() {
						continue
					}
					cu, cv := coilCentre(seg)
					fu, _ := cu.Float64()
					fv, _ := cv.Float64()
					rr := new(big.Float).SetPrec(512)
					switch rs := seg.Record.(type) {
					case sectionrecord.CircleSeg:
						rr.SetFloat64(rs.Radius.Base())
					case sectionrecord.ArcSeg:
						du := new(big.Float).SetPrec(512).Sub(new(big.Float).SetFloat64(rs.Start.U), new(big.Float).SetFloat64(rs.Center.U))
						dv := new(big.Float).SetPrec(512).Sub(new(big.Float).SetFloat64(rs.Start.V), new(big.Float).SetFloat64(rs.Center.V))
						rr.Add(new(big.Float).SetPrec(512).Mul(du, du), new(big.Float).SetPrec(512).Mul(dv, dv))
						rr.Sqrt(rr)
					}
					pts := denotedRimPoints(t, rec, fu, fv, rr, turn)
					gap := 0.0
					for _, pt := range pts {
						g, _ := bigCircleGap(pt, centre, axis, radius).Float64()
						gap = math.Max(gap, g)
					}
					best = math.Min(best, gap)
				}
				// 1e-100 charges the reference's own 512-bit arithmetic.
				require.LessOrEqual(t, best, e.curveBound+1e-100, "the denoted rim lies within the curve bound")
				worst = math.Max(worst, best)
			}
			require.Positive(t, rims)
			require.Positive(t, worst, "the fixture's rims round")
		})
	}
}

// TestCoilUnstitchCarriesTheRimCurveBound unstitches a coil and reads the
// cap sheet's free rim: the sheet's edge carries the coil rim's own proven
// curve bound rather than dropping it.
func TestCoilUnstitchCarriesTheRimCurveBound(t *testing.T) {
	s, p := coilRoundWire(t, 0.5)
	b, err := New().Coil(t.Context(), s, p, coilAxisV, units.Millimeters(1.5), units.Scalar(2.3))
	require.NoError(t, err)
	var want float64
	for _, e := range coilFace(t, b, roleCapEnd).Loops()[0].CoEdges() {
		require.True(t, e.Edge().curveBounded)
		want = e.Edge().curveBound
	}
	sheets, err := b.Unstitch(t.Context())
	require.NoError(t, err)
	found := false
	for _, sh := range sheets {
		if sh.Faces()[0].Origins()[0].Role != roleCapEnd {
			continue
		}
		for _, e := range sh.Edges() {
			require.True(t, e.curveBounded)
			require.Equal(t, want, e.curveBound)
			found = true
		}
	}
	require.True(t, found)
}

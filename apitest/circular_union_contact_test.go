package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func insetPoint(s *sketch.Sketch, x, y float64) *sketch.Point {
	p := s.CreatePoint(x, y)
	s.AddConstraint(sketch.NewHorizontalDistance(s.Origin(), p, x), sketch.NewVerticalDistance(s.Origin(), p, y))
	return p
}
func insetSketch(t *testing.T, origin, u, v r3.Vec) *sketch.Sketch {
	t.Helper()
	f, e := r3.NewFrame(origin, u, v)
	require.NoError(t, e)
	w := sketch.NewWorld()
	p, e := w.CreatePlaneFromFrame(f)
	require.NoError(t, e)
	s, e := w.CreateSketch(p)
	require.NoError(t, e)
	return s
}
func insetExtrude(t *testing.T, doc *decad.Document, s *sketch.Sketch, d float64, dir decad.Direction) *decad.Body {
	t.Helper()
	_, e := s.Solve(t.Context())
	require.NoError(t, e)
	p := s.Profiles()
	require.Len(t, p, 1)
	b, e := decad.Extrude(t.Context(), doc, s, p[0], decad.Distance{D: units.Millimeters(d), Dir: dir})
	require.NoError(t, e)
	return b
}
func insetCheekOutline(s *sketch.Sketch) {
	n := s.Origin()
	u := insetPoint(s, 7.5, 7.5)
	l := insetPoint(s, 7.5, -7.5)
	d := math.Hypot(7.5, 7.5)
	c := -0.25 / d
	q := math.Sqrt(1 - c*c)
	a, b := (c-q)/math.Sqrt2, (c+q)/math.Sqrt2
	nu := s.CreatePoint(3.50*a, 3.50*b)
	nl := s.CreatePoint(3.50*a, -3.50*b)
	to := s.CreatePoint(7.5+3.75*a, 7.5+3.75*b)
	lo := s.CreatePoint(7.5+3.75*a, -7.5-3.75*b)
	ur, lr := s.CreatePoint(11.25, 7.5), s.CreatePoint(11.25, -7.5)
	arcs := []*sketch.Arc{s.CreateArc(n, nu, nl), s.CreateArc(u, ur, to), s.CreateArc(l, lo, lr)}
	t, dd, rr := s.CreateLine(to, nu), s.CreateLine(nl, lo), s.CreateLine(lr, ur)
	s.AddConstraint(sketch.NewRadius(arcs[0], 3.50), sketch.NewRadius(arcs[1], 3.75), sketch.NewEqualRadius(arcs[2], arcs[1]),
		sketch.NewTangent(t, arcs[1]), sketch.NewTangent(t, arcs[0]), sketch.NewTangent(dd, arcs[0]),
		sketch.NewTangent(dd, arcs[2]), sketch.NewTangent(rr, arcs[2]), sketch.NewTangent(rr, arcs[1]))
}

func insetRoundedRect(s *sketch.Sketch, x0, x1, y0, y1, rad float64) error {
	o := s.Origin()
	v := [][2]float64{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}
	p := make([]*sketch.Point, 4)
	for i, q := range v {
		p[i] = s.CreatePoint(q[0], q[1])
	}
	l := make([]*sketch.Line, 4)
	for i := range p {
		l[i] = s.CreateLine(p[i], p[(i+1)%4])
		if i%2 == 0 {
			s.AddConstraint(sketch.NewHorizontal(l[i]), sketch.NewDistancePointLine(o, l[i], math.Abs(v[i][1])))
		} else {
			s.AddConstraint(sketch.NewVertical(l[i]), sketch.NewDistancePointLine(o, l[i], math.Abs(v[i][0])))
		}
	}
	if rad > 0 {
		for i := range p {
			if _, err := sketch.CreateFillet(s, l[(i+3)%4], l[i], p[i], rad); err != nil {
				return err
			}
		}
	}
	return nil
}

func insetForkTarget(t *testing.T, doc *decad.Document, relief bool) *decad.Body {
	t.Helper()
	ctx := t.Context()
	x, y, z := r3.Vec{X: 1}, r3.Vec{Y: 1}, r3.Vec{Z: 1}
	cheek := func(start, depth float64) *decad.Body {
		s := insetSketch(t, r3.Vec{X: 32, Y: start, Z: 52}, x, z)
		insetCheekOutline(s)
		return insetExtrude(t, doc, s, depth, decad.Against)
	}
	base := cheek(-75.9, 2.4)
	s := insetSketch(t, r3.Vec{X: 36, Y: -77.5, Z: 52}, y, z)
	require.NoError(t, insetRoundedRect(s, -5, 5, -5, 5, 2))
	stem := insetExtrude(t, doc, s, 33, decad.Along)
	stem, e := decad.Fillet(ctx, stem, decad.Edges(decad.EndpointAt(r3.Vec{X: 36, Y: -80.5, Z: 47}), decad.ParallelTo(y)).Exactly(1), units.Millimeters(1.5), decad.WithTangentChain())
	require.NoError(t, e)
	front := cheek(-82.5, 3.4)
	stem, e = decad.Union(ctx, stem, base)
	require.NoError(t, e)
	stem, e = decad.Fillet(ctx, stem, decad.Edges(decad.EndpointAt(r3.Vec{X: 37.5, Y: -75.9, Z: 47}), decad.ParallelTo(x)).Exactly(1), units.Millimeters(1.2), decad.WithTangentChain())
	require.NoError(t, e)
	if relief {
		tool, e := decad.Copy(ctx, front)
		require.NoError(t, e)
		stem, e = decad.Cut(ctx, stem, tool)
		require.NoError(t, e)
	}
	return stem
}
func insetBoss(t *testing.T, doc *decad.Document, z float64) *decad.Body {
	t.Helper()
	s := insetSketch(t, r3.Vec{X: 39.5, Y: -79.1, Z: z}, r3.Vec{X: 1}, r3.Vec{Z: 1})
	s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), 3.75), 3.75))
	return insetExtrude(t, doc, s, 4.85, decad.Against)
}
func TestCircularUnionContact(t *testing.T) {
	for _, tc := range []struct {
		name               string
		z                  float64
		reverse, wantShift bool
	}{
		{"lower boss", 44.5, false, false}, {"reversed lower boss", 44.5, true, false},
		{"mixed tangent", 60.75, false, true}, {"reversed mixed tangent", 60.75, true, true}, {"shallow overlap", 60.75 - .00001, false, true},
		{"deep overlap", 60.65, false, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New(decad.WithChainTolerance(units.Millimeters(.05)))
			a, b := insetForkTarget(t, doc, true), insetBoss(t, doc, tc.z)
			if tc.reverse {
				a, b = b, a
			}
			got, err := decad.Union(t.Context(), a, b)
			require.NoError(t, err)
			require.Len(t, got.Lumps(), 1)
			if !tc.wantShift && tc.z == 44.5 {
				for _, d := range got.OperationDiagnostics() {
					require.NotEqual(t, decad.OpDiagGapShiftApplied, d.Code)
				}
			}
			if tc.wantShift {
				// The radial candidate must decline this contact. The ordinary retry records its movement.
				shifted := false
				for _, diagnostic := range got.OperationDiagnostics() {
					if diagnostic.Code == decad.OpDiagGapShiftApplied {
						shifted = true
					}
				}
				require.True(t, shifted)
				return
			}
			m, e := got.Tessellate(t.Context(), units.Millimeters(.05), decad.WithVerification(decad.VerifyAll))
			require.NoError(t, e)
			require.Less(t, m.Bound().Base(), .05)
		})
	}
}
func TestCircularUnionCancellation(t *testing.T) {
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(.05)))
	a, b := insetForkTarget(t, doc, true), insetBoss(t, doc, 44.5)
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	_, e := decad.Union(ctx, a, b)
	require.ErrorIs(t, e, context.Canceled)
}

func TestCircularUnionZeroGap(t *testing.T) {
	for _, tc := range []struct {
		name    string
		z       float64
		reverse bool
	}{{"mixed tangent", 60.75, false}, {"reversed mixed tangent", 60.75, true}, {"shallow overlap", 60.75 - .00001, false}} {
		t.Run(tc.name, func(t *testing.T) {
			doc := decad.New(decad.ExactOnly(), decad.WithChainTolerance(units.Millimeters(.05)))
			a, b := insetForkTarget(t, doc, false), insetBoss(t, doc, tc.z)
			if tc.reverse {
				a, b = b, a
			}
			_, err := decad.Union(t.Context(), a, b)
			require.ErrorIs(t, err, decad.ErrUnsupported)
		})
	}
}

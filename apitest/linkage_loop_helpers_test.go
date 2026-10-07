package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file builds the closed-loop scenes of docs/linkage-check-design.md
// §15.10 and the closed forms they are checked against: the four-bar's
// two-circle construction. Every link sits in its own layer along Z, so the
// layer exclusion settles every link-link pair.

// polyPrismAtZ extrudes a closed polygon drawn on the plane z = z0, every
// vertex pinned at its authored coordinate, to z0 + h.
func polyPrismAtZ(t *testing.T, doc *decad.Document, pts [][2]float64, z0, h float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	plane, err := w.CreateOffsetPlane(w.XY(), z0)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	sp := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		sp[i] = s.CreatePoint(p[0], p[1])
		s.Fix(sp[i])
	}
	for i := range sp {
		s.CreateLine(sp[i], sp[(i+1)%len(sp)])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profs := s.Profiles()
	require.Len(t, profs, 1)
	body, err := doc.Extrude(s, profs[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// barBody is a bar of half-width 4 from p to q in the XY plane, z ∈ [z0, z0 + 8].
func barBody(t *testing.T, doc *decad.Document, p, q [2]float64, z0 float64) *decad.Body {
	t.Helper()
	dx, dy := q[0]-p[0], q[1]-p[1]
	n := math.Hypot(dx, dy)
	nx, ny := -dy/n*4, dx/n*4
	return polyPrismAtZ(t, doc, [][2]float64{
		{p[0] - nx, p[1] - ny}, {q[0] - nx, q[1] - ny}, {q[0] + nx, q[1] + ny}, {p[0] + nx, p[1] + ny},
	}, z0, 8)
}

// fourBarTheta4 is the follower's angle from the ground line at crank angle
// th2 (radians) for ground g, crank r, coupler l and follower f, on the branch
// with the coupler pin above the ground line.
func fourBarTheta4(th2, g, r, l, f float64) float64 {
	d := math.Sqrt(g*g + r*r - 2*g*r*math.Cos(th2))
	phi := math.Atan2(r*math.Sin(th2), r*math.Cos(th2)-g)
	beta := math.Acos((f*f + d*d - l*l) / (2 * f * d))
	return math.Mod(phi-beta+2*math.Pi, 2*math.Pi)
}

// bisectRoot brackets a root of fn in [lo, hi] to 1e-12.
func bisectRoot(fn func(float64) float64, lo, hi float64) float64 {
	flo := fn(lo)
	for hi-lo > 1e-12 {
		mid := (lo + hi) / 2
		fm := fn(mid)
		if (fm < 0) == (flo < 0) {
			lo, flo = mid, fm
			continue
		}
		hi = mid
	}
	return (lo + hi) / 2
}

// fourBar is a four-bar linkage in two branches off the ground: the crank and
// the follower under the ground, the coupler under the crank, the coupler
// closed onto the follower at B.
type fourBar struct {
	doc                      *decad.Document
	g, r, l, f               float64
	crankBody, coupler, foll *decad.Body
	linkage                  *decad.Linkage
	crank, couplerLk, follow *decad.Link
	loop                     *decad.LinkageLoop
	b                        [2]float64
}

// fourBarGround is the ground bar of every four-bar the loop tests build:
// scene 7's crank-rocker and scene 9's fold alike.
const fourBarGround = 100.0

// buildFourBar builds the four-bar with ground pivots (0, 0) and
// (fourBarGround, 0), the crank along +X at the zero pose and the coupler pin
// B on the branch above the ground line, crank z ∈ [0, 8], coupler
// z ∈ [10, 18], follower z ∈ [20, 28].
func buildFourBar(t *testing.T, doc *decad.Document, r, l, f float64) fourBar {
	t.Helper()
	g := fourBarGround
	fb := fourBar{doc: doc, g: g, r: r, l: l, f: f}
	t4 := fourBarTheta4(0, g, r, l, f)
	fb.b = [2]float64{g + f*math.Cos(t4), f * math.Sin(t4)}
	fb.crankBody = boxBodyAtZ(t, doc, 0, -4, r, 4, 0, 8)
	fb.coupler = barBody(t, doc, [2]float64{r, 0}, fb.b, 10)
	fb.foll = barBody(t, doc, [2]float64{g, 0}, fb.b, 20)
	fb.linkage = decad.NewLinkage()
	z := r3.NewVec(0, 0, 1)
	var err error
	fb.crank, err = fb.linkage.Ground().Revolute(r3.Vec{}, z, []*decad.Body{fb.crankBody})
	require.NoError(t, err)
	fb.couplerLk, err = fb.crank.Revolute(r3.NewVec(r, 0, 0), z, []*decad.Body{fb.coupler})
	require.NoError(t, err)
	fb.follow, err = fb.linkage.Ground().Revolute(r3.NewVec(g, 0, 0), z, []*decad.Body{fb.foll})
	require.NoError(t, err)
	fb.loop, err = fb.linkage.Close(fb.couplerLk, fb.follow, r3.NewVec(fb.b[0], fb.b[1], 0), z)
	require.NoError(t, err)
	return fb
}

// crankDrive turns the crank from 0 to to.
func (fb fourBar) crankDrive(to units.Value) decad.Drive {
	return decad.Drive{{Link: fb.crank, From: units.Degrees(0), To: to}}
}

package zzclipdiff

import (
	"context"
	"fmt"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"math"
	"os"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

var capLimits = []int64{0, 1, 2, 3, 4, 100}

func capXforms() []r3.Transform {
	refl, _ := r3.Reflection(mustFrame(vec(0, 0, 0), ex, ez))
	return []r3.Transform{r3.Identity(), rotation(ex, 0.4), rotation(vec(1, 2, 3), 1.1),
		then(rotation(ey, 0.2), translation(1, -2, 3)), translation(5, 0, 0),
		rigid(mustFrame(vec(0, 0, 0), ey, neg(ex))), refl}
}

// capPlanes lists planes whose unit normal components are one shared float
// times small integers, so the exact rational cap coefficients are dyadic and
// the cap line's endpoints are exactly representable.
var capPlanes = []struct{ n, u r3.Vec }{
	{vec(1, 0, 1), vec(0, 1, 0)}, {vec(1, 0, 2), vec(0, 1, 0)}, {vec(2, 0, 1), vec(0, 1, 0)},
	{vec(0, 1, 1), vec(1, 0, 0)}, {vec(0, 1, 2), vec(1, 0, 0)}, {vec(0, -1, 1), vec(1, 0, 0)},
	{vec(-1, 0, 1), vec(0, 1, 0)}, {vec(1, 1, 2), vec(1, -1, 0)}, {vec(1, 1, 1), vec(1, -1, 0)},
	{vec(1, -1, 2), vec(1, 1, 0)}, {vec(1, 1, 4), vec(1, -1, 0)}, {vec(0, 0, 1), vec(1, 0, 0)},
	{vec(1, 0, 0), vec(0, 1, 0)}, {vec(1, 0, 1), vec(0, 0, 1)},
}

func capPlane(i int, origin r3.Vec) r3.Frame {
	pl := capPlanes[i]
	return mustFrame(origin, pl.u, pl.n.Cross(pl.u))
}

func gridStride() int {
	if s := os.Getenv("GRID_STRIDE"); s != "" {
		n, _ := strconv.Atoi(s)
		return n
	}
	return 1
}

func TestCapDyadicGrid(t *testing.T) {
	c := newTally(t)
	stride := gridStride()
	xfs := []r3.Transform{r3.Identity(), translation(1, -2, 3), rigid(mustFrame(vec(0, 0, 0), ey, neg(ex))), rotation(ex, 0.4)}
	frames := []r3.Frame{mustFrame(vec(0, 0, 0), ex, ey), mustFrame(vec(1, 2, 3), ex, ey)}
	n := 0
	for pi := range capPlanes {
		for xi, xf := range xfs {
			for fi, f := range frames {
				for _, r := range []float64{1, 4, 0.5} {
					for _, kz := range []float64{1, 4} {
						for ci, cen := range [][2]float64{{0, 0}, {r / 2, -r / 4}} {
							pp := capPayload(cen[0], cen[1], r, 0, kz*r, f, xf)
							for step := -16; step <= int(kz*8)+16; step++ {
								n++
								if n%stride != 0 {
									continue
								}
								oz := float64(step) * r / 8
								ox, oy := float64(step%3)*r/4, float64(step%5)*r/8
								pl := capPlane(pi, vec(ox, oy, oz))
								cmpCap(c, fmt.Sprint("dy p", pi, "x", xi, "f", fi, "r", r, "k", kz, "c", ci, "s", step), pp, pl, []int64{100})
							}
						}
					}
				}
			}
		}
	}
	c.report()
}

// TestCapBoundaryBisect scans the plane origin height for admitted/refused
// flips, bisects each to adjacent floats and compares around both.
func TestCapBoundaryBisect(t *testing.T) {
	c := newTally(t)
	xfs := []r3.Transform{r3.Identity(), translation(1, -2, 3)}
	f := mustFrame(vec(0, 0, 0), ex, ey)
	flips := 0
	for _, pi := range []int{0, 1, 7, 9} {
		for xi, xf := range xfs {
			for _, r := range []float64{1} {
				for _, kz := range []float64{2} {
					pp := capPayload(r/2, -r/4, r, 0, kz*r, f, xf)
					at := func(oz float64) r3.Frame { return capPlane(pi, vec(0.25, 0.5, oz)) }
					admitted := func(oz float64) bool {
						_, _, k := oldObliqueCircleCapClip(t.Context(), pp, at(oz))
						return k
					}
					step := r / 4
					lo, hi := -4*r, kz*r+4*r
					prevS := lo
					prevOK := admitted(prevS)
					for s := lo + step; s <= hi; s += step {
						ok := admitted(s)
						if ok != prevOK {
							good, bad := prevS, s
							if !prevOK {
								good, bad = s, prevS
							}
							if a, b, found := bisectFlip(good, bad, admitted); found {
								flips++
								for _, e := range []float64{prev(a), a, next(a), prev(b), b, next(b)} {
									lims := []int64{100}
									if e == a || e == b {
										lims = capLimits
									}
									cmpCap(c, fmt.Sprint("bisect p", pi, "x", xi, "r", r, "k", kz, e), pp, at(e), lims)
								}
							}
						}
						prevS, prevOK = s, ok
					}
				}
			}
		}
	}
	if flips == 0 {
		t.Fatal("boundary scan found no admission/refusal flips")
	}
	t.Logf("flips=%d", flips)
	c.report()
}

func TestCapGuards(t *testing.T) {
	c := newTally(t)
	f := mustFrame(vec(0, 0, 0), ex, ey)
	xf := rotation(ex, 0.4)
	// Find an admitted setup by scanning the plane height over dyadic planes.
	var basePP prismPayload
	var basePlane r3.Frame
	found := false
	for pi := range capPlanes {
		for step := -16; step <= 48 && !found; step++ {
			pp := capPayload(0.5, -0.25, 1, 0, 2, f, xf)
			pl := capPlane(pi, vec(0, 0, float64(step)/8))
			if _, _, k := oldObliqueCircleCapClip(t.Context(), pp, pl); k {
				basePP, basePlane, found = pp, pl, true
			}
		}
	}
	if !found {
		t.Fatal("no admitted base case")
	}
	base := func() prismPayload { return basePP }
	nan, inf := math.NaN(), math.Inf(1)
	planes := []r3.Frame{basePlane, capPlane(0, basePlane.Origin().Add(vec(0, 0, 0.125))),
		capPlane(0, basePlane.Origin().Add(vec(0, 0, -4))), capPlane(0, basePlane.Origin().Add(vec(0, 0, 4))),
		capPlane(11, basePlane.Origin()), capPlane(12, basePlane.Origin()), mustFrame(vec(0, 0, 1), ex, ey)}
	muts := map[string]func(*prismPayload){
		"base":       func(*prismPayload) {},
		"hole":       func(p *prismPayload) { p.Profile.Holes = []loopRecord{circleLoop(0, 0, 1, false)} },
		"hole-empty": func(p *prismPayload) { p.Profile.Holes = []loopRecord{} },
		"two-seg": func(p *prismPayload) {
			p.Profile.Outer.Segments = append(p.Profile.Outer.Segments, p.Profile.Outer.Segments[0])
		},
		"no-seg":      func(p *prismPayload) { p.Profile.Outer.Segments = nil },
		"blend":       func(p *prismPayload) { p.BlendKind = "fillet" },
		"blendsegs":   func(p *prismPayload) { p.BlendSegs = []map[int]struct{}{{0: {}}} },
		"blendsegs0":  func(p *prismPayload) { p.BlendSegs = []map[int]struct{}{} },
		"z-equal":     func(p *prismPayload) { p.Z1 = p.Z0 },
		"z-rev":       func(p *prismPayload) { p.Z0, p.Z1 = p.Z1, p.Z0 },
		"z-next":      func(p *prismPayload) { p.Z1 = next(p.Z0) },
		"line":        func(p *prismPayload) { p.Profile.Outer.Segments[0] = lineRec(planePoint{}, planePoint{U: 1}) },
		"cw":          func(p *prismPayload) { p.Profile.Outer.Segments[0] = circleRec(0.5, -0.25, 1, false) },
		"tstart":      func(p *prismPayload) { p.Profile = circleSegT(1, 0.25, 1) },
		"tend":        func(p *prismPayload) { p.Profile = circleSegT(1, 0, 0.75) },
		"tend-over":   func(p *prismPayload) { p.Profile = circleSegT(1, 0, next(1)) },
		"tstart-neg":  func(p *prismPayload) { p.Profile = circleSegT(1, prev(0), 1) },
		"radius-zero": func(p *prismPayload) { p.Profile = circleProfile(0.5, -0.25, 0) },
		"radius-neg":  func(p *prismPayload) { p.Profile = circleProfile(0.5, -0.25, -5) },
		"radius-nan":  func(p *prismPayload) { p.Profile = circleProfile(0.5, -0.25, nan) },
		"radius-inf":  func(p *prismPayload) { p.Profile = circleProfile(0.5, -0.25, inf) },
		"radius-tiny": func(p *prismPayload) { p.Profile = circleProfile(0.5, -0.25, 5e-324) },
		"radius-max":  func(p *prismPayload) { p.Profile = circleProfile(0.5, -0.25, math.MaxFloat64) },
		"radius-kind": func(p *prismPayload) {
			cs := circleRec(0.5, -0.25, 1, true)
			cs.Radius = units.Radians(5)
			p.Profile.Outer.Segments[0] = cs
		},
		"radius-cm": func(p *prismPayload) {
			cs := circleRec(0.5, -0.25, 1, true)
			cs.Radius = units.Centimeters(0.5)
			p.Profile.Outer.Segments[0] = cs
		},
		"center-nan":  func(p *prismPayload) { p.Profile = circleProfile(nan, -0.25, 5) },
		"center-inf":  func(p *prismPayload) { p.Profile = circleProfile(0.5, inf, 5) },
		"center-max":  func(p *prismPayload) { p.Profile = circleProfile(math.MaxFloat64, 0, 5) },
		"z0-nan":      func(p *prismPayload) { p.Z0 = nan },
		"z1-inf":      func(p *prismPayload) { p.Z1 = inf },
		"z-max":       func(p *prismPayload) { p.Z1 = math.MaxFloat64 },
		"z0-neg-max":  func(p *prismPayload) { p.Z0 = -math.MaxFloat64 },
		"xform-zero":  func(p *prismPayload) { p.Xform = r3.Transform{} },
		"xform-id":    func(p *prismPayload) { p.Xform = r3.Identity() },
		"xform-far":   func(p *prismPayload) { p.Xform = translation(1e300, 0, 0) },
		"frame-zero":  func(p *prismPayload) { p.Frame = r3.Frame{} },
		"frame-off":   func(p *prismPayload) { p.Frame = mustFrame(vec(1e300, 0, 0), ex, ey) },
		"center-off":  func(p *prismPayload) { p.Profile = circleProfile(3, 4, 5) },
		"secdelta":    func(p *prismPayload) { p.SectionDelta = 1 },
		"z-deltas":    func(p *prismPayload) { p.Z0Delta, p.Z1Delta = 1, 1 },
		"profile-big": func(p *prismPayload) { p.Profile = circleProfile(0, 0, 1e200) },
	}
	for name, m := range muts {
		pp := base()
		pp.Profile.Outer.Segments = append([]curveSegment(nil), pp.Profile.Outer.Segments...)
		m(&pp)
		for pi, pl := range planes {
			cmpCap(c, fmt.Sprint(name, pi), pp, pl, capLimits)
		}
	}
	// A pre-cancelled context.
	ctx, cancel := context.WithCancel(t.Context())
	cancel()
	for pi, pl := range planes {
		pp := base()
		o := safely(func() any { p, d, k := oldObliqueCircleCapClip(ctx, pp, pl); return capOut{p, d, k} })
		n := safely(func() any { p, d, k := sectionclip.ObliqueCircleCapClip(ctx, pp, pl); return capOut{p, d, k} })
		c.same("ObliqueCircleCapClip", fmt.Sprint("canceled", pi), false, o, n)
	}
	c.report()
}

func circleSegT(r, ts, te float64) profileRecord {
	cs := circleRec(0.5, -0.25, r, true)
	cs.TStart, cs.TEnd = ts, te
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{cs}}}
}

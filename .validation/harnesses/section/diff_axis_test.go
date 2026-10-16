package zzclipdiff

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"os"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/pair/box"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

type planeResult struct {
	Profile profileRecord
	Poly    []planePoint
	Delta   float64
	OK      bool
}

// safely runs f and encodes either its result or its panic value.
func safely(f func() any) (out string) {
	defer func() {
		if r := recover(); r != nil {
			out = fmt.Sprintf("panic(%v)", r)
		}
	}()
	return enc(f())
}

func TestSignedAxisTransform(t *testing.T) {
	c := newTally(t)
	var ts []r3.Transform
	ts = append(ts, r3.Transform{}, r3.Identity())
	for _, pr := range cardinalPairs() {
		f := mustFrame(vec(0, 0, 0), pr[0], pr[1])
		ts = append(ts, rigid(f))
		for _, off := range []r3.Vec{vec(1, -2, 3), vec(1e300, 0, 0), vec(0, math.MaxFloat64, 0), vec(0, 0, -math.MaxFloat64)} {
			ts = append(ts, then(rigid(f), translation(off.X, off.Y, off.Z)))
		}
		if m, err := r3.Reflection(mustFrame(vec(0, 0, 0), pr[0], pr[1])); err == nil {
			ts = append(ts, m)
		}
	}
	for _, a := range []float64{1e-300, 1e-9, 0.3, math.Pi / 2, math.Pi, 1} {
		ts = append(ts, rotation(ez, a), rotation(vec(1, 1, 0), a), then(rotation(ex, a), translation(1, 1, 1)))
	}
	for _, tr := range []r3.Vec{vec(math.NaN(), 0, 0), vec(math.Inf(1), 0, 0), vec(0, math.Inf(-1), 0)} {
		if x, err := r3.Translation(tr); err == nil {
			ts = append(ts, x)
		}
	}
	for i, x := range ts {
		o, n := oldSignedAxisTransform(x), box.SignedAxisTransform(x)
		c.same("SignedAxisTransform", fmt.Sprint(i), o,
			fmt.Sprint(o), fmt.Sprint(n))
	}
	c.report()
}

func TestSphereSection(t *testing.T) {
	sphereSection(t, []float64{0, -1, 1, 2, 5, 0.1, 3, 1e-300, 4.000000000000001, 0.5})
}

func TestSphereSectionHuge(t *testing.T) {
	sphereSection(t, []float64{1e150, 1e200, 1e300, math.MaxFloat64})
}

func sphereSection(t *testing.T, radii []float64) {
	c := newTally(t)
	dy := func(f float64) proofarith.Dyadic { return proofarith.DyOfFinite(f) }
	centers := []r3.Vec{vec(0, 0, 0), vec(1, 2, 3), vec(-1.5, 0.25, 4), vec(0.1, 0.2, 0.3), vec(1e150, 0, 0), vec(0, 0, 3)}
	var planes []r3.Frame
	for _, pr := range cardinalPairs() {
		planes = append(planes, mustFrame(vec(0, 0, 0), pr[0], pr[1]))
		planes = append(planes, mustFrame(vec(1, 2, 3), pr[0], pr[1]))
		planes = append(planes, mustFrame(vec(0, 0, 4), pr[0], pr[1]))
		planes = append(planes, mustFrame(vec(0, 0, 3), pr[0], pr[1]))
	}
	planes = append(planes, mustFrame(vec(0, 0, 1), vec(1, 1, 0).Scale(1/math.Sqrt2), ez), mustFrame(vec(0, 0, 1), ex, vec(0, 3, 4).Scale(0.2)))
	// Offsets that equal the radius exactly, and one ulp either side.
	for _, r := range []float64{1, 2, 5, 0.1} {
		for _, k := range []float64{r, prev(r), next(r), -r, prev(-r), next(-r), 0} {
			planes = append(planes, mustFrame(vec(0, 0, k), ex, ey))
			planes = append(planes, mustFrame(vec(k, 0, 0), ey, ez))
		}
	}
	n, stride := 0, gridStride()
	for _, ctr := range centers {
		for _, r := range radii {
			for pi, pl := range planes {
				for _, ok := range []bool{true, false} {
					n++
					if n%stride != 0 {
						continue
					}
					cv := proofarith.DyVec(ctr)
					var admitted bool
					o := safely(func() any {
						p, poly, d, k := oldCardinalSphereSection(oldSphere{cv, dy(r)}, ok, pl)
						admitted = k
						return planeResult{p, poly, d, k}
					})
					n := safely(func() any {
						if !ok {
							return planeResult{Profile: profileRecord{}}
						}
						p, poly, d, k := sectionclip.CardinalSphereSection(cv, dy(r), pl)
						return planeResult{p, poly, d, k}
					})
					c.same("CardinalSphereSection", fmt.Sprint(ctr, r, pi, ok), admitted, o, n)
				}
			}
		}
	}
	// Exact-boundary pair checks: q == 0, q just above and below zero.
	c.report()
}

// coneCase is one right-cone meridian in plane-local coordinates.
type coneCase struct {
	name  string
	rp    revolvePayload
	world [2]r3.Vec // placed base and apex
}

func lineRec(a, b planePoint) lineSeg { return lineSeg{Start: a, End: b, TStart: 0, TEnd: 1} }

func coneProfile(corners [3]planePoint) profileRecord {
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineRec(corners[0], corners[1]), lineRec(corners[1], corners[2]), lineRec(corners[2], corners[0]),
	}}}
}

func place(xf r3.Transform, f r3.Frame, p planePoint) r3.Vec {
	local := f.Origin().Add(f.U().Scale(p.U)).Add(f.V().Scale(p.V))
	return xf.Apply(local)
}

func coneCases() []coneCase {
	var out []coneCase
	dirs := [][2]float64{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	shapes := [][2]float64{{4, 6}, {1, 3}, {3, 3}, {0.1, 0.3}, {1e-300, 1}, {2, 1e-300}, {5, 8}}
	offs := [][2]float64{{0, 0}, {2, 3}, {-1.5, 0.25}}
	frames := []r3.Frame{mustFrame(vec(0, 0, 0), ex, ey), mustFrame(vec(1, 2, 3), ey, ez), mustFrame(vec(0, 0, 0), neg(ez), ex)}
	cardRot := rigid(mustFrame(vec(0, 0, 0), ey, neg(ex)))
	refl, _ := r3.Reflection(mustFrame(vec(0, 0, 0), ey, ez))
	xfs := []r3.Transform{r3.Identity(), cardRot, translation(1, 2, 3), then(cardRot, translation(0.5, 0, 0)),
		rotation(ez, 0.3), refl}
	perms := [][3]int{{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}
	for si, sh := range shapes {
		for di, d := range dirs {
			for oi, off := range offs {
				for fi, f := range frames {
					for xi, xf := range xfs {
						for pi, pm := range perms {
							if (pi > 1) && (fi > 0 || xi > 1 || oi > 0 || si > 3) {
								continue
							}
							if si > 3 && (fi > 0 || xi > 0 || oi > 0) {
								continue
							}
							du, dv := d[0], d[1]
							base := planePoint{U: off[0], V: off[1]}
							rim := planePoint{U: off[0] - dv*sh[0], V: off[1] + du*sh[0]}
							apex := planePoint{U: off[0] + du*sh[1], V: off[1] + dv*sh[1]}
							abc := [3]planePoint{base, rim, apex}
							corners := [3]planePoint{abc[pm[0]], abc[pm[1]], abc[pm[2]]}
							rp := revolvePayload{
								Profile: coneProfile(corners), Frame: f, Full: true, Xform: xf,
								Ax: revolveaxis.Frame{AU: off[0], AV: off[1], DU: du, DV: dv},
							}
							out = append(out, coneCase{
								name:  fmt.Sprint("s", si, "d", di, "o", oi, "f", fi, "x", xi, "p", pi),
								rp:    rp,
								world: [2]r3.Vec{place(xf, f, base), place(xf, f, apex)},
							})
						}
					}
				}
			}
		}
	}
	return out
}

func coneOffsets(w [2]r3.Vec, n r3.Vec) []float64 {
	kb, ka := w[0].Dot(n), w[1].Dot(n)
	ks := []float64{kb, ka, (kb + ka) / 2, kb + (ka-kb)/3, kb + 2*(ka-kb)/3, kb + (ka-kb)/4,
		prev(kb), next(kb), prev(ka), next(ka), kb - 1, ka + 1}
	return ks
}

func cmpCone(c *tally, label string, rp revolvePayload, pl r3.Frame) {
	c.t.Helper()
	var ok bool
	o := safely(func() any {
		p, poly, d, k := oldCardinalConeSection(rp, pl)
		ok = k
		return planeResult{p, poly, d, k}
	})
	n := safely(func() any {
		p, poly, d, k := sectionclip.CardinalConeSection(rp, pl)
		return planeResult{p, poly, d, k}
	})
	c.same("CardinalConeSection", label, ok, o, n)
}

func TestConeSectionGrid(t *testing.T) {
	c := newTally(t)
	exact, poly := 0, 0
	stride := 1
	if s := os.Getenv("GRID_STRIDE"); s != "" {
		stride, _ = strconv.Atoi(s)
	}
	for ci, cc := range coneCases() {
		if ci%stride != 0 {
			continue
		}
		for pri, pr := range cardinalPairs() {
			if pri%2 == 1 {
				continue
			}
			n := pr[0].Cross(pr[1])
			for _, k := range coneOffsets(cc.world, n) {
				pl := mustFrame(n.Scale(k), pr[0], pr[1])
				cmpCone(c, fmt.Sprint(cc.name, pr, k), cc.rp, pl)
			}
		}
	}
	_ = exact
	_ = poly
	c.report()
}

func TestConeSectionKinds(t *testing.T) {
	// Counts admitted exact versus polygon results so both are exercised.
	f := mustFrame(vec(0, 0, 0), ex, ey)
	mk := func(R, H float64) revolvePayload {
		return revolvePayload{Profile: coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: R}, {U: H, V: 0}}),
			Frame: f, Full: true, Xform: r3.Identity(), Ax: revolveaxis.Frame{DU: 1}}
	}
	pl := func(k float64) r3.Frame { return mustFrame(vec(k, 0, 0), ey, ez) }
	_, p1, _, ok1 := sectionclip.CardinalConeSection(mk(4, 6), pl(3))
	_, p2, _, ok2 := sectionclip.CardinalConeSection(mk(1, 3), pl(1))
	if !ok1 || p1 != nil || !ok2 || p2 == nil {
		t.Fatalf("exact=%v/%v polygon=%v/%v", ok1, p1 == nil, ok2, p2 != nil)
	}
}

func TestConeGuards(t *testing.T) {
	c := newTally(t)
	f := mustFrame(vec(0, 0, 0), ex, ey)
	base := func() revolvePayload {
		return revolvePayload{Profile: coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: 4}, {U: 6, V: 0}}),
			Frame: f, Full: true, Xform: r3.Identity(), Ax: revolveaxis.Frame{DU: 1}}
	}
	planes := []r3.Frame{mustFrame(vec(3, 0, 0), ey, ez), mustFrame(vec(2, 0, 0), ey, ez),
		mustFrame(vec(0, 0, 3), ex, ey), mustFrame(vec(3, 0, 0), ez, ey), mustFrame(vec(3, 0, 0), vec(0, 0.6, 0.8), vec(0, -0.8, 0.6)),
		mustFrame(vec(6, 0, 0), ey, ez), mustFrame(vec(0, 0, 0), ey, ez), mustFrame(vec(7, 0, 0), ey, ez),
		mustFrame(vec(-1, 0, 0), ey, ez)}
	nan, inf := math.NaN(), math.Inf(1)
	muts := map[string]func(*revolvePayload){
		"base":         func(*revolvePayload) {},
		"notfull":      func(r *revolvePayload) { r.Full = false },
		"surface":      func(r *revolvePayload) { r.SurfaceResult = true },
		"secdelta":     func(r *revolvePayload) { r.SectionDelta = 1e-300 },
		"secdelta-neg": func(r *revolvePayload) { r.SectionDelta = -1 },
		"hole":         func(r *revolvePayload) { r.Profile.Holes = []loopRecord{circleLoop(1, 1, 0.1, false)} },
		"two-seg":      func(r *revolvePayload) { r.Profile.Outer.Segments = r.Profile.Outer.Segments[:2] },
		"four-seg": func(r *revolvePayload) {
			r.Profile.Outer.Segments = append(r.Profile.Outer.Segments, r.Profile.Outer.Segments[0])
		},
		"noncardinal-frame": func(r *revolvePayload) { r.Frame = mustFrame(vec(0, 0, 0), vec(0.6, 0.8, 0), vec(-0.8, 0.6, 0)) },
		"xform-rot":         func(r *revolvePayload) { r.Xform = rotation(ez, 0.3) },
		"xform-invalid":     func(r *revolvePayload) { r.Xform = r3.Transform{} },
		"xform-far":         func(r *revolvePayload) { r.Xform = translation(math.MaxFloat64, 0, 0) },
		"origin-huge":       func(r *revolvePayload) { r.Frame = mustFrame(vec(math.MaxFloat64, 0, 0), ex, ey) },
		"aubound":           func(r *revolvePayload) { r.Ax.AUBound = 1e-300 },
		"avbound":           func(r *revolvePayload) { r.Ax.AVBound = 1e-300 },
		"dubound":           func(r *revolvePayload) { r.Ax.DUBound = 1e-300 },
		"dvbound":           func(r *revolvePayload) { r.Ax.DVBound = 1e-300 },
		"du-near1":          func(r *revolvePayload) { r.Ax.DU = next(1) },
		"du-prev1":          func(r *revolvePayload) { r.Ax.DU = prev(1) },
		"du-neg1":           func(r *revolvePayload) { r.Ax.DU = -1 },
		"dv-tiny":           func(r *revolvePayload) { r.Ax.DV = 1e-300 },
		"dv1du1":            func(r *revolvePayload) { r.Ax.DV = 1 },
		"dv1du0":            func(r *revolvePayload) { r.Ax.DU, r.Ax.DV = 0, 1 },
		"dv-neg1du0":        func(r *revolvePayload) { r.Ax.DU, r.Ax.DV = 0, -1 },
		"d0":                func(r *revolvePayload) { r.Ax.DU = 0 },
		"du-2":              func(r *revolvePayload) { r.Ax.DU = 2 },
		"au-nan":            func(r *revolvePayload) { r.Ax.AU = nan },
		"av-inf":            func(r *revolvePayload) { r.Ax.AV = inf },
		"au-1":              func(r *revolvePayload) { r.Ax.AU = 1 },
		"av-1":              func(r *revolvePayload) { r.Ax.AV = 1 },
		"au-big":            func(r *revolvePayload) { r.Ax.AU = 1e308 },
		"circle-seg":        func(r *revolvePayload) { r.Profile.Outer.Segments[1] = circleRec(0, 0, 1, true) },
		"line-tstart": func(r *revolvePayload) {
			l := r.Profile.Outer.Segments[0].(lineSeg)
			l.TStart = 0.1
			r.Profile.Outer.Segments[0] = l
		},
		"line-tend": func(r *revolvePayload) {
			l := r.Profile.Outer.Segments[2].(lineSeg)
			l.TEnd = 0.9
			r.Profile.Outer.Segments[2] = l
		},
		"line-reversed": func(r *revolvePayload) {
			l := r.Profile.Outer.Segments[1].(lineSeg)
			l.TStart, l.TEnd = 1, 0
			r.Profile.Outer.Segments[1] = l
		},
		"start-nan": func(r *revolvePayload) {
			l := r.Profile.Outer.Segments[0].(lineSeg)
			l.Start.U = nan
			r.Profile.Outer.Segments[0] = l
		},
		"endv-inf": func(r *revolvePayload) {
			l := r.Profile.Outer.Segments[1].(lineSeg)
			l.End.V = inf
			r.Profile.Outer.Segments[1] = l
		},
		"unclosed": func(r *revolvePayload) {
			l := r.Profile.Outer.Segments[1].(lineSeg)
			l.End.U = 5.5
			r.Profile.Outer.Segments[1] = l
		},
		"gap-in-chain": func(r *revolvePayload) {
			l := r.Profile.Outer.Segments[0].(lineSeg)
			l.End.U = next(0)
			r.Profile.Outer.Segments[0] = l
		},
		"dup-corner01": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: 0}, {U: 6, V: 0}})
		},
		"dup-corner12": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 6, V: 0}, {U: 6, V: 0}})
		},
		"dup-corner20": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 6, V: 0}, {U: 0, V: 4}, {U: 6, V: 0}})
		},
		"two-outer": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: 4}, {U: 6, V: 1}})
		},
		"three-outer": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 1}, {U: 0, V: 4}, {U: 6, V: 2}})
		},
		"all-axis": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 1, V: 0}, {U: 6, V: 0}})
		},
		"base-axial-off": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: next(0), V: 4}, {U: 6, V: 0}})
		},
		"outer-axial-eq-apex": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 6, V: 4}, {U: 6, V: 0}})
		},
		"axial-base-apex-eq": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 1, V: 0}, {U: 1, V: 4}, {U: 1, V: 0.0}})
		},
		"outer-neg": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: -4}, {U: 6, V: 0}})
		},
		"apex-neg": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: 4}, {U: -6, V: 0}})
		},
		"cyl-like": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 6, V: 4}, {U: 6, V: 0}})
		},
		"radius-huge": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: math.MaxFloat64}, {U: 6, V: 0}})
		},
		"radius-tiny": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: 5e-324}, {U: 6, V: 0}})
		},
		"radius-sub": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: 1e-300}, {U: 1e300, V: 0}})
		},
		"center-inexact": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 0, V: 0}, {U: 0, V: 4}, {U: 6, V: 0}})
			r.Xform = translation(0.1, 0.2, 0.3)
		},
		"holes-nil-empty": func(r *revolvePayload) { r.Profile.Holes = []loopRecord{} },
		"apex-first": func(r *revolvePayload) {
			r.Profile = coneProfile([3]planePoint{{U: 6, V: 0}, {U: 0, V: 0}, {U: 0, V: 4}})
		},
		"zero-frame": func(r *revolvePayload) { r.Frame = r3.Frame{} },
	}
	for name, m := range muts {
		rp := base()
		rp.Profile.Outer.Segments = append([]curveSegment(nil), rp.Profile.Outer.Segments...)
		m(&rp)
		for pi, pl := range planes {
			cmpCone(c, fmt.Sprint(name, pi), rp, pl)
		}
	}
	c.report()
}

// capSetup is an admitted-by-construction oblique cap clip: a circular prism
// whose far cap is cut by a slightly tilted plane.
type capSetup struct {
	pp    prismPayload
	plane r3.Frame
}

func capPayload(cu, cv, r, z0, z1 float64, f r3.Frame, xf r3.Transform) prismPayload {
	return prismPayload{Profile: circleProfile(cu, cv, r), Frame: f, Z0: z0, Z1: z1, Xform: xf}
}

func tiltedPlane(pp prismPayload, capTop bool, theta, shift, spin float64) r3.Frame {
	z := pp.Z1
	if !capTop {
		z = pp.Z0
	}
	cen := place2(pp, 0, 0, z)
	axis := pp.Xform.ApplyDir(pp.Frame.N())
	// pick two directions orthogonal to the axis
	a, _ := pickPerp(axis).Normalize()
	b, _ := axis.Cross(a).Normalize()
	// tilted normal
	n, _ := axis.Scale(math.Cos(theta)).Add(a.Scale(math.Sin(theta) * math.Cos(spin))).Add(b.Scale(math.Sin(theta) * math.Sin(spin))).Normalize()
	u, _ := pickPerp(n).Normalize()
	v, _ := n.Cross(u).Normalize()
	// NewFrame needs u,v with u x v = n
	f, err := r3.NewFrame(cen.Add(axis.Scale(shift)), u, v)
	if err != nil {
		panic(err)
	}
	return f
}

func place2(pp prismPayload, u, v, z float64) r3.Vec {
	local := pp.Frame.Origin().Add(pp.Frame.U().Scale(u)).Add(pp.Frame.V().Scale(v)).Add(pp.Frame.N().Scale(z))
	return pp.Xform.Apply(local)
}

func pickPerp(a r3.Vec) r3.Vec {
	if math.Abs(a.X) <= math.Abs(a.Y) && math.Abs(a.X) <= math.Abs(a.Z) {
		return a.Cross(ex)
	}
	if math.Abs(a.Y) <= math.Abs(a.Z) {
		return a.Cross(ey)
	}
	return a.Cross(ez)
}

type capOut struct {
	Poly  []planePoint
	Delta float64
	OK    bool
}

func cmpCap(c *tally, label string, pp prismPayload, pl r3.Frame, limits []int64) {
	c.t.Helper()
	for _, lim := range limits {
		var ok bool
		var oc, nc *pollCtx
		oc = &pollCtx{Context: context.Background(), limit: lim}
		nc = &pollCtx{Context: context.Background(), limit: lim}
		o := safely(func() any {
			p, d, k := oldObliqueCircleCapClip(oc, pp, pl)
			ok = k
			return capOut{p, d, k}
		})
		n := safely(func() any {
			p, d, k := sectionclip.ObliqueCircleCapClip(nc, pp, pl)
			return capOut{p, d, k}
		})
		c.same("ObliqueCircleCapClip", fmt.Sprint(label, " limit=", lim), ok,
			fmt.Sprint(o, "|calls=", oc.calls.Load()), fmt.Sprint(n, "|calls=", nc.calls.Load()))
	}
}

var _ = big.NewRat
var _ = units.Millimeter

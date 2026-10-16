package decad

// Validation-only differential: zzold_oblique.go holds the base root
// bodies renamed old*; every test compares them with internal/obliqueboss.

import (
	"context"
	"errors"
	"fmt"
	"math"
	"os"
	"reflect"
	"testing"

	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

type zzCtx struct {
	context.Context
	n      int
	failAt int
}

func (c *zzCtx) Err() error {
	c.n++
	if c.failAt > 0 && c.n >= c.failAt {
		return context.Canceled
	}
	return nil
}

func zzErrEq(t *testing.T, name string, a, b error) {
	t.Helper()
	if (a == nil) != (b == nil) {
		t.Fatalf("%s: error presence differs: old %v new %v", name, a, b)
	}
	if a == nil {
		return
	}
	if a.Error() != b.Error() {
		t.Fatalf("%s: error text differs:\n old %q\n new %q", name, a.Error(), b.Error())
	}
	for _, target := range []error{ErrUnsupported, ErrBooleanFailed, context.Canceled} {
		if errors.Is(a, target) != errors.Is(b, target) {
			t.Fatalf("%s: errors.Is(%v) differs", name, target)
		}
	}
}

func zzCatch(f func()) (msg string) {
	defer func() {
		if r := recover(); r != nil {
			msg = fmt.Sprint(r)
		}
	}()
	f()
	return ""
}

func zzBitsEq(a, b []r3.Vec) bool {
	if (a == nil) != (b == nil) || len(a) != len(b) {
		return false
	}
	for i := range a {
		if math.Float64bits(a[i].X) != math.Float64bits(b[i].X) ||
			math.Float64bits(a[i].Y) != math.Float64bits(b[i].Y) ||
			math.Float64bits(a[i].Z) != math.Float64bits(b[i].Z) {
			return false
		}
	}
	return true
}

var zzFast = os.Getenv("ZZ_FAST") != ""

func zzF(a, b float64) bool { return math.Float64bits(a) == math.Float64bits(b) }

// zzPlate builds the plate and boss source bodies of a configuration.
func zzSources(t *testing.T, x0, y0, x1, y1, thick, bossR, bossLen float64,
	center, tiltAxis r3.Vec, deg float64) (*Body, *Body) {
	t.Helper()
	ctx := t.Context()
	doc := New()
	w := sketch.NewWorld()
	ps, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatal(err)
	}
	rect := ps.CreateRectangle(x0, y0, x1, y1)
	ps.Fix(rect.A)
	if _, err := ps.Solve(ctx); err != nil {
		t.Fatal(err)
	}
	plate, err := Extrude(ctx, doc, ps, ps.Profiles()[0], Distance{D: units.Millimeters(thick), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}
	bs, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatal(err)
	}
	p := bs.CreatePoint(0, 0)
	bs.Fix(p)
	bs.CreateCircle(p, bossR)
	if _, err := bs.Solve(ctx); err != nil {
		t.Fatal(err)
	}
	boss, err := Extrude(ctx, doc, bs, bs.Profiles()[0], Distance{D: units.Millimeters(bossLen), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}
	turn, err := r3.RotationAround(r3.Vec{}, tiltAxis, units.Degrees(deg))
	if err != nil {
		t.Fatal(err)
	}
	shift, err := r3.Translation(center)
	if err != nil {
		t.Fatal(err)
	}
	pose, err := turn.Then(shift)
	if err != nil {
		t.Fatal(err)
	}
	boss, err = Transform(ctx, boss, pose)
	if err != nil {
		t.Fatal(err)
	}
	return plate, boss
}

type zzFx struct {
	name   string
	rec    *obliqueBossRecord
	radius float64
	plate  *Body
	boss   *Body
}

func zzNext(f float64) float64 { return math.Nextafter(f, math.Inf(1)) }
func zzPrev(f float64) float64 { return math.Nextafter(f, math.Inf(-1)) }

// zzSynth swaps a record's plate for another body and rereads the plate numbers.
func zzSynth(base *obliqueBossRecord, plate *Body) *obliqueBossRecord {
	r := *base
	pp := plate.n.Payload.(prismPayload)
	rect, _ := momentinput.AxisRectangle(pp.Profile)
	r.Plate, r.Rect, r.Bottom, r.Top = bodyNode(plate), rect, pp.Z0, pp.Z1
	return &r
}

func zzAxisSet(r *obliqueBossRecord, ax r3.Vec) {
	sinTilt := math.Hypot(ax.X, ax.Y)
	hx, hy := ax.X/sinTilt, ax.Y/sinTilt
	r.Axis = ax
	r.Ex = r3.Vec{X: ax.Z * hx, Y: ax.Z * hy, Z: -sinTilt}
	r.Ey = r3.Vec{X: -hy, Y: hx}
}

func zzFixtures(t *testing.T) []zzFx {
	t.Helper()
	if zzFast {
		t.Fatal("reduced differential fixture mode is forbidden")
	}
	var out []zzFx
	var admittedNames, omittedNames []string
	add := func(name string, rec *obliqueBossRecord, radius float64, plate, boss *Body) {
		out = append(out, zzFx{name, rec, radius, plate, boss})
	}
	type cfg struct {
		name                          string
		x0, y0, x1, y1, th, br, bl, d float64
		c, ax                         r3.Vec
	}
	cfgs := []cfg{
		{"y15", 0, 0, 20, 20, 5, 4, 6, 15, r3.Vec{X: 10, Y: 10, Z: 3.5}, r3.Vec{Y: 1}},
		{"x15", 0, 0, 20, 20, 5, 4, 6, 15, r3.Vec{X: 10, Y: 10, Z: 3.5}, r3.Vec{X: 1}},
		{"xy15", 0, 0, 20, 20, 5, 4, 6, 15, r3.Vec{X: 10, Y: 10, Z: 3.5}, r3.Vec{X: 1, Y: 1}},
		{"mxy10", -5, -6, 25, 18, 4, 3, 9, 10, r3.Vec{X: 9, Y: 7, Z: 2}, r3.Vec{X: -1, Y: -2}},
		{"mx19", 0, 0, 30, 14, 5, 3.5, 8, 19, r3.Vec{X: 14, Y: 7, Z: 3}, r3.Vec{X: -1}},
		{"y5", 0, 0, 24, 24, 3, 5, 7, 5, r3.Vec{X: 12, Y: 12, Z: 1}, r3.Vec{Y: -1}},
		{"offc", 0, 0, 40, 40, 5, 4, 6, 12, r3.Vec{X: 17, Y: 23, Z: 3.5}, r3.Vec{X: 2, Y: 1}},
		{"tall", 0, 0, 20, 20, 5, 2, 20, 15, r3.Vec{X: 10, Y: 10, Z: 3.5}, r3.Vec{Y: 1}},
	}
	for _, c := range cfgs {
		plate, boss := zzSources(t, c.x0, c.y0, c.x1, c.y1, c.th, c.br, c.bl, c.c, c.ax, c.d)
		rec := recognizeObliqueBoss(plate, boss)
		if rec == nil {
			omittedNames = append(omittedNames, c.name)
			t.Logf("fixture omitted: %s; source recognition refused", c.name)
			continue
		}
		admittedNames = append(admittedNames, c.name)
		for _, r := range []float64{0.1, 0.5, 0.99, 1.0} {
			add(fmt.Sprintf("%s/r%g", c.name, r), rec, r, plate, boss)
		}
	}
	t.Logf("fixture coverage: configured=%d admitted=%v omitted=%v radius_cases=%d",
		len(cfgs), admittedNames, omittedNames, len(out))
	for _, required := range []string{"y15", "x15", "xy15"} {
		found := false
		for _, admitted := range admittedNames {
			if admitted == required {
				found = true
				break
			}
		}
		if !found {
			t.Fatalf("required tilt fixture %s was omitted", required)
		}
	}
	return out
}

func zzOldMesh(m *obliqueboss.Mesh) *oldObliqueBossMesh {
	if m == nil {
		return nil
	}
	return &oldObliqueBossMesh{verts: m.Verts, tris: m.Tris, src: m.Src, groups: m.Groups}
}

func zzNewMesh(m *oldObliqueBossMesh) *obliqueboss.Mesh {
	if m == nil {
		return nil
	}
	return &obliqueboss.Mesh{Verts: m.verts, Tris: m.tris, Src: m.src, Groups: m.groups}
}

func zzCmpMesh(t *testing.T, name string, o *oldObliqueBossMesh, n *obliqueboss.Mesh) {
	t.Helper()
	if (o == nil) != (n == nil) {
		t.Fatalf("%s: mesh presence differs", name)
	}
	if o == nil {
		return
	}
	if !zzBitsEq(o.verts, n.Verts) {
		t.Fatalf("%s: vertices differ", name)
	}
	if !reflect.DeepEqual(o.tris, n.Tris) || !reflect.DeepEqual(o.src, n.Src) {
		t.Fatalf("%s: triangles or sources differ", name)
	}
	if len(o.groups) != len(n.Groups) {
		t.Fatalf("%s: group count differs", name)
	}
	for i := range o.groups {
		if !reflect.DeepEqual(o.groups[i], n.Groups[i]) {
			t.Fatalf("%s: group %d differs", name, i)
		}
	}
}

func zzFails() []int {
	if zzFast {
		return []int{0, 1, 2, 5}
	}
	return []int{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}
}

// zzBuild compares the mesh builders including every cancel point.
func zzBuild(t *testing.T, name string, rec *obliqueBossRecord, radius float64) (*oldObliqueBossMesh, bool) {
	t.Helper()
	var first *oldObliqueBossMesh
	for _, failAt := range zzFails() {
		oc, nc := &zzCtx{Context: t.Context(), failAt: failAt}, &zzCtx{Context: t.Context(), failAt: failAt}
		var om *oldObliqueBossMesh
		var nm *obliqueboss.Mesh
		var oerr, nerr error
		op := zzCatch(func() { om, oerr = oldBuildObliqueBossFilletMesh(oc, rec, radius) })
		np := zzCatch(func() { nm, nerr = obliqueboss.BuildFilletMesh(nc, rec, radius) })
		if op != np {
			t.Fatalf("%s: panic differs: old %q new %q", name, op, np)
		}
		if op != "" {
			return nil, false
		}
		zzErrEq(t, fmt.Sprintf("%s/build/cancel%d", name, failAt), oerr, nerr)
		if oc.n != nc.n {
			t.Fatalf("%s: ctx.Err calls differ at cancel %d: old %d new %d", name, failAt, oc.n, nc.n)
		}
		zzCmpMesh(t, fmt.Sprintf("%s/build/cancel%d", name, failAt), om, nm)
		if failAt == 0 {
			first = om
		}
	}
	return first, first != nil
}

type zzProveOut struct {
	a, b, c float64
	err     error
	n       int
	panic   string
}

func zzProve(t *testing.T, name string, rec *obliqueBossRecord, radius float64, om *oldObliqueBossMesh, cancels bool) {
	t.Helper()
	top := 0
	if cancels {
		top = 14
	}
	for failAt := 0; failAt <= top; failAt++ {
		oc, nc := &zzCtx{Context: t.Context(), failAt: failAt}, &zzCtx{Context: t.Context(), failAt: failAt}
		var o, n zzProveOut
		o.panic = zzCatch(func() { o.a, o.b, o.c, o.err = oldProveObliqueBossFillet(oc, rec, radius, om) })
		n.panic = zzCatch(func() { n.a, n.b, n.c, n.err = obliqueboss.ProveFillet(nc, rec, radius, zzNewMesh(om)) })
		if o.panic != n.panic {
			t.Fatalf("%s: prove panic differs: old %q new %q", name, o.panic, n.panic)
		}
		if o.panic != "" {
			return
		}
		zzErrEq(t, fmt.Sprintf("%s/prove/cancel%d", name, failAt), o.err, n.err)
		if oc.n != nc.n {
			t.Fatalf("%s: prove ctx.Err calls differ: old %d new %d", name, oc.n, nc.n)
		}
		if !zzF(o.a, n.a) || !zzF(o.b, n.b) || !zzF(o.c, n.c) {
			t.Fatalf("%s: prove results differ: old %v %v %v new %v %v %v", name, o.a, o.b, o.c, n.a, n.b, n.c)
		}
	}
}

func TestZZObliqueFaceGroupAndHasOrigin(t *testing.T) {
	ctx := t.Context()
	var roles = []string{provenance.RoleCapStart, provenance.RoleCapEnd, prismrecord.OuterFirstSideRole,
		"side(0,0)", "side(0,1)", "side(0,2)", "side(0,3)", "side(0,4)", "junctionFillet", "", "nope"}
	seen := map[*Body]bool{}
	for _, fx := range zzFixtures(t) {
		if seen[fx.plate] {
			// Radius variants share these source bodies; provenance is checked once.
			continue
		}
		seen[fx.plate] = true
		bodies := []*Body{fx.plate, fx.boss}
		var groups []facetGroup
		for bi := 0; bi < 3; bi++ {
			if bi == 2 {
				joined, err := Union(ctx, fx.plate, fx.boss)
				if err != nil {
					t.Fatal(err)
				}
				bodies = append(bodies, joined)
			}
			b := bodies[bi]
			for _, role := range roles {
				og, ook := oldObliqueBossFaceGroup(b, role)
				ng, nok := obliqueboss.FaceGroup(bodyNode(b), role)
				if ook != nok || !reflect.DeepEqual(og, ng) {
					t.Fatalf("%s body %d role %q: FaceGroup differs: old admitted=%v new admitted=%v", fx.name, bi, role, ook, nok)
				}
				if ook {
					groups = append(groups, og)
				}
			}
		}
		for _, b := range bodies {
			for _, f := range b.Faces() {
				for _, g := range groups {
					if oldObliqueBossHasOrigin(&f.n, g) != obliqueboss.HasOrigin(&f.n, g) {
						t.Fatalf("%s: HasOrigin differs", fx.name)
					}
				}
				if oldObliqueBossHasOrigin(&f.n, facetGroup{}) || obliqueboss.HasOrigin(&f.n, facetGroup{}) {
					t.Fatal("empty group matched")
				}
			}
		}
	}
	// A body with no faces, a bare node and a nil-payload node.
	empty := zzNode()
	if _, ok := obliqueboss.FaceGroup(empty, "x"); ok {
		t.Fatal("empty body has a group")
	}
}

func TestZZObliqueDiameterBound(t *testing.T) {
	bounds := []float64{0, 0.01, 0.1, 1, 2.5, -1, math.NaN(), math.Inf(1), math.Inf(-1), 1e300, 5e-324}
	for _, fx := range zzFixtures(t) {
		variants := []obliqueBossRecord{*fx.rec}
		v := *fx.rec
		v.Length, v.Radius = 1e300, 1e300
		variants = append(variants, v)
		v = *fx.rec
		v.Rect.U1 = math.Inf(1)
		variants = append(variants, v)
		v = *fx.rec
		v.Top = v.Bottom - 3
		variants = append(variants, v)
		v = *fx.rec
		v.Length, v.Radius = math.NaN(), 2
		variants = append(variants, v)
		for vi := range variants {
			for _, b := range bounds {
				var o, n float64
				op := zzCatch(func() { o = oldObliqueBossDiameterBound(&variants[vi], b) })
				np := zzCatch(func() { n = obliqueboss.DiameterBound(&variants[vi], b) })
				if op != np || !zzF(o, n) {
					t.Fatalf("%s v%d bound %v: old %v %q new %v %q", fx.name, vi, b, o, op, n, np)
				}
			}
		}
	}
}

func TestZZObliqueBuildAndProve(t *testing.T) {
	for _, fx := range zzFixtures(t) {
		om, ok := zzBuild(t, fx.name, fx.rec, fx.radius)
		if !ok {
			t.Fatalf("%s: build failed", fx.name)
		}
		zzProve(t, fx.name, fx.rec, fx.radius, om, fx.radius == 0.5)
	}
}

// zzRun builds with both builders and proves when a mesh results; fabricates a
// zero mesh when the builder refuses so the proof's own gates still compare.
func zzRun(t *testing.T, name string, rec *obliqueBossRecord, radius float64) {
	t.Helper()
	om, ok := zzBuild(t, name, rec, radius)
	if ok {
		zzProve(t, name+"/real", rec, radius, om, false)
		return
	}
	var plan []oldObliqueBossSample
	if msg := zzCatch(func() { plan = oldObliqueBossSamples(rec, radius) }); msg != "" {
		return
	}
	stride := 5 + oldObliqueBossBlendCells - 1
	zero := &oldObliqueBossMesh{verts: make([]r3.Vec, len(plan)*stride+2)}
	zzProve(t, name+"/zero", rec, radius, zero, false)
}

func TestZZObliqueProveMeshShapes(t *testing.T) {
	fx := zzBase(t)
	plan := oldObliqueBossSamples(fx.rec, fx.radius)
	stride := 5 + oldObliqueBossBlendCells - 1
	n := len(plan)*stride + 2
	for _, d := range []int{-3, -1, 0, 1, stride} {
		m := &oldObliqueBossMesh{verts: make([]r3.Vec, n+d)}
		zzProve(t, fmt.Sprintf("size%+d", d), fx.rec, fx.radius, m, d == 0)
	}
	for _, idx := range []int{0, 3, stride + 4, n - 2, n - 1} {
		for _, bad := range []r3.Vec{{X: math.NaN()}, {Y: math.Inf(1)}, {Z: math.Inf(-1)}} {
			m := &oldObliqueBossMesh{verts: make([]r3.Vec, n)}
			m.verts[idx] = bad
			zzProve(t, fmt.Sprintf("bad%d-%v", idx, bad), fx.rec, fx.radius, m, false)
		}
	}
	// nonzero triangles on the zero mesh and an empty triangle list
	m := &oldObliqueBossMesh{verts: make([]r3.Vec, n), tris: [][3]int{{0, 1, 2}}}
	zzProve(t, "tris", fx.rec, fx.radius, m, false)
}

func zzBase(t *testing.T) zzFx { return zzFixtures(t)[1] } // y15 r=0.5

func TestZZObliqueBasisBoundaries(t *testing.T) {
	fx := zzBase(t)
	ex := func(ax r3.Vec) (r3.Vec, r3.Vec) {
		r := *fx.rec
		zzAxisSet(&r, ax)
		return r.Ex, r.Ey
	}
	type v struct {
		name         string
		axis, ex, ey r3.Vec
	}
	var vs []v
	s := 0.05
	zc := math.Sqrt(1 - s*s)
	for _, z := range []float64{zzPrev(0.94), 0.94, zzNext(0.94)} {
		x := math.Sqrt(1 - z*z)
		a := r3.Vec{X: x, Z: z}
		e1, e2 := ex(a)
		vs = append(vs, v{fmt.Sprintf("axisZ-%v", z), a, e1, e2})
	}
	for _, x := range []float64{zzPrev(0.349), 0.349, zzNext(0.349)} {
		z := math.Sqrt(1 - x*x)
		a := r3.Vec{X: x, Z: z}
		e1, e2 := ex(a)
		vs = append(vs, v{fmt.Sprintf("tilt-%v", x), a, e1, e2})
		a = r3.Vec{Y: -x, Z: z}
		e1, e2 = ex(a)
		vs = append(vs, v{fmt.Sprintf("tiltY-%v", x), a, e1, e2})
	}
	for _, l := range []float64{0.999, 1.009} {
		for _, y := range []float64{zzPrev(l), l, zzNext(l)} {
			for _, sign := range []float64{1, -1} {
				a := r3.Vec{X: s, Z: zc}
				e1, _ := ex(a)
				vs = append(vs, v{fmt.Sprintf("ey-%v-%v", sign, y), a, e1, r3.Vec{Y: sign * y}})
				b := r3.Vec{Y: s, Z: zc}
				_, e2 := ex(b)
				vs = append(vs, v{fmt.Sprintf("ex-%v-%v", sign, y), b, r3.Vec{X: sign * y}, e2})
			}
		}
	}
	lim := 0.0009
	for _, d := range []float64{zzPrev(lim), lim, zzNext(lim)} {
		for _, sign := range []float64{1, -1} {
			// pair (Ex,Ey)
			b := r3.Vec{Y: s, Z: zc}
			ey := r3.Vec{X: sign * d, Y: 1}
			ey.Z = -b.Y * ey.Y / b.Z
			vs = append(vs, v{fmt.Sprintf("pairExEy-%v-%v", sign, d), b, r3.Vec{X: 1}, ey})
			// pair (Axis,Ex): Axis.X = d
			a := r3.Vec{X: sign * d, Y: s, Z: zc}
			e1 := r3.Vec{X: 1}
			e2 := r3.Vec{Y: 1}
			e2.Z = -a.Y / a.Z
			vs = append(vs, v{fmt.Sprintf("pairAxEx-%v-%v", sign, d), a, e1, e2})
			// pair (Axis,Ey): Axis.Y = d
			a = r3.Vec{X: s, Y: sign * d, Z: zc}
			e2 = r3.Vec{Y: 1}
			e1 = r3.Vec{X: 1}
			e1.Z = -a.X / a.Z
			vs = append(vs, v{fmt.Sprintf("pairAxEy-%v-%v", sign, d), a, e1, e2})
		}
	}
	for _, x := range vs {
		r := *fx.rec
		r.Axis, r.Ex, r.Ey = x.axis, x.ex, x.ey
		for _, rad := range []float64{0.5} {
			zzRun(t, x.name, &r, rad)
		}
	}
}

func TestZZObliqueSourceBoundThreshold(t *testing.T) {
	fx := zzBase(t)
	om, ok := zzBuild(t, "thr", fx.rec, fx.radius)
	if !ok {
		t.Fatal("build")
	}
	passes := func(sb float64) bool {
		r := *fx.rec
		r.SourceBound = sb
		_, _, _, err := oldProveObliqueBossFillet(t.Context(), &r, fx.radius, om)
		return err == nil
	}
	if !passes(0) || passes(1) {
		t.Fatal("bisect ends")
	}
	lo, hi := math.Float64bits(0), math.Float64bits(1)
	for hi-lo > 1 {
		mid := lo + (hi-lo)/2
		if passes(math.Float64frombits(mid)) {
			lo = mid
		} else {
			hi = mid
		}
	}
	t.Logf("threshold: pass %v fail %v", math.Float64frombits(lo), math.Float64frombits(hi))
	for _, bits := range []uint64{lo - 1, lo, hi, hi + 1} {
		r := *fx.rec
		r.SourceBound = math.Float64frombits(bits)
		zzProve(t, fmt.Sprintf("thr-%d", bits), &r, fx.radius, om, false)
	}
}

func TestZZObliquePlanAndRim(t *testing.T) {
	fx := zzFixtures(t)[1]
	s := math.Sin(15 * math.Pi / 180)
	c := math.Cos(15 * math.Pi / 180)
	axes := []r3.Vec{{Y: s, Z: c}, {Y: -s, Z: c}, {X: s, Z: c}, {X: -s, Z: c}}
	deltas := []float64{0, 1e-17, -1e-17, 1e-15, -1e-15, 1e-13, -1e-13, 5e-13, -5e-13, 9e-13, -9e-13, 2e-12, -2e-12, 1e-9}
	edges := []float64{0, 20, 10}
	if zzFast {
		axes = []r3.Vec{axes[0], axes[3]}
		edges = []float64{0, 20}
		deltas = []float64{0, 1e-17, -1e-17, 5e-13, -5e-13, 2e-12, -2e-12}
	}
	for ai, ax := range axes {
		for _, edge := range edges {
			for _, d := range deltas {
				for _, alongX := range []bool{true, false} {
					r := *fx.rec
					zzAxisSet(&r, ax)
					if alongX {
						r.Base.X = edge + d
					} else {
						r.Base.Y = edge + d
					}
					zzRun(t, fmt.Sprintf("plan-a%d-e%v-d%v-x%v", ai, edge, d, alongX), &r, 0.5)
				}
			}
		}
	}
}

func zzVariantsOf(t *testing.T, fx zzFx) {
	t.Helper()
	rec := fx.rec
	pp := fx.plate.n.Payload.(prismPayload)
	mk := func(segs []momentinput.CurveSegment) *obliqueBossRecord {
		p2 := pp
		p2.Profile.Outer.Segments = segs
		nb := zzNode()
		nb.Lumps = fx.plate.n.Lumps
		nb.Origin = fx.plate.n.Origin
		nb.Payload = p2
		r := *rec
		r.Plate = nb
		return &r
	}
	segs := func() []momentinput.CurveSegment {
		return append([]momentinput.CurveSegment(nil), pp.Profile.Outer.Segments...)
	}
	run := func(name string, r *obliqueBossRecord) { zzRun(t, "var-"+name, r, 0.5) }
	run("control", rec)
	run("copy", mk(segs()))
	for k := 1; k < 4; k++ {
		s := segs()
		s = append(s[k:], s[:k]...)
		run(fmt.Sprintf("rot%d", k), mk(s))
	}
	{
		s := segs()
		s[0], s[3] = s[3], s[0]
		run("swap03", mk(s))
	}
	rect := rec.Rect
	for i := 0; i < 4; i++ {
		for _, f := range []string{"EndU", "StartU", "EndV", "StartV"} {
			for _, d := range []float64{math.SmallestNonzeroFloat64, 1e-12} {
				s := segs()
				l := s[i].(lineSeg)
				switch f {
				case "EndU":
					l.End.U = zzNext(l.End.U) + d
				case "StartU":
					l.Start.U = zzNext(l.Start.U) + d
				case "EndV":
					l.End.V = zzNext(l.End.V) + d
				case "StartV":
					l.Start.V = zzNext(l.Start.V) + d
				}
				s[i] = l
				run(fmt.Sprintf("seg%d-%s-%g", i, f, d), mk(s))
			}
		}
		s := segs()
		s[i] = circleSeg{}
		run(fmt.Sprintf("circle%d", i), mk(s))
		s = segs()
		s[i] = s[(i+1)%4]
		run(fmt.Sprintf("dup%d", i), mk(s))
		s = segs()
		l := s[i].(lineSeg)
		l.Start, l.End = l.End, l.Start
		s[i] = l
		run(fmt.Sprintf("rev%d", i), mk(s))
		for _, pt := range []lineSeg{
			{Start: planePoint{U: rect.U0, V: rect.V0}, End: planePoint{U: rect.U0, V: rect.V0}},
			{Start: planePoint{U: rect.U1, V: rect.V1}, End: planePoint{U: rect.U1, V: rect.V1}},
			{Start: planePoint{U: rect.U1, V: rect.V0}, End: planePoint{U: rect.U1, V: rect.V0}},
			{Start: planePoint{U: rect.U0, V: rect.V1}, End: planePoint{U: rect.U0, V: rect.V1}},
		} {
			s = segs()
			s[i] = pt
			run(fmt.Sprintf("degenerate%d-%v", i, pt.Start), mk(s))
		}
	}
	run("three", mk(segs()[:3]))
	run("five", mk(append(segs(), segs()[0])))
	run("empty", mk(nil))
	for _, side := range []string{"U0", "U1", "V0", "V1"} {
		r := *rec
		switch side {
		case "U0":
			r.Rect.U0 = zzPrev(r.Rect.U0)
		case "U1":
			r.Rect.U1 = zzNext(r.Rect.U1)
		case "V0":
			r.Rect.V0 = zzPrev(r.Rect.V0)
		case "V1":
			r.Rect.V1 = zzNext(r.Rect.V1)
		}
		run("rect-"+side, &r)
	}
	// bodies without the payload, with the boss payload, without faces, with other faces
	nb := zzNode()
	nb.Lumps, nb.Origin = fx.plate.n.Lumps, fx.plate.n.Origin
	r := *rec
	r.Plate = nb
	run("nil-payload", &r)
	nb2 := zzNode()
	nb2.Lumps, nb2.Origin, nb2.Payload = fx.plate.n.Lumps, fx.plate.n.Origin, fx.boss.n.Payload
	r = *rec
	r.Plate = nb2
	run("boss-payload", &r)
	nb3 := zzNode()
	nb3.Origin, nb3.Payload = fx.plate.n.Origin, pp
	r = *rec
	r.Plate = nb3
	run("no-faces", &r)
	nb4 := zzNode()
	nb4.Lumps, nb4.Origin, nb4.Payload = fx.boss.n.Lumps, fx.plate.n.Origin, pp
	r = *rec
	r.Plate = nb4
	run("boss-faces-for-plate", &r)
	nb5 := zzNode()
	nb5.Lumps, nb5.Origin, nb5.Payload = fx.plate.n.Lumps, fx.plate.n.Origin, pp
	nb5.Origin.Producer++
	r = *rec
	r.Plate = nb5
	run("plate-other-producer", &r)
	r = *rec
	r.Boss = rec.Plate
	run("boss-is-plate", &r)
	r = *rec
	nb6 := zzNode()
	nb6.Lumps, nb6.Origin = fx.boss.n.Lumps, fx.boss.n.Origin
	nb6.Origin.Producer++
	r.Boss = nb6
	run("boss-other-producer", &r)
	r = *rec
	r.Boss = zzNode()
	run("boss-empty", &r)
}

func TestZZObliqueVariants(t *testing.T) {
	fxs := zzFixtures(t)
	zzVariantsOf(t, fxs[1])
	zzVariantsOf(t, fxs[len(fxs)-3])
}

func TestZZObliqueSynthPlates(t *testing.T) {
	base := zzFixtures(t)[1]
	type pl struct {
		name               string
		x0, y0, x1, y1, th float64
	}
	for _, p := range []pl{
		{"small", 7, 7, 13, 13, 5}, {"narrow", 0, 8, 20, 12, 5}, {"off", -30, -30, -2, -2, 4},
		{"shift", 5, 5, 25, 25, 5}, {"thin", 0, 0, 20, 20, 0.5}, {"edge", 10, 0, 40, 20, 5},
		{"left", -10, 0, 10, 20, 5}, {"corner", 10, 10, 30, 30, 5}, {"wide", -100, -100, 100, 100, 5},
	} {
		ctx := t.Context()
		doc := New()
		w := sketch.NewWorld()
		ps, _ := w.CreateSketch(w.XY())
		rect := ps.CreateRectangle(p.x0, p.y0, p.x1, p.y1)
		ps.Fix(rect.A)
		if _, err := ps.Solve(ctx); err != nil {
			t.Fatal(err)
		}
		plate, err := Extrude(ctx, doc, ps, ps.Profiles()[0], Distance{D: units.Millimeters(p.th), Dir: Along})
		if err != nil {
			t.Fatal(err)
		}
		r := zzSynth(base.rec, plate)
		for _, rad := range []float64{0.1, 0.5, 1} {
			zzRun(t, "synth-"+p.name, r, rad)
			r2 := *r
			zzAxisSet(&r2, r3.Vec{X: 0.2, Y: -0.1, Z: math.Sqrt(1 - 0.05)})
			zzRun(t, "synth-skew-"+p.name, &r2, rad)
		}
	}
}

func zzNode() *topo.Body { return bodyNode(bornBody(&Body{})) }

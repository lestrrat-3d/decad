package zzclipdiff

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"
	"reflect"
	"strings"
	"sync/atomic"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionclip"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// enc is a bit-exact, nil-versus-empty-aware canonical form of a result.
func enc(v any) string {
	var sb strings.Builder
	encValue(&sb, reflect.ValueOf(v))
	return sb.String()
}

var ratType = reflect.TypeOf((*big.Rat)(nil))
var unitsValueType = reflect.TypeOf(units.Value{})

func encValue(sb *strings.Builder, v reflect.Value) {
	if !v.IsValid() {
		sb.WriteString("<invalid>")
		return
	}
	switch {
	case v.Type() == ratType:
		if v.IsNil() {
			sb.WriteString("rat(nil)")
			return
		}
		sb.WriteString("rat(" + v.Interface().(*big.Rat).String() + ")")
		return
	case v.Type() == unitsValueType:
		u := v.Interface().(units.Value)
		fmt.Fprintf(sb, "units(%x,%s)", math.Float64bits(u.Base()), u.Unit().String())
		return
	}
	switch v.Kind() {
	case reflect.Float64, reflect.Float32:
		fmt.Fprintf(sb, "f%x", math.Float64bits(v.Float()))
	case reflect.Bool:
		fmt.Fprintf(sb, "b%v", v.Bool())
	case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		fmt.Fprintf(sb, "i%d", v.Int())
	case reflect.String:
		fmt.Fprintf(sb, "s%q", v.String())
	case reflect.Slice:
		if v.IsNil() {
			sb.WriteString("slice(nil)")
			return
		}
		fmt.Fprintf(sb, "slice%d[", v.Len())
		for i := range v.Len() {
			encValue(sb, v.Index(i))
			sb.WriteString(",")
		}
		sb.WriteString("]")
	case reflect.Array:
		sb.WriteString("arr[")
		for i := range v.Len() {
			encValue(sb, v.Index(i))
			sb.WriteString(",")
		}
		sb.WriteString("]")
	case reflect.Struct:
		sb.WriteString(v.Type().String() + "{")
		for i := range v.NumField() {
			encValue(sb, v.Field(i))
			sb.WriteString(",")
		}
		sb.WriteString("}")
	case reflect.Interface:
		if v.IsNil() {
			sb.WriteString("iface(nil)")
			return
		}
		sb.WriteString(v.Elem().Type().String() + ":")
		encValue(sb, v.Elem())
	case reflect.Pointer:
		if v.IsNil() {
			sb.WriteString("ptr(nil)")
			return
		}
		sb.WriteString("ptr:")
		encValue(sb, v.Elem())
	default:
		fmt.Fprintf(sb, "?%s", v.Kind())
	}
}

func encErr(err error) string {
	if err == nil {
		return "err(nil)"
	}
	return fmt.Sprintf("err(%q,unsupported=%v,canceled=%v)", err.Error(),
		errors.Is(err, decaderr.ErrUnsupported), errors.Is(err, context.Canceled))
}

func encSketch(s *sketch.Sketch, ps []*sketch.Profile, err error) string {
	var sb strings.Builder
	sb.WriteString(encErr(err))
	fmt.Fprintf(&sb, "|sketch=%v|n=%d", s != nil, len(ps))
	if ps == nil {
		sb.WriteString("|nilprofiles")
	}
	for _, p := range ps {
		fmt.Fprintf(&sb, "|valid=%v,area=%x,outer=%d,holes=%d,entities=%d", p.Valid,
			math.Float64bits(p.Area), len(p.Outer), len(p.Holes), len(p.Entities))
		for _, e := range p.Outer {
			fmt.Fprintf(&sb, ";%T", e)
		}
	}
	return sb.String()
}

type tally struct {
	t      *testing.T
	checks map[string]int
	oks    map[string]int
}

func newTally(t *testing.T) *tally {
	return &tally{t: t, checks: map[string]int{}, oks: map[string]int{}}
}

func (c *tally) same(fn, label string, ok bool, o, n string) {
	c.t.Helper()
	c.checks[fn]++
	if ok {
		c.oks[fn]++
	}
	if o != n {
		c.t.Fatalf("%s %s differs: lengths old=%d new=%d; prefixes old=%q new=%q",
			fn, label, len(o), len(n), o[:min(len(o), 256)], n[:min(len(n), 256)])
	}
}

func (c *tally) report() {
	for fn, n := range c.checks {
		c.t.Logf("%-40s checks=%6d admitted=%6d refused=%6d", fn, n, c.oks[fn], n-c.oks[fn])
	}
}

// pollCtx cancels itself on the (limit+1)-th Err call.
type pollCtx struct {
	context.Context
	limit int64
	calls atomic.Int64
}

func (c *pollCtx) Err() error {
	if c.calls.Add(1) > c.limit {
		return context.Canceled
	}
	return nil
}

func mustFrame(o, u, v r3.Vec) r3.Frame {
	f, err := r3.NewFrame(o, u, v)
	if err != nil {
		panic(err)
	}
	return f
}

func vec(x, y, z float64) r3.Vec { return r3.NewVec(x, y, z) }

func next(x float64) float64 { return math.Nextafter(x, math.Inf(1)) }
func prev(x float64) float64 { return math.Nextafter(x, math.Inf(-1)) }

func rectProfile(u0, v0, u1, v1 float64) profileRecord {
	a, b := planePoint{U: u0, V: v0}, planePoint{U: u1, V: v0}
	c, d := planePoint{U: u1, V: v1}, planePoint{U: u0, V: v1}
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{
		lineSeg{Start: a, End: b, TStart: 0, TEnd: 1},
		lineSeg{Start: b, End: c, TStart: 0, TEnd: 1},
		lineSeg{Start: c, End: d, TStart: 0, TEnd: 1},
		lineSeg{Start: d, End: a, TStart: 0, TEnd: 1},
	}}}
}

func circleRec(cu, cv, r float64, ccw bool) circleSeg {
	if ccw {
		return circleSeg{Center: planePoint{U: cu, V: cv}, Radius: units.Millimeters(r), CCW: true, TStart: 0, TEnd: 1}
	}
	return circleSeg{Center: planePoint{U: cu, V: cv}, Radius: units.Millimeters(r), CCW: false, TStart: 1, TEnd: 0}
}

func circleProfile(cu, cv, r float64) profileRecord {
	return profileRecord{Outer: loopRecord{Segments: []curveSegment{circleRec(cu, cv, r, true)}}}
}

func circleLoop(cu, cv, r float64, ccw bool) loopRecord {
	return loopRecord{Segments: []curveSegment{circleRec(cu, cv, r, ccw)}}
}

var (
	ex = vec(1, 0, 0)
	ey = vec(0, 1, 0)
	ez = vec(0, 0, 1)
)

func neg(v r3.Vec) r3.Vec { return vec(-v.X, -v.Y, -v.Z) }

func cardinalAxes() []r3.Vec { return []r3.Vec{ex, neg(ex), ey, neg(ey), ez, neg(ez)} }

// cardinalPairs lists every ordered pair of perpendicular cardinal axes.
func cardinalPairs() [][2]r3.Vec {
	var out [][2]r3.Vec
	for _, u := range cardinalAxes() {
		for _, v := range cardinalAxes() {
			if u.Dot(v) == 0 {
				out = append(out, [2]r3.Vec{u, v})
			}
		}
	}
	return out
}

func rigid(f r3.Frame) r3.Transform {
	x, err := r3.FromFrame(f)
	if err != nil {
		panic(err)
	}
	return x
}

func translation(x, y, z float64) r3.Transform {
	x2, err := r3.Translation(vec(x, y, z))
	if err != nil {
		panic(err)
	}
	return x2
}

func rotation(axis r3.Vec, rad float64) r3.Transform {
	x, err := r3.Rotation(axis, units.Radians(rad))
	if err != nil {
		panic(err)
	}
	return x
}

func then(a, b r3.Transform) r3.Transform {
	x, err := a.Then(b)
	if err != nil {
		panic(err)
	}
	return x
}

// bisectFlip finds adjacent floats a<b (or a>b) where pred flips between
// lo (pred true) and hi (pred false); pred must be monotone between them.
func bisectFlip(lo, hi float64, pred func(float64) bool) (float64, float64, bool) {
	if !pred(lo) || pred(hi) {
		return 0, 0, false
	}
	for range 4000 {
		mid := lo + (hi-lo)/2
		if mid == lo || mid == hi {
			return lo, hi, true
		}
		if pred(mid) {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo, hi, true
}

var _ = sectionclip.SectionRatAdd
var _ = sectionrecord.Point2{}

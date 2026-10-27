package brepmodify

import (
	"context"
	"errors"
	"fmt"
	"math"
	"reflect"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/opdiagnostic"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/rimoffset"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The real rim cutter builder runs in both cases. The Boolean hook supplies
// bounded accepted-step summaries to test their transport independently of
// which repairs a particular cutter's contact arrangement happens to require.
func TestCutRimsAggregatesOnlyAnAcceptedAttempt(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	pp := prismrecord.Payload{Profile: momentinput.Profile{Outer: rimWarningCircle()},
		Frame: frame, Z0: 0, Z1: 10, Xform: r3.Identity()}
	refusal := errors.New("the second private Cut refused")
	for _, failSecond := range []bool{false, true} {
		name := "both accepted"
		if failSecond {
			name = "later failure"
		}
		t.Run(name, func(t *testing.T) {
			env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
			source := &topo.Body{OperationWarnings: []opdiagnostic.Record{
				{Code: opdiagnostic.CodeStitchMergeApplied, DisplacementBound: 8},
			}}
			calls := 0
			hooks := Hooks{Boolean: func(_ context.Context, op meshbool.OperationKind,
				_ topo.Env, _, tool *topo.Body) (*topo.Body, error) {
				calls++
				require.Equal(t, meshbool.OpCut, op)
				require.True(t, tool.Solid)
				require.Positive(t, tool.Volume.Value.Base())
				if failSecond && calls == 2 {
					return nil, refusal
				}
				return &topo.Body{OperationWarnings: []opdiagnostic.Record{{
					Code: opdiagnostic.CodeGapShiftApplied, Operand: 1,
					DisplacementBound: float64(calls) * 0.001, AllowedReach: 0.01,
				}}}, nil
			}}
			out, used, retry, err := cutRims(t.Context(), env, hooks, source, pp,
				[]rim{{start: true}, {start: false}}, 0.5, rimAttempts[len(rimAttempts)-1], 0)
			if failSecond {
				require.ErrorIs(t, err, refusal)
			} else {
				require.NoError(t, err)
			}
			require.Equal(t, 2, calls)
			require.Equal(t, 4, used)
			require.Len(t, source.OperationWarnings, 1)
			require.Equal(t, opdiagnostic.CodeStitchMergeApplied, source.OperationWarnings[0].Code)
			if failSecond {
				require.ErrorIs(t, err, refusal)
				require.Nil(t, out)
				require.True(t, retry)
				return
			}
			require.NoError(t, err)
			require.False(t, retry)
			require.Len(t, out.OperationWarnings, 1)
			warning := out.OperationWarnings[0]
			require.Equal(t, opdiagnostic.CodeGapShiftApplied, warning.Code)
			require.Equal(t, -1, warning.Operand)
			require.GreaterOrEqual(t, warning.DisplacementBound, 0.003)
			require.Less(t, warning.DisplacementBound, 0.0031)
		})
	}
}

func rimWarningCircle() sectionrecord.LoopRecord {
	var loop sectionrecord.LoopRecord
	for s := range 8 {
		fit := make([]sectionrecord.Point2, 9)
		for i := range fit {
			a := 2 * math.Pi * (float64(s) + float64(i)/8) / 8
			fit[i] = sectionrecord.Point2{U: 10 * math.Cos(a), V: 10 * math.Sin(a)}
		}
		if s == 7 {
			fit[8] = sectionrecord.Point2{U: 10}
		}
		loop.Segments = append(loop.Segments, sectionrecord.FitSplineSeg{Fit: fit, TStart: 0, TEnd: 1})
	}
	return loop
}

// BenchmarkCutRimsCutters builds both rims' cutters of a fit-spline prism. The
// Boolean hook hands the receiver back, so the time is the cutters' own.
func BenchmarkCutRimsCutters(b *testing.B) {
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(b, err)
	pp := prismrecord.Payload{Profile: momentinput.Profile{Outer: rimWarningCircle()},
		Frame: frame, Z0: 0, Z1: 10, Xform: r3.Identity()}
	hooks := Hooks{Boolean: func(_ context.Context, _ meshbool.OperationKind,
		_ topo.Env, receiver, _ *topo.Body) (*topo.Body, error) {
		return receiver, nil
	}}
	rims := []rim{{start: true}, {start: false}}
	b.ReportAllocs()
	for b.Loop() {
		env := topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}}
		if _, _, _, err := cutRims(b.Context(), env, hooks, &topo.Body{}, pp, rims, 0.5, rimAttempts[len(rimAttempts)-1], 0); err != nil {
			b.Fatal(err)
		}
	}
}

// A later rim's cutter that reads the first rim's held pairing is the cutter a
// fresh build makes, to the float bit, and a cutter whose records differ, or
// whose planes coincide, builds or refuses as a fresh build does. The rotated
// hole bounds the same area as the held one, so only the record comparison
// tells the two apart.
func TestRimCutterReuseMatchesFreshBuild(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.Vec{}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	pp := prismrecord.Payload{Profile: momentinput.Profile{Outer: rimWarningCircle()},
		Frame: frame, Z0: 0, Z1: 10, Xform: r3.Identity()}
	const d = 0.5
	attempt := rimAttempts[len(rimAttempts)-1]
	over, tol := attempt.overshoot*d, rimoffset.MaxDeviationRatio*d
	into, err := rimoffset.OffsetLoop(pp.Profile.Outer, d+over, tol)
	require.NoError(t, err)
	away, err := rimoffset.OffsetLoop(pp.Profile.Outer, -over, tol)
	require.NoError(t, err)
	outer := rimBoundary(pp.Profile.Outer, attempt.scale)
	// The same hole drawn from another segment has the same area and another record.
	rotated := append(slices.Clone(into.Segments[1:]), into.Segments[0])

	reuse := new(loftrecord.PairingReuse)
	cutter := func(ref int, z0, z1 float64, hole0 []sectionrecord.CurveSegment,
		reuse *loftrecord.PairingReuse) (*topo.Body, error) {
		env := topo.Env{Doc: &topo.Document{}, Ref: provenance.Producer(ref), Alloc: topotest.Alloc{}}
		return rimCutter(t.Context(), env, pp, z0, z1, outer, hole0, away.Segments, reuse)
	}
	_, err = cutter(3, pp.Z0-over, pp.Z0+d+over, into.Segments, reuse)
	require.NoError(t, err)
	require.Zero(t, reuse.Reused())

	other := func(reuse *loftrecord.PairingReuse) (*topo.Body, error) {
		return cutter(5, pp.Z1+over, pp.Z1-d-over, into.Segments, reuse)
	}
	want, err := other(nil)
	require.NoError(t, err)
	got, err := other(reuse)
	require.NoError(t, err)
	require.Equal(t, 1, reuse.Reused())
	if diff := bitDifference(reflect.ValueOf(want), reflect.ValueOf(got), map[[2]uintptr]struct{}{}); diff != "" {
		require.Fail(t, "the reused cutter differs from the fresh one", "body%s", diff)
	}

	// A fresh build of two rings on one plane refuses at the S5 gate.
	_, err = cutter(5, pp.Z1, pp.Z1, into.Segments, reuse)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.ErrorContains(t, err, "the two profiles lie in the same geometric plane")

	_, err = cutter(5, pp.Z1+over, pp.Z1-d-over, rotated, reuse)
	require.NoError(t, err)
	require.Equal(t, 1, reuse.Reused(), "the rotated hole is another record")
}

// bitDifference returns where a and b first differ, as a path below them, or
// "" when they hold the same values with the same float bits. It follows
// pointers once per pair, skips the lazily filled sync, sync/atomic and
// radiusmemo caches, and builds the path only on the way back from a
// difference, so a failure prints no graph and a match allocates no path.
func bitDifference(a, b reflect.Value, seen map[[2]uintptr]struct{}) string {
	if !a.IsValid() || !b.IsValid() {
		return differsIf(a.IsValid() != b.IsValid(), " (valid)")
	}
	if a.Type() != b.Type() {
		return " (type)"
	}
	switch a.Kind() { //nolint:exhaustive // every other kind is reported as a difference.
	case reflect.Bool:
		return differsIf(a.Bool() != b.Bool(), " (value)")
	case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		return differsIf(a.Int() != b.Int(), " (value)")
	case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64, reflect.Uintptr:
		return differsIf(a.Uint() != b.Uint(), " (value)")
	case reflect.Float32, reflect.Float64:
		return differsIf(math.Float64bits(a.Float()) != math.Float64bits(b.Float()), " (bits)")
	case reflect.String:
		return differsIf(a.String() != b.String(), " (value)")
	case reflect.Struct:
		for i := range a.NumField() {
			switch a.Type().Field(i).Type.PkgPath() {
			case "sync", "sync/atomic", "github.com/lestrrat-3d/decad/internal/radiusmemo":
				continue
			}
			if diff := bitDifference(a.Field(i), b.Field(i), seen); diff != "" {
				return "." + a.Type().Field(i).Name + diff
			}
		}
		return ""
	case reflect.Slice, reflect.Array:
		if a.Kind() == reflect.Slice && a.IsNil() != b.IsNil() {
			return " (nil)"
		}
		if a.Len() != b.Len() {
			return " (len)"
		}
		for i := range a.Len() {
			if diff := bitDifference(a.Index(i), b.Index(i), seen); diff != "" {
				return fmt.Sprintf("[%d]", i) + diff
			}
		}
		return ""
	case reflect.Map:
		if a.IsNil() != b.IsNil() {
			return " (nil)"
		}
		if a.Len() != b.Len() {
			return " (len)"
		}
		iter := a.MapRange()
		for iter.Next() {
			other := b.MapIndex(iter.Key())
			if !other.IsValid() {
				return fmt.Sprintf("[%v] (missing)", iter.Key())
			}
			if diff := bitDifference(iter.Value(), other, seen); diff != "" {
				return fmt.Sprintf("[%v]", iter.Key()) + diff
			}
		}
		return ""
	case reflect.Pointer:
		if a.IsNil() || b.IsNil() {
			return differsIf(a.IsNil() != b.IsNil(), " (nil)")
		}
		key := [2]uintptr{a.Pointer(), b.Pointer()}
		if _, ok := seen[key]; ok || key[0] == key[1] {
			return ""
		}
		seen[key] = struct{}{}
		return bitDifference(a.Elem(), b.Elem(), seen)
	case reflect.Interface:
		if a.IsNil() || b.IsNil() {
			return differsIf(a.IsNil() != b.IsNil(), " (nil)")
		}
		return bitDifference(a.Elem(), b.Elem(), seen)
	default:
		return " (kind " + a.Kind().String() + ")"
	}
}

func differsIf(differs bool, why string) string {
	if differs {
		return why
	}
	return ""
}

package decad

import (
	"context"
	"errors"
	"math"
	"regexp"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

var slabRefusalPattern = regexp.MustCompile(
	`within a bound of (\S+) mm, coarser than this pair's chord tolerance (\S+) mm`)

// slabRefusal reads the touched bound and chord tolerance out of a plain
// chain-depth ErrUnsupported, and checks the bound is the coarser of the two.
func slabRefusal(t *testing.T, err error) (float64, float64) {
	t.Helper()
	require.ErrorIs(t, err, ErrUnsupported)
	var be *BooleanError
	require.False(t, errors.As(err, &be), "the chain-depth refusal is a plain ErrUnsupported")
	m := slabRefusalPattern.FindStringSubmatch(err.Error())
	require.Len(t, m, 3, "the refusal quotes the touched bound and the pair's chord tolerance")
	touched, perr := strconv.ParseFloat(m[1], 64)
	require.NoError(t, perr)
	tol, perr := strconv.ParseFloat(m[2], 64)
	require.NoError(t, perr)
	require.Greater(t, touched, tol)
	return touched, tol
}

// planesAlong returns the plane faces of b whose normal is parallel to n.
func planesAlong(b *Body, n r3.Vec) []*topo.Face {
	var out []*topo.Face
	for _, f := range topobuild.Faces(&b.n) {
		pl, ok := f.Surface.(surfacegeom.Plane)
		if ok && math.Abs(pl.Frame.N().Dot(n)) > 1-1e-9 {
			out = append(out, f)
		}
	}
	return out
}

// requireSameRefusal checks the real run and the control refuse with the same
// touched bound and chord tolerance: the slab changed nothing.
func requireSameRefusal(t *testing.T, realErr, ctrlErr error) {
	t.Helper()
	rb, rt := slabRefusal(t, realErr)
	cb, ct := slabRefusal(t, ctrlErr)
	require.Equal(t, cb, rb, "the real run quotes the control's touched bound")
	require.Equal(t, ct, rt, "the real run quotes the control's chord tolerance")
}

// fallbackP3 cuts the P3 scene's first bore, places both it and the second
// y-tool (x = 30, r = 4) by opt (none when nil), and cuts the placed pair. It
// returns the placed target beside the second cut's result.
func fallbackP3(t *testing.T, ctx context.Context, opt CopyOption) (*Body, *Body, error) {
	t.Helper()
	sc := &slabScene{w: sketch.NewWorld(), doc: New()}
	target, err := Cut(ctx, sc.roundedBox(ctx, t), sc.yTool(ctx, t, 20, 4))
	require.NoError(t, err)
	tool := sc.yTool(ctx, t, 30, 4)
	target, err = Copy(ctx, target, opt)
	require.NoError(t, err)
	tool, err = Copy(ctx, tool, opt)
	require.NoError(t, err)
	got, err := Cut(ctx, target, tool)
	return target, got, err
}

// requireTwoBores checks the volume of the P3 part with two full-depth y-bores
// of radius 4.
func requireTwoBores(t *testing.T, b *Body) {
	t.Helper()
	want := (800-4*(1-math.Pi/4)*9)*20 - 20*(2*math.Pi*16)
	got, err := b.Volume()
	require.NoError(t, err)
	require.InDelta(t, want, got.Value.Mag(), got.Bound.Mag()+1e-6)
}

// TestFacetedSourcePatchFallback shows the retained-plane slab stays off where
// placement leaves a plane tag a float image off its plane, where a face
// carries a denoted patch, and where the contact lies on a curved rim, and
// stays on for an exact translation and for one whose rounding stays within
// the planes (agent-docs/faceted-source-retention-design.md §3.4). Each case
// runs against booleaneval.WithoutRetainedPlaneSlab.
func TestFacetedSourcePatchFallback(t *testing.T) {
	t.Parallel()

	yAxis := r3.Vec{Y: 1}
	rot, err := r3.RotationAround(r3.Vec{X: 20, Y: 10, Z: 10}, r3.Vec{Z: 1}, units.Degrees(20))
	require.NoError(t, err)
	tilted := rot.ApplyDir(yAxis)

	for _, tc := range []struct {
		name string
		opt  CopyOption
		axis r3.Vec
	}{
		{"a 20 degree exact rotation", WithCopyRotation(r3.Vec{X: 20, Y: 10, Z: 10}, r3.Vec{Z: 1}, units.Degrees(20)), tilted},
	} {
		t.Run(tc.name+" refuses as the control does", func(t *testing.T) {
			t.Parallel()
			target, _, realErr := fallbackP3(t, t.Context(), tc.opt)
			planes := planesAlong(target, tc.axis)
			require.Len(t, planes, 2, "the later cut touches the front and back faces")
			for _, f := range planes {
				require.True(t, f.RetainedTag)
				require.Positive(t, f.NormalBound, "the placement leaves the tag a float image")
			}
			_, _, ctrlErr := fallbackP3(t, booleaneval.WithoutRetainedPlaneSlab(t.Context()), tc.opt)
			requireSameRefusal(t, realErr, ctrlErr)
		})
	}

	// The front and back faces are y = 0 and y = 20. Moved by Y = 0.1, the
	// front face's level is the exact float 0 + 0.1 while the back face's is
	// the rounded sum 20 + 0.1, a tag off its plane. The back face keeps its
	// slab off, so the cut refuses where the integer translation builds.
	t.Run("a translation by Y = 0.1 that moves the back face refuses", func(t *testing.T) {
		t.Parallel()
		target, _, err := fallbackP3(t, t.Context(), WithCopyPlacement(mustTranslation(r3.Vec{Y: 0.1})))
		planes := planesAlong(target, yAxis)
		require.Len(t, planes, 2, "the later cut touches the front and back faces")
		for _, f := range planes {
			require.True(t, f.RetainedTag)
			level := f.Surface.(surfacegeom.Plane).Frame.Origin().Y
			if level < 10 {
				require.Zero(t, f.NormalBound, "the front face's level 0 + 0.1 is exact")
				continue
			}
			require.Positive(t, f.NormalBound, "the back face's level 20 + 0.1 rounds")
		}
		slabRefusal(t, err)
	})

	// An integer translation is exact. A translation by X = 0.1 rounds each
	// moved frame origin's x, which lies within the front and back planes, so
	// both tags stay their planes exactly.
	for _, tc := range []struct {
		name string
		move r3.Vec
	}{
		{"an integer translation", r3.Vec{X: 5}},
		{"a translation by X = 0.1 within the planes", r3.Vec{X: 0.1}},
	} {
		t.Run(tc.name+" builds where the control refuses", func(t *testing.T) {
			t.Parallel()
			opt := WithCopyPlacement(mustTranslation(tc.move))
			target, got, err := fallbackP3(t, t.Context(), opt)
			require.NoError(t, err)
			planes := planesAlong(target, yAxis)
			require.Len(t, planes, 2)
			for _, f := range planes {
				require.True(t, f.RetainedTag)
				require.Zero(t, f.NormalBound, "the tag is the plane itself")
			}
			requireTwoBores(t, got)

			_, _, ctrlErr := fallbackP3(t, booleaneval.WithoutRetainedPlaneSlab(t.Context()), opt)
			slabRefusal(t, ctrlErr)
		})
	}

	t.Run("a denoted cap refuses as the control does", func(t *testing.T) {
		t.Parallel()
		run := func(ctx context.Context) (*Body, error) {
			sc := &slabScene{w: sketch.NewWorld(), doc: New()}
			part := sc.revolvedShaft(ctx, t)
			built, err := Cut(ctx, part, sc.zPin(ctx, t, 4, 6))
			require.NoError(t, err)
			// A Copy ends the operation history's Boolean chain, so the later cut meets the
			// sequential gate and not a chain the regroup rebuilds.
			first, err := Copy(ctx, built)
			require.NoError(t, err)
			caps := planesThrough(first, r3.Vec{X: 4, Y: 6})
			require.Len(t, caps, 1, "the pin pierces one cap")
			require.True(t, caps[0].RetainedTag)
			require.NotNil(t, caps[0].Denoted, "the cap carries a denoted patch")
			return Cut(ctx, first, sc.zPin(ctx, t, 8, 6))
		}
		_, realErr := run(t.Context())
		_, ctrlErr := run(booleaneval.WithoutRetainedPlaneSlab(t.Context()))
		requireSameRefusal(t, realErr, ctrlErr)
	})

	t.Run("a cut through a curved rim refuses as the control does", func(t *testing.T) {
		t.Parallel()
		run := func(ctx context.Context) error {
			// The tool's wall crosses the first bore's wall, whose facets hold
			// the rim's bound: a cylinder states no thinner slab.
			sc := &slabScene{w: sketch.NewWorld(), doc: New()}
			built, err := Cut(ctx, sc.roundedBox(ctx, t), sc.yTool(ctx, t, 20, 4))
			require.NoError(t, err)
			// A Copy ends the operation history's Boolean chain, so the refusal under test
			// is the sequential gate and not a chain the regroup rebuilds.
			first, err := Copy(ctx, built)
			require.NoError(t, err)
			_, err = Cut(ctx, first, sc.yTool(ctx, t, 22, 3))
			return err
		}
		realErr := run(t.Context())
		ctrlErr := run(booleaneval.WithoutRetainedPlaneSlab(t.Context()))
		requireSameRefusal(t, realErr, ctrlErr)
	})
}

// planesThrough returns the z-normal plane faces of b that contain p.
func planesThrough(b *Body, p r3.Vec) []*topo.Face {
	var out []*topo.Face
	for _, f := range planesAlong(b, r3.Vec{Z: 1}) {
		fr := f.Surface.(surfacegeom.Plane).Frame
		if math.Abs(fr.N().Dot(p.Sub(fr.Origin()))) < 1e-9 {
			out = append(out, f)
		}
	}
	return out
}

// revolvedShaft revolves a stepped profile 270 degrees about the x axis, so
// its end faces are caps.
func (sc *slabScene) revolvedShaft(ctx context.Context, t *testing.T) *Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	corners := [][2]float64{{0, 0}, {20, 0}, {20, 5}, {10, 8}, {10, 10}, {0, 10}}
	pts := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		pts[i] = s.CreatePoint(c[0], c[1])
		s.Fix(pts[i])
	}
	for i := range pts {
		s.CreateLine(pts[i], pts[(i+1)%len(pts)])
	}
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
	part, err := Revolve(ctx, sc.doc, s, s.Profiles()[0], axis, AngleExtent{A: units.Degrees(270), Dir: Along})
	require.NoError(t, err)
	return part
}

// zPin builds a short z-axis cylinder of radius 1.5 at (x, y) that pierces the
// revolved shaft's cap in the sketch plane and ends inside the material, so it
// meets no other face of the shaft.
func (sc *slabScene) zPin(ctx context.Context, t *testing.T, x, y float64) *Body {
	t.Helper()
	s, err := sc.w.CreateSketch(sc.w.XY())
	require.NoError(t, err)
	c := s.CreatePoint(x, y)
	s.Fix(c)
	s.CreateCircle(c, 1.5)
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	pin, err := Extrude(ctx, sc.doc, s, s.Profiles()[0], Symmetric{D: units.Millimeters(4)})
	require.NoError(t, err)
	return pin
}

package decad_test

import (
	"bytes"
	"context"
	"encoding/json"
	"math"
	"runtime"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// shellBoxHeight is the sweep height of the plate every shell test hollows.
const shellBoxHeight = 20.0

// shellBox extrudes the 100×60 plate by shellBoxHeight — a straight prism whose
// two caps are the faces a shell removes.
func shellBox(t *testing.T) (*decad.Document, *decad.Body) {
	t.Helper()
	s, p := plateSketch(t)
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// bothCaps selects a prism's two caps: planar and normal to the z sweep axis.
func bothCaps() *decad.FaceQuery {
	return decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1)))
}

// holedBox extrudes the 100×60 plate with one rectangular hole (corners
// (x0,y0)-(x1,y1)) by shellBoxHeight — a straight prism whose section carries a
// single polygonal hole, for the both-caps holed refusals.
func holedBox(t *testing.T, x0, y0, x1, y1 float64) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateRectangle(x0, y0, x1, y1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	doc := decad.New()
	box, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	return box
}

// topCap selects a prism's end cap by its role.
func topCap(b *decad.Body) *decad.FaceQuery {
	return decad.Faces(decad.FaceCreatedBy(decad.FeatureRef{Step: b.Origin().Step, Role: roleCapEnd}))
}

// forwardingFaceSelector is a foreign selector implementation that embeds the
// built-in query to promote Selector's sealed marker, then overrides resolution.
type forwardingFaceSelector struct {
	*decad.FaceQuery
	calls *int
}

type offsetPreprocessingCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	entered         bool
}

type operationCancelContext struct {
	context.Context //nolint:containedctx // deterministic cancellation wrapper used only within one test call.
	target          string
	cancelErr       error
	entered         bool
}

func (c *operationCancelContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	for {
		frame, more := frames.Next()
		if strings.HasSuffix(frame.Function, "."+c.target) {
			c.entered = true
			if c.cancelErr != nil {
				return c.cancelErr
			}
			return context.Canceled
		}
		if !more {
			return c.Context.Err()
		}
	}
}

func (c *offsetPreprocessingCancelContext) Err() error {
	pcs := make([]uintptr, 32)
	frames := runtime.CallersFrames(pcs[:runtime.Callers(2, pcs)])
	for {
		frame, more := frames.Next()
		if strings.HasSuffix(frame.Function, ".offsetProfile") {
			c.entered = true
			return context.Canceled
		}
		if !more {
			return c.Context.Err()
		}
	}
}

func TestShellContextCancellationDuringOffsetLeavesReceiverLive(t *testing.T) {
	t.Parallel()
	doc, box := shellBox(t)
	before := doc.Recipe()
	ctx := &offsetPreprocessingCancelContext{Context: t.Context()}

	body, err := box.ShellContext(ctx, topCap(box), units.Millimeters(5))

	require.Nil(t, body)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

func TestShellContextCancellationDuringOffsetSetupLeavesReceiverLive(t *testing.T) {
	t.Parallel()
	doc, box := shellBox(t)
	before := doc.Recipe()
	ctx := &operationCancelContext{Context: t.Context(), target: "prismCornerLoopsBudget"}

	body, err := box.ShellContext(ctx, topCap(box), units.Millimeters(5))

	require.Nil(t, body)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

func TestShellContextCancellationDuringSectionSurveyLeavesReceiverLive(t *testing.T) {
	t.Parallel()
	doc, box := shellBox(t)
	before := doc.Recipe()
	ctx := &operationCancelContext{Context: t.Context(), target: "sectionInradius"}

	body, err := box.ShellContext(ctx, topCap(box), units.Millimeters(5))

	require.Nil(t, body)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

func TestShellContextCancellationDuringKernelSetupLeavesReceiverLive(t *testing.T) {
	t.Parallel()
	doc, box := shellBox(t)
	before := doc.Recipe()
	ctx := &operationCancelContext{Context: t.Context(), target: "newWallKernelBudget"}

	body, err := box.ShellContext(ctx, topCap(box), units.Millimeters(5))

	require.Nil(t, body)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, []*decad.Body{box}, doc.Bodies())
}

func TestShellContextCancellationDuringAuditPreservesError(t *testing.T) {
	t.Parallel()
	for _, cancelErr := range []error{context.Canceled, context.DeadlineExceeded} {
		t.Run(cancelErr.Error(), func(t *testing.T) {
			doc, box := manySidedPrism(t, 17)
			before := doc.Recipe()
			ctx := &operationCancelContext{
				Context:   t.Context(),
				target:    "auditOffsetSectionBudget",
				cancelErr: cancelErr,
			}

			body, err := box.ShellContext(ctx, topCap(box), units.Millimeters(5))

			require.Nil(t, body)
			require.True(t, err == cancelErr, "ShellContext must return the exact context error")
			require.True(t, ctx.entered)
			require.Equal(t, before, doc.Recipe())
			require.Equal(t, []*decad.Body{box}, doc.Bodies())
		})
	}
}

func (s forwardingFaceSelector) SelectFaces(body *decad.Body) ([]*decad.Face, error) {
	(*s.calls)++
	return s.FaceQuery.SelectFaces(body)
}

func TestShellSelectorAdmission(t *testing.T) {
	t.Parallel()
	t.Run("BuiltInQuery", func(t *testing.T) {
		doc, box := shellBox(t)

		_, err := box.Shell(bothCaps(), units.Millimeters(5))
		require.NoError(t, err)
		require.Len(t, doc.Recipe().Steps, 2)
		_, err = json.Marshal(doc.Recipe())
		require.NoError(t, err)
	})

	t.Run("ForeignImplementation", func(t *testing.T) {
		doc, box := shellBox(t)
		before := doc.Recipe()
		calls := 0
		foreign := forwardingFaceSelector{
			FaceQuery: bothCaps(),
			calls:     &calls,
		}

		_, err := box.Shell(foreign, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Zero(t, calls, `Shell rejects a foreign selector before invoking its callback`)
		require.Equal(t, before, doc.Recipe(), `a rejected selector records no step`)
		require.Equal(t, []*decad.Body{box}, doc.Bodies(), `a rejected selector does not retire the receiver`)
		_, err = json.Marshal(doc.Recipe())
		require.NoError(t, err)
	})

	t.Run("TypedNilQuery", func(t *testing.T) {
		doc, box := shellBox(t)
		before := doc.Recipe()
		var query *decad.FaceQuery
		var selector decad.FaceSelector = query

		_, err := box.Shell(selector, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.Equal(t, before, doc.Recipe(), `a typed nil selector records no step`)
		require.Equal(t, []*decad.Body{box}, doc.Bodies(), `a typed nil selector does not retire the receiver`)
	})
}

type unknownShellOption struct {
	decad.ShellOption
	calls *int
}

type unknownShellOptionIdent struct{}

func (o unknownShellOption) Ident() any {
	(*o.calls)++
	return unknownShellOptionIdent{}
}

func TestShellOptionDispatch(t *testing.T) {
	t.Parallel()
	tests := []struct {
		name string
		opts []decad.ShellOption
		want decad.ShellSense
	}{
		{name: "default", want: decad.Inward},
		{name: "inward", opts: []decad.ShellOption{decad.WithShellSense(decad.Inward)}, want: decad.Inward},
		{name: "outward", opts: []decad.ShellOption{decad.WithShellSense(decad.Outward)}, want: decad.Outward},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			doc, box := shellBox(t)
			body, err := box.Shell(topCap(box), units.Millimeters(5), tc.opts...)
			require.NoError(t, err)
			require.NotNil(t, body)
			recipe := doc.Recipe()
			require.Len(t, recipe.Steps, 2)
			require.Equal(t, decad.ShellOpts{Sense: tc.want}, recipe.Steps[1].Opts)
		})
	}

	t.Run("ForeignImplementation", func(t *testing.T) {
		doc, box := shellBox(t)
		before := doc.Recipe()
		calls := 0
		opt := unknownShellOption{
			ShellOption: decad.WithShellSense(decad.Outward),
			calls:       &calls,
		}

		body, err := box.Shell(topCap(box), units.Millimeters(5), opt)
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "not a decad shell option")
		require.Zero(t, calls, `Shell rejects a foreign option before invoking its callback`)
		require.Equal(t, before, doc.Recipe())
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})

	t.Run("TypedNilImplementation", func(t *testing.T) {
		doc, box := shellBox(t)
		before := doc.Recipe()
		var opt *unknownShellOption
		var body *decad.Body
		var err error

		require.NotPanics(t, func() {
			body, err = box.Shell(topCap(box), units.Millimeters(5), opt)
		})
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "not a decad shell option")
		require.Equal(t, before, doc.Recipe())
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})

	t.Run("UnknownSense", func(t *testing.T) {
		doc, box := shellBox(t)
		before := doc.Recipe()

		body, err := box.Shell(topCap(box), units.Millimeters(5), decad.WithShellSense(decad.ShellSense(2)))
		require.Nil(t, body)
		require.ErrorIs(t, err, decad.ErrDegenerate)
		require.ErrorContains(t, err, "unknown shell sense 2")
		require.Equal(t, before, doc.Recipe())
		require.Equal(t, []*decad.Body{box}, doc.Bodies())
	})
}

func TestShellTubeInwardBox(t *testing.T) {
	t.Parallel()
	const th = 5.0
	h := shellBoxHeight
	doc, box := shellBox(t)

	// Both caps removed, inward — a tube: a prism over the annular section
	// {Outer: 100×60, Hole: 90×50}.
	body, err := box.Shell(bothCaps(), units.Millimeters(th))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)

	// Volume = (A_P − A_Q)·h, Q = P ⊖ t, both exact.
	aP := 100.0 * 60.0
	aQ := (100 - 2*th) * (60 - 2*th)
	wantVol := (aP - aQ) * h
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.True(t, vol.Bound.Equal(units.CubicMillimeters(0), 1e-12), `a shell introduces no bound`)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// A tube is a prismPayload: 4 outer walls + 4 cavity walls + 2 rim annuli.
	require.Len(t, body.Faces(), 10)

	// It tessellates (a prismPayload does), and Verify reads it Sound.
	mesh, err := body.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	require.NotNil(t, mesh)
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())

	// The wall survey reads the uniform wall thickness exactly.
	report, err = doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinWallThickness)
	require.True(t, br.MinWallThickness.Value.Equal(units.Millimeters(th), 1e-9),
		`the tube wall is the shell thickness, got %s`, br.MinWallThickness.Value)

	// The recipe records the shell intent and round-trips.
	recipe := doc.Recipe()
	require.Len(t, recipe.Steps, 2)
	require.Equal(t, decad.OpShell, recipe.Steps[1].Op)
	require.Equal(t, decad.ShellOpts{Sense: decad.Inward}, recipe.Steps[1].Opts)
	require.Len(t, recipe.Steps[1].Selectors, 1)
	buf, err := json.Marshal(recipe)
	require.NoError(t, err)
	var got decad.Recipe
	require.NoError(t, json.Unmarshal(buf, &got))
	require.Equal(t, recipe, got, `the recorded shell recipe round-trips`)
}

func TestShellTubeInwardCylinder(t *testing.T) {
	t.Parallel()
	const R, th = 20.0, 4.0
	h := 12.0
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), R)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	disk, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)

	body, err := disk.Shell(bothCaps(), units.Millimeters(th))
	require.NoError(t, err)
	requireManifold(t, body)

	// A cylindrical tube: outer cylinder, inner cylinder, two rim annuli.
	wantVol := (math.Pi*R*R - math.Pi*(R-th)*(R-th)) * h
	vol, err := body.Volume()
	require.NoError(t, err)
	got, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, got, 1e-9)

	cyl := 0
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cyl++
		}
	}
	require.Equal(t, 2, cyl, `a cylindrical tube has an outer and an inner cylinder`)
}

func TestShellCupInwardBox(t *testing.T) {
	t.Parallel()
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)

	// One cap removed, inward — a cup (cupPayload): the outer prism over P on
	// [z0, z1] and the cavity prism over Q = P ⊖ t on [z0 + t, z1].
	body, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)

	// Volume = A_P·h − A_Q·(h − t).
	aP := 100.0 * 60.0
	aQ := (100 - 2*th) * (60 - 2*th)
	wantVol := aP*h - aQ*(h-th)
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Exact, vol.Exactness)
	require.True(t, vol.Bound.Equal(units.CubicMillimeters(0), 1e-12))
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// Faces: 4 outer walls + 4 cavity walls + capStart + shellCap + rim = 11.
	require.Len(t, body.Faces(), 11)
	roles := shellRoleSet(body)
	require.Contains(t, roles, roleCapStart)
	require.Contains(t, roles, "shellCap")
	require.Contains(t, roles, "rim(0)")
	require.Contains(t, roles, "side(0,0)")
	require.Contains(t, roles, "shellSide(0,0)")

	// A shelled body has no void — the opening joins inner and outer into one
	// shell (docs/modify-design.md §8).
	require.False(t, body.IsSolid() && anyVoid(body), `a cup has no void shell`)
	for _, sh := range body.Shells() {
		require.False(t, sh.IsVoid())
	}
}

// TestShellCupFloorCarriesThicknessConversion keeps a cup floor derived from
// a non-millimetre thickness from claiming the rescaled level was recorded.
func TestShellCupFloorCarriesThicknessConversion(t *testing.T) {
	t.Parallel()
	thickness := units.Inches(0.1)
	floor, err := thickness.In(units.Millimeter)
	require.NoError(t, err)

	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), thickness)
	require.NoError(t, err)

	floorVertices := 0
	for _, vertex := range cup.Vertices() {
		position := vertex.Position()
		if position.Value.Z != floor {
			continue
		}
		require.Equal(t, decad.Approximate, position.Exactness)
		require.Positive(t, position.Bound.Mag())
		floorVertices++
	}
	require.Equal(t, 4, floorVertices, "the rectangular cavity floor has four vertices")
}

func TestShellCupOutwardBox(t *testing.T) {
	t.Parallel()
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)

	body, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)

	// Volume = A_Q·(h + t) − A_P·h, Q = P ⊕ t. The outward dilation ROUNDS the
	// four convex corners with quarter arcs of radius t, so A_Q is the 110×70
	// box less the (4 − π)t² the rounds cut away (§7).
	aP := 100.0 * 60.0
	aQ := (100+2*th)*(60+2*th) - (4-math.Pi)*th*th
	wantVol := aQ*(h+th) - aP*h
	vol, err := body.Volume()
	require.NoError(t, err)
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)
	// Outer walls: 4 straight + 4 rounded-corner cylinders (8). Cavity walls: 4
	// (the rectangular P). Plus capStart, shellCap, rim.
	require.Len(t, body.Faces(), 15)
}

func TestShellCupPlacedComposes(t *testing.T) {
	t.Parallel()
	const th = 5.0
	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	want, err := cup.Volume()
	require.NoError(t, err)

	// Body.Placed re-evaluates the cupPayload under the composed motion; volume
	// is invariant under a rigid move.
	motion, err := r3.Translation(r3.NewVec(10, 20, 30))
	require.NoError(t, err)
	moved, err := cup.Placed(motion)
	require.NoError(t, err)
	require.True(t, moved.IsSolid())
	requireManifold(t, moved)
	got, err := moved.Volume()
	require.NoError(t, err)
	require.True(t, got.Value.Equal(want.Value, 1e-9), `a rigid move preserves volume`)
	bounds, err := moved.Bounds()
	require.NoError(t, err)
	require.InDelta(t, 10.0, bounds.Min.X, 1e-9, `the placed cup's box shifts by the motion`)
}

// circleHoledBox extrudes the 100×60 plate carrying the given circular holes
// (each [cx, cy, r]) by shellBoxHeight — a straight prism whose section has
// k ≥ 1 holes, the posts a one-cap shell must wrap.
func circleHoledBox(t *testing.T, holes ...[3]float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	for _, h := range holes {
		s.CreateCircle(s.CreatePoint(h[0], h[1]), h[2])
	}
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == len(holes) {
			prof = p
		}
	}
	require.NotNil(t, prof)
	doc := decad.New()
	box, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	return doc, box
}

func TestShellCupHoledInward(t *testing.T) {
	t.Parallel()
	const th, rh = 5.0, 8.0
	h := shellBoxHeight
	doc, box := circleHoledBox(t, [3]float64{50, 30, rh})

	// One cap removed, inward, on a section with one central hole — a holed cup:
	// a wall around the pocket, a floor, and a POST (a tube wall) around the hole
	// rising from the floor, all one lump.
	body, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)
	require.Len(t, body.Lumps(), 1, `a one-cap holed cup is one lump — the floor joins every band`)
	for _, sh := range body.Shells() {
		require.False(t, sh.IsVoid(), `an opening joins inner and outer into one non-void shell`)
	}

	// Volume = A_P·h − A_Q·(h − t): the outer prism over P less the cavity prism
	// over Q = P ⊖ t, each on its own interval. Inward, the hole GROWS by t
	// (its wall's material lies outside it), so Q's hole is radius rh + t.
	aP := 100.0*60.0 - math.Pi*rh*rh
	aQ := (100-2*th)*(60-2*th) - math.Pi*(rh+th)*(rh+th)
	wantVol := aP*h - aQ*(h-th)
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// Area = 2·A_P + perim_O·h_o + perim_C·h_c, summed over every loop.
	perimO := 2*(100.0+60.0) + 2*math.Pi*rh
	perimC := 2*(90.0+50.0) + 2*math.Pi*(rh+th)
	wantArea := 2*aP + perimO*h + perimC*(h-th)
	area, err := body.Area()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, area.Exactness)
	require.Positive(t, area.Bound.Base())
	gotArea, err := area.Value.In(units.SquareMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantArea, gotArea, 1e-9)

	// Centroid: symmetric in x and y about the hole, sitting low over the floor.
	massO, massC := aP*h, aQ*(h-th)
	zO, zC := h/2, (th+h)/2
	wantZ := (massO*zO - massC*zC) / (massO - massC)
	c, err := body.Centroid()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, c.Exactness)
	require.Positive(t, c.Bound.Base())
	require.InDelta(t, 50.0, c.Value.X, 1e-9)
	require.InDelta(t, 30.0, c.Value.Y, 1e-9)
	require.InDelta(t, wantZ, c.Value.Z, 1e-9)

	// Faces: 4 outer walls + 1 tunnel cylinder + 4 cavity walls + 1 post cylinder
	// + capStart + shellCap + rim(0) + rim(1) = 14, with two cylinders.
	require.Len(t, body.Faces(), 14)
	cyl := 0
	for _, f := range body.Faces() {
		if _, ok := f.Surface().(decad.Cylinder); ok {
			cyl++
		}
	}
	require.Equal(t, 2, cyl, `the tunnel and the post are each a cylinder`)

	// Roles are minted from the RESULT's own record (§11): a rim per loop, and
	// the hole's own wall (loop 1) in both the outer and the cavity index space.
	roles := shellRoleSet(body)
	for _, want := range []string{roleCapStart, "shellCap", "rim(0)", "rim(1)", "side(0,0)", "side(1,0)", "shellSide(0,0)", "shellSide(1,0)"} {
		require.Contains(t, roles, want, `missing role %q`, want)
	}

	// A lone cup verifies Sound (no pairs to clear, valid by construction).
	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
	require.True(t, report.Trustworthy())
}

// rimByRole returns the body's face carrying the given rim role.
func rimByRole(t *testing.T, b *decad.Body, role string) *decad.Face {
	t.Helper()
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			if o.Role == role {
				return f
			}
		}
	}
	t.Fatalf("no face for role %q", role)
	return nil
}

// TestShellCupHoledRimLoopOrder pins the outer-loop-first contract of
// Face.Loops() (topology.go) on a holed cup's rims. rim(0) bounds the outer
// region (its outer boundary is O), and a POST rim rim(i≥1) bounds a solid
// column the cup wraps (its outer boundary is the wider cavity opening C) — in
// both, the loop flagged IsOuter must head the slice, so any consumer taking
// Loops()[0] as the outer contour reads it correctly.
func TestShellCupHoledRimLoopOrder(t *testing.T) {
	t.Parallel()
	const th, rh = 5.0, 8.0
	_, box := circleHoledBox(t, [3]float64{50, 30, rh})
	body, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	for _, role := range []string{"rim(0)", "rim(1)"} {
		rim := rimByRole(t, body, role)
		loops := rim.Loops()
		require.Len(t, loops, 2, `%s is a rim annulus with two loops`, role)
		require.True(t, loops[0].IsOuter(), `%s must list its outer loop first`, role)
		require.False(t, loops[1].IsOuter(), `%s must list its hole loop second`, role)
	}
}

func TestShellCupHoledTwoPosts(t *testing.T) {
	t.Parallel()
	const th, rh = 5.0, 6.0
	h := shellBoxHeight
	doc, box := circleHoledBox(t, [3]float64{30, 30, rh}, [3]float64{70, 30, rh})

	body, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)
	require.Len(t, body.Lumps(), 1, `two posts on one floor is still one lump`)

	// Volume with two holes, each growing to radius rh + t in the cavity.
	aP := 100.0*60.0 - 2*math.Pi*rh*rh
	aQ := (100-2*th)*(60-2*th) - 2*math.Pi*(rh+th)*(rh+th)
	wantVol := aP*h - aQ*(h-th)
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// A rim per loop: outer plus one per post.
	roles := shellRoleSet(body)
	for _, want := range []string{"rim(0)", "rim(1)", "rim(2)", "shellSide(1,0)", "shellSide(2,0)"} {
		require.Contains(t, roles, want, `missing role %q`, want)
	}

	report, err := doc.Verify(t.Context())
	require.NoError(t, err)
	require.Equal(t, decad.Sound, report.Status)
}

func TestShellCupHoledOutward(t *testing.T) {
	t.Parallel()
	const th, rh = 4.0, 12.0
	h := shellBoxHeight
	_, box := circleHoledBox(t, [3]float64{50, 30, rh})

	// Outward: the original solid P is the cavity, and the wall grows off it. The
	// hole SHRINKS by t in the outer dilation Q (its wall material lies outside
	// it), so the outer region's tunnel is radius rh − t and the pocket's post is
	// the original radius rh.
	body, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)
	require.Len(t, body.Lumps(), 1)

	// Q = P ⊕ t: the four convex corners round with quarter arcs of radius t, and
	// the central hole shrinks to radius rh − t. Volume = A_Q·(h + t) − A_P·h.
	aP := 100.0*60.0 - math.Pi*rh*rh
	aQ := (100+2*th)*(60+2*th) - (4-math.Pi)*th*th - math.Pi*(rh-th)*(rh-th)
	wantVol := aQ*(h+th) - aP*h
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)
}

func TestShellCupHoledRectangularPost(t *testing.T) {
	t.Parallel()
	const th = 5.0
	// A rectangular hole gives a rectangular post; its grown cavity hole rounds
	// its corners (an inward-reflex corner offsets to an arc, §7), so the post
	// wall is four planes joined by four corner cylinders — all still one lump.
	box := holedBox(t, 40, 20, 60, 40)
	body, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	require.True(t, body.IsSolid())
	requireManifold(t, body)
	require.Len(t, body.Lumps(), 1)
	for _, sh := range body.Shells() {
		require.False(t, sh.IsVoid())
	}

	// Volume: P is the 100×60 plate less the 20×20 hole; Q erodes the outer to
	// 90×50 and GROWS the hole to a 30×30 rounded square (corners of radius t).
	h := shellBoxHeight
	aP := 100.0*60.0 - 20.0*20.0
	holeQ := 30.0*30.0 - (4-math.Pi)*th*th
	aQ := (100-2*th)*(60-2*th) - holeQ
	wantVol := aP*h - aQ*(h-th)
	vol, err := body.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, vol.Exactness)
	require.Positive(t, vol.Bound.Base())
	gotVol, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	require.InDelta(t, wantVol, gotVol, 1e-9)

	// The post rim is a rounded-outer/sharp-inner band whose corner tangent
	// points sit on the tunnel's own edge lines — the bridge-collinear case. It
	// triangulates into a watertight band: every directed edge is matched, the
	// tunnel and the rounded post are both present, and the mesh under-encloses
	// the exact volume by no more than the chorded post corners' area over the
	// cavity height (the only curved feature; the outer plate is planar).
	tol := 0.1
	mesh, err := body.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	tunnel := map[[2]float64]bool{{40, 20}: false, {60, 20}: false, {40, 40}: false, {60, 40}: false}
	post := 0
	for _, v := range mesh.Vertices() {
		if _, ok := tunnel[[2]float64{v.X, v.Y}]; ok {
			tunnel[[2]float64{v.X, v.Y}] = true
		}
		// The rounded post corners sit at radius th about the tunnel corners.
		for _, c := range [][2]float64{{40, 20}, {60, 20}, {60, 40}, {40, 40}} {
			if math.Abs(math.Hypot(v.X-c[0], v.Y-c[1])-th) < 1e-9 {
				post++
			}
		}
	}
	for c, seen := range tunnel {
		require.True(t, seen, `tunnel corner %v is meshed`, c)
	}
	require.Positive(t, post, `the rounded post wall is meshed`)

	mv := meshVolume(mesh)
	slack := 2 * math.Pi * th * mesh.Bound().Mag() * (h - th)
	require.Less(t, mv, gotVol, `the chorded post inscribes its arcs, so the mesh under-encloses`)
	require.Greater(t, mv, gotVol-slack, `the deficit is only the chorded post corners`)
}

func TestShellRefusals(t *testing.T) {
	t.Parallel()
	t.Run("holed both caps is S12 unsupported", func(t *testing.T) {
		w := sketch.NewWorld()
		s, err := w.CreateSketch(w.XY())
		require.NoError(t, err)
		rect := s.CreateRectangle(0, 0, 100, 60)
		s.Fix(rect.A)
		s.CreateCircle(s.CreatePoint(70, 30), 10)
		_, err = s.Solve(t.Context())
		require.NoError(t, err)
		var prof *sketch.Profile
		for _, p := range s.Profiles() {
			if len(p.Holes) == 1 {
				prof = p
			}
		}
		require.NotNil(t, prof)
		doc := decad.New()
		box, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
		require.NoError(t, err)
		// The 5 mm inward wall leaves the radius-10 hole (2t = 10 < 20 diameter),
		// so the hole survives the offset and the refusal is the lump count (S12),
		// not a dropped feature.
		_, err = box.Shell(bothCaps(), units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrUnsupported, `1 + k lumps has no prismPayload`)
		require.Contains(t, err.Error(), "disjoint lumps", `a surviving hole refuses via S12, the lump count`)
	})

	t.Run("offset crossing keeps the Shell diagnostic", func(t *testing.T) {
		// The hole is 5 mm from the outer wall. A 3 mm inward offset moves the
		// outer wall past the hole's expanded boundary, so the shared audit
		// reaches its crossing refusal before the both-caps lump-count gate.
		box := holedBox(t, 5, 25, 15, 35)
		_, err := box.Shell(bothCaps(), units.Millimeters(3))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Equal(t, `decad: not supported by the current evaluator: the rewrite crosses itself; a resolving kernel is not available`,
			err.Error(), `Shell keeps its established shared-audit diagnostic`)
	})

	t.Run("outward offset erasing a hole is S11a, not the S12 lump count", func(t *testing.T) {
		// A 10×10 hole with a 6 mm OUTWARD wall: 2t = 12 > 10, so the offset
		// erodes the hole to nothing — a dropped loop (S11a). The erased loop
		// keeps its walk sense (its signed area does not change sign), so S8
		// cannot see it and it must be caught as the offset is built — before the
		// B4/S12 lump-count branch. Both are ErrUnsupported, so assert the message
		// sub-case to prove S11a fired, not S12.
		box := holedBox(t, 65, 25, 75, 35)
		_, err := box.Shell(bothCaps(), units.Millimeters(6), decad.WithShellSense(decad.Outward))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "drops a section feature", `the erased hole is S11a, a dropped feature`)
		require.NotContains(t, err.Error(), "disjoint lumps", `S11a is antecedent to S12; the lump count is never reached`)
	})

	t.Run("outward offset keeping a hole is S12, not a drop", func(t *testing.T) {
		// The same 10×10 hole with a 3 mm OUTWARD wall: 2t = 6 < 10, so the hole
		// survives (it shrinks to 4×4). The offset drops nothing, so the both-caps
		// holed refusal is the S12 lump count — proving the drop detection did not
		// over-broaden onto a valid offset.
		box := holedBox(t, 65, 25, 75, 35)
		_, err := box.Shell(bothCaps(), units.Millimeters(3), decad.WithShellSense(decad.Outward))
		require.ErrorIs(t, err, decad.ErrUnsupported)
		require.Contains(t, err.Error(), "disjoint lumps", `a surviving hole refuses via S12, the lump count`)
	})

	t.Run("t at or past the inradius is S10 degenerate", func(t *testing.T) {
		_, box := shellBox(t)
		// Inradius of a 100×60 rectangle is 30; a 35 mm inward wall eats it.
		_, err := box.Shell(bothCaps(), units.Millimeters(35))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	t.Run("t past the sweep height on a kept cap is S10 degenerate", func(t *testing.T) {
		_, box := shellBox(t)
		// 25 mm < inradius 30 (the section survives) but ≥ the 20 mm sweep, so
		// the kept cap's floor eats the cavity.
		_, err := box.Shell(topCap(box), units.Millimeters(25))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})

	t.Run("a topology-changing offset is S11 unsupported", func(t *testing.T) {
		_, box := shellBox(t)
		rounded, err := box.Fillet(verticalEdges(), units.Millimeters(2))
		require.NoError(t, err)
		// A 5 mm inward wall erodes the 2 mm corner arcs past zero — a dropped
		// feature, which needs a trimmed-offset kernel.
		_, err = rounded.Shell(bothCaps(), units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("removing a side wall is S2 unsupported", func(t *testing.T) {
		_, box := shellBox(t)
		wall := decad.Faces(decad.FaceCreatedBy(decad.FeatureRef{Step: box.Origin().Step, Role: "side(0,0)"}))
		_, err := box.Shell(wall, units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	})

	t.Run("a query matching nothing is loud", func(t *testing.T) {
		_, box := shellBox(t)
		_, err := box.Shell(decad.Faces(decad.Cylindrical()), units.Millimeters(5))
		require.ErrorIs(t, err, decad.ErrNoMatch)
	})

	t.Run("a zero thickness is S14 degenerate", func(t *testing.T) {
		_, box := shellBox(t)
		_, err := box.Shell(bothCaps(), units.Millimeters(0))
		require.ErrorIs(t, err, decad.ErrDegenerate)
	})
}

func TestShellInwardSectionWorkBudget(t *testing.T) {
	t.Parallel()
	const segments = 128
	pts := make([][2]float64, segments)
	for i := range pts {
		theta := 2 * math.Pi * float64(i) / segments
		pts[i] = [2]float64{100 * math.Cos(theta), 100 * math.Sin(theta)}
	}
	s, profile := polygonSketch(t, pts)
	doc := decad.New()
	box, err := doc.Extrude(s, profile, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
	require.NoError(t, err)
	before := snapshotDocument(t, doc)

	body, err := box.Shell(bothCaps(), units.Millimeters(1))
	require.Nil(t, body)
	require.ErrorIs(t, err, decad.ErrUnsupported)
	require.Contains(t, err.Error(), "work budget")
	require.Contains(t, err.Error(), "candidate-family visits")
	require.Equal(t, before.bodies, doc.Bodies(), `a refused shell must preserve live body membership and order`)
	recipe, marshalErr := json.Marshal(doc.Recipe())
	require.NoError(t, marshalErr)
	require.Equal(t, before.recipe, recipe, `a refused shell must not append a recipe step`)
	_, err = box.Duplicate()
	require.NoError(t, err, `the receiver must remain live after a work-budget refusal`)
}

func TestShellCupDownstream(t *testing.T) {
	t.Parallel()
	const th = 5.0

	t.Run("MinWallThickness is exact", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		requireWall(t, br, decad.Exact, th)
		require.Equal(t, decad.Sound, br.Status)
		require.True(t, report.Trustworthy())
	})

	t.Run("a lone cup verifies Sound", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status)
		require.True(t, report.Trustworthy())
	})

	t.Run("a box-disjoint cup pair is Sound, but WithClearances invokes the kernel and reads Suspect", func(t *testing.T) {
		doc, box1 := shellBox(t)
		cup1, err := box1.Shell(topCap(box1), units.Millimeters(th))
		require.NoError(t, err)
		// Move the first cup far clear, then build a second cup at the origin —
		// two live, box-disjoint cups in one document.
		far, err := r3.Translation(r3.NewVec(500, 0, 0))
		require.NoError(t, err)
		_, err = cup1.Placed(far)
		require.NoError(t, err)
		s2, p2 := plateSketch(t)
		box2, err := doc.Extrude(s2, p2, decad.Distance{D: units.Millimeters(shellBoxHeight), Dir: decad.Along})
		require.NoError(t, err)
		_, err = box2.Shell(topCap(box2), units.Millimeters(th))
		require.NoError(t, err)
		require.Len(t, doc.Bodies(), 2)

		// The cheap box test proves the pair disjoint; the kernel is never
		// invoked, so both cups verify Sound.
		report, err := doc.Verify(t.Context())
		require.NoError(t, err)
		require.Equal(t, decad.Sound, report.Status)

		// WithClearances asks for the gap, which invokes the kernel; it has no
		// cupPayload case, so the pair reads Suspect — never a fabricated pass.
		report, err = doc.Verify(t.Context(), decad.WithClearances())
		require.NoError(t, err)
		require.Equal(t, decad.Suspect, report.Status, `an invoked cup pair is staged`)
		require.False(t, report.Trustworthy())
	})
}

// shellRoleSet gathers every provenance role on a body's faces.
func shellRoleSet(b *decad.Body) map[string]struct{} {
	roles := map[string]struct{}{}
	for _, f := range b.Faces() {
		for _, o := range f.Origins() {
			roles[o.Role] = struct{}{}
		}
	}
	return roles
}

// anyVoid reports whether any shell of the body bounds a void.
func anyVoid(b *decad.Body) bool {
	for _, sh := range b.Shells() {
		if sh.IsVoid() {
			return true
		}
	}
	return false
}

// cylinderCup extrudes a disk of radius R to height h and shells its top cap
// inward by th — a cup with a cylindrical outer wall, a cylindrical cavity wall
// of radius R − th, and a circular floor.
func cylinderCup(t *testing.T, R, h, th float64) (*decad.Document, *decad.Body) {
	t.Helper()
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), R)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	disk, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	cup, err := disk.Shell(topCap(disk), units.Millimeters(th))
	require.NoError(t, err)
	return doc, cup
}

// polyArea is the area of the regular n-gon inscribed in a circle of radius r —
// the exact area a cup's chorded circular region encloses.
func polyArea(r float64, n int) float64 {
	return float64(n) / 2 * r * r * math.Sin(2*math.Pi/float64(n))
}

func requireCupWall(t *testing.T, doc *decad.Document, tool, want float64, status decad.Status) {
	t.Helper()
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(tool)))
	require.NoError(t, err)
	require.Len(t, report.Bodies, 1)
	requireWall(t, report.Bodies[0], decad.Exact, want)
	require.Equal(t, status, report.Bodies[0].Status)
	require.Equal(t, status, report.Status)
}

func TestShellCupWallContextCancellationDuringOffset(t *testing.T) {
	t.Parallel()
	doc, box := shellBox(t)
	_, err := box.Shell(topCap(box), units.Millimeters(5))
	require.NoError(t, err)

	ctx := &offsetPreprocessingCancelContext{Context: t.Context()}
	report, err := doc.Verify(ctx, decad.WithMinWallThickness(units.Millimeters(1)))

	require.Nil(t, report)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
}

func TestShellCupWallContextCancellationDuringFollowUpLeavesDocumentUnchanged(t *testing.T) {
	t.Parallel()
	doc, box := shellBox(t)
	_, err := box.Shell(topCap(box), units.Millimeters(5))
	require.NoError(t, err)
	before := doc.Recipe()
	bodies := doc.Bodies()
	ctx := &operationCancelContext{Context: t.Context(), target: "recordLoopsBudget"}

	report, err := doc.Verify(ctx, decad.WithMinWallThickness(units.Millimeters(1)))

	require.Nil(t, report)
	require.ErrorIs(t, err, context.Canceled)
	require.True(t, ctx.entered)
	require.Equal(t, before, doc.Recipe())
	require.Equal(t, bodies, doc.Bodies())
}

func TestShellCupWallThickness(t *testing.T) {
	t.Parallel()
	const th = 5.0

	t.Run("inward box", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		requireCupWall(t, doc, th, th, decad.Sound)
	})

	t.Run("bottom-open mirror", func(t *testing.T) {
		const mirrorThickness = 0.1
		doc, box := shellBox(t)
		bottom := decad.Faces(decad.FaceCreatedBy(decad.CapStart(box)))
		_, err := box.Shell(bottom, units.Millimeters(mirrorThickness))
		require.NoError(t, err)
		requireCupWall(t, doc, mirrorThickness, mirrorThickness, decad.Sound)
	})

	t.Run("outward bottom-open mirror", func(t *testing.T) {
		const mirrorThickness = 0.1
		doc, box := shellBox(t)
		bottom := decad.Faces(decad.FaceCreatedBy(decad.CapStart(box)))
		_, err := box.Shell(
			bottom,
			units.Millimeters(mirrorThickness),
			decad.WithShellSense(decad.Outward),
		)
		require.NoError(t, err)
		// The wall is Exact and met, but this cup's mass-subtracted centroid
		// (outer minus a cavity nearly as large) carries a genuine ~5000mm
		// bound here — a real precision defect in evalCup's centroid formula
		// on a near-degenerate outward shell, separate from the tolerance
		// gate this file otherwise exercises. Suspect is the correct verdict.
		requireCupWall(t, doc, mirrorThickness, mirrorThickness, decad.Suspect)
	})

	t.Run("cylinder", func(t *testing.T) {
		doc, _ := cylinderCup(t, 20, 12, th)
		requireCupWall(t, doc, th, th, decad.Sound)
	})

	t.Run("outward rounded box", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
		require.NoError(t, err)
		requireCupWall(t, doc, th, th, decad.Sound)
	})

	t.Run("holed cup", func(t *testing.T) {
		doc, box := circleHoledBox(t, [3]float64{50, 30, 8})
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		requireCupWall(t, doc, th, th, decad.Sound)
	})
}

func TestShellCupWallToolVerdict(t *testing.T) {
	t.Parallel()
	const th = 5.0
	doc, box := shellBox(t)
	_, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	requireCupWall(t, doc, th-1, th, decad.Sound)
	requireCupWall(t, doc, th, th, decad.Sound)
	requireCupWall(t, doc, th+1, th, decad.Violating)
}

func TestShellCupWallQualifyingPinch(t *testing.T) {
	t.Parallel()
	// The narrow right triangle has a roughly 5.7 degree material corner.
	// Under the default 15 degree allowance that junction admits spanning
	// balls tending to zero, so the wall is exactly zero and every legal tool
	// proves a violation.
	s, p := polygonSketch(t, [][2]float64{{0, 0}, {100, 0}, {100, 10}})
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	_, err = body.Shell(topCap(body), units.Millimeters(1))
	require.NoError(t, err)
	requireCupWall(t, doc, 0.001, 0, decad.Violating)

	// Below the actual corner angle, the junction is an edge rather than a
	// wall pinch, so the exact shell thickness is the answer.
	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(
		units.Millimeters(1),
		decad.WithDraftAllowance(units.Degrees(5)),
	))
	require.NoError(t, err)
	requireWall(t, report.Bodies[0], decad.Exact, 1)
	require.Equal(t, decad.Sound, report.Status)
}

func TestShellCupWallOutwardCavityQualifyingPinch(t *testing.T) {
	t.Parallel()
	const th = 0.3
	// The V-notch has a roughly 7.15 degree apex. An outward shell keeps that
	// notch on the cavity boundary, where its material-side corner qualifies
	// as a wall pinch under the default 15 degree allowance.
	s, p := polygonSketch(t, [][2]float64{
		{0, 0},
		{200, 0},
		{200, 100},
		{105, 100},
		{100, 20},
		{95, 100},
		{0, 100},
	})
	doc := decad.New()
	body, err := doc.Extrude(s, p, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	require.NoError(t, err)
	_, err = body.Shell(
		topCap(body),
		units.Millimeters(th),
		decad.WithShellSense(decad.Outward),
	)
	require.NoError(t, err)
	requireCupWall(t, doc, th, 0, decad.Violating)
}

func TestShellCupTessellateBox(t *testing.T) {
	t.Parallel()
	// A box cup is all planar: it triangulates exactly (bound zero), its mesh
	// is watertight, and the enclosed volume is the cup's own — proof the three
	// caps and both wall bands are wound outward and share their rings.
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	mesh, err := cup.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.True(t, mesh.Bound().Equal(units.Millimeters(0), 1e-12), `a box cup chords nothing, got %s`, mesh.Bound())

	aP := 100.0 * 60.0
	aQ := (100 - 2*th) * (60 - 2*th)
	wantVol := aP*h - aQ*(h-th)
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-9, `the mesh encloses exactly the cup volume`)

	// Every facet's source is a live face, and all eleven faces are covered.
	live := map[*decad.Face]struct{}{}
	for _, f := range cup.Faces() {
		live[f] = struct{}{}
	}
	covered := map[*decad.Face]struct{}{}
	for _, f := range mesh.SourceFaces() {
		_, ok := live[f]
		require.True(t, ok, `a facet's source is one of the cup's own faces`)
		covered[f] = struct{}{}
	}
	require.Len(t, covered, 11, `every cup face carries at least one facet`)

	// STL and OBJ round-trip deterministically — the writers are byte-stable.
	var stl1, stl2 bytes.Buffer
	require.NoError(t, cup.STL(&stl1))
	require.NoError(t, cup.STL(&stl2))
	require.Equal(t, stl1.Bytes(), stl2.Bytes(), `STL export is deterministic`)
	require.Positive(t, stl1.Len())
	var obj1, obj2 bytes.Buffer
	require.NoError(t, cup.OBJ(&obj1))
	require.NoError(t, cup.OBJ(&obj2))
	require.Equal(t, obj1.Bytes(), obj2.Bytes(), `OBJ export is deterministic`)
	require.Positive(t, obj1.Len())
}

func TestShellCupTessellateCylinder(t *testing.T) {
	t.Parallel()
	// A cylindrical cup: outer and cavity walls chord their circles, and the
	// cap and rim share that chording. The mesh is watertight, its bound is a
	// positive sagitta within tolerance, and its volume is exactly the chorded
	// outer prism less the chorded cavity prism.
	const R, h, th = 20.0, 12.0, 4.0
	tol := 0.5
	_, cup := cylinderCup(t, R, h, th)

	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.Positive(t, mesh.Bound().Mag(), `a chorded circle carries a real sagitta`)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	// Separate the two rings by their radius from the axis: the outer ring sits
	// at R, the cavity ring at R − th. Each sample owns two vertices.
	nO, nC := 0, 0
	for _, v := range mesh.Vertices() {
		switch rho := math.Hypot(v.X, v.Y); {
		case math.Abs(rho-R) < 1e-6:
			nO++
		case math.Abs(rho-(R-th)) < 1e-6:
			nC++
		default:
			t.Fatalf("a cup vertex sits off both rings, ρ = %g", rho)
		}
	}
	require.Positive(t, nO)
	require.Positive(t, nC)
	nO /= 2
	nC /= 2

	// The outer prism over [0, h] less the cavity prism over [th, h], each on
	// its inscribed polygon — exact.
	wantVol := polyArea(R, nO)*h - polyArea(R-th, nC)*(h-th)
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-9)

	// And the chorded volume converges on the true cup as the tolerance shrinks.
	trueVol := math.Pi*R*R*h - math.Pi*(R-th)*(R-th)*(h-th)
	require.InDelta(t, trueVol, meshVolume(mesh), trueVol*0.05, `the chorded cup approaches the analytic one`)
}

func TestShellCupTessellateReflected(t *testing.T) {
	t.Parallel()
	// A reflected placement flips handedness; the cup mesh must still come out
	// wound outward, enclosing the same positive volume.
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	mirror, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	refl, err := r3.Reflection(mirror)
	require.NoError(t, err)
	placed, err := cup.Placed(refl)
	require.NoError(t, err)

	mesh, err := placed.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	requireWatertight(t, mesh)

	aP := 100.0 * 60.0
	aQ := (100 - 2*th) * (60 - 2*th)
	wantVol := aP*h - aQ*(h-th)
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-9, `outward winding survives a reflected placement`)
}

func TestShellCupTessellateOutwardBox(t *testing.T) {
	t.Parallel()
	// An outward box cup dilates its rectangular section, landing the rounded
	// outer corners' tangent points on the cavity's own top and bottom edge
	// lines — the bridge-collinear rim band. It is a normal 5 mm-wide frame
	// (rounded outer, sharp inner), so it triangulates into a watertight mesh;
	// the only chorded feature is the four dilated outer corners, over the
	// outer prism's full height h + th.
	const th = 5.0
	h := shellBoxHeight
	_, box := shellBox(t)
	cup, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)

	tol := 0.1
	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	vol, err := cup.Volume()
	require.NoError(t, err)
	exact, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	slack := 2 * math.Pi * th * mesh.Bound().Mag() * (h + th)
	require.InDelta(t, exact, meshVolume(mesh), slack, `the mesh encloses the cup volume up to the chorded outer corners`)
}

func TestShellCupUndercutsBox(t *testing.T) {
	t.Parallel()
	const th = 5.0

	// Pulled straight out of its open top, a box cup has no undercut: every
	// wall is exactly perpendicular, the pocket floor and rim face the pull,
	// and the outer floor is exactly antiparallel (it separates, not hooks).
	t.Run("clear under an axial pull", func(t *testing.T) {
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		require.NotNil(t, br.Undercuts)
		require.Empty(t, br.Undercuts, `an axial pull frees the whole cup`)
		require.Equal(t, decad.Sound, br.Status)
		require.True(t, report.Trustworthy())
	})

	// Tilt the pull and three faces hook against it: the outer wall facing the
	// tilt, the matching cavity wall, and the outer floor.
	t.Run("tilt hooks three faces", func(t *testing.T) {
		doc, box := shellBox(t)
		cup, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		require.Len(t, br.Undercuts, 3)
		for _, f := range br.Undercuts {
			require.Equal(t, decad.KindPlane, f.Surface().Kind())
		}
		// The kept outer floor (capStart) is one of them.
		capStartRef := decad.FeatureRef{Step: cup.Origin().Step, Role: roleCapStart}
		found := false
		for _, f := range br.Undercuts {
			for _, o := range f.Origins() {
				if o == capStartRef {
					found = true
				}
			}
		}
		require.True(t, found, `the tilted-away outer floor is an undercut`)
		require.Equal(t, decad.Violating, br.Status)
	})
}

func TestShellCupMinRadius(t *testing.T) {
	t.Parallel()
	// A cylindrical cup's cavity wall curves away from the material: its radius
	// R − th is the tightest concave one. The convex outer cylinder is not a
	// concave feature and does not appear.
	t.Run("cylindrical cavity reads its radius", func(t *testing.T) {
		const R, h, th = 20.0, 12.0, 4.0
		doc, _ := cylinderCup(t, R, h, th)
		report, err := doc.Verify(t.Context(), decad.WithMinRadius())
		require.NoError(t, err)
		br := report.Bodies[0]
		require.NotNil(t, br.MinRadius)
		require.Equal(t, decad.Exact, br.MinRadius.Exactness)
		require.True(t, br.MinRadius.Value.Equal(units.Millimeters(R-th), 1e-9),
			`the cavity cylinder is the cup's one concave face, got %s`, br.MinRadius.Value)
		require.Equal(t, decad.Sound, br.Status)
		require.True(t, report.Trustworthy())
	})

	// A box cup has no curved face — its concave wall/floor edges carry no
	// radius, so the proven answer is nil.
	t.Run("box cup has no concave radius", func(t *testing.T) {
		const th = 5.0
		doc, box := shellBox(t)
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithMinRadius())
		require.NoError(t, err)
		br := report.Bodies[0]
		require.Nil(t, br.MinRadius, `a box cup has no concave curvature`)
		require.Equal(t, decad.Sound, br.Status)
		require.True(t, report.Trustworthy())
	})
}

// diskHoledCup extrudes an annular section (outer circle R, concentric hole rh)
// to height h — a straight prism with one circular hole, whose outward cup
// wraps into a cylindrical post. (The rectangular post, whose rim is
// bridge-collinear, is covered by TestShellCupHoledRectangularPost.)
func diskHoledCup(t *testing.T, R, rh, h float64) (*decad.Document, *decad.Body) {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), R)
	s.CreateCircle(s.CreatePoint(0, 0), rh)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var prof *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 1 {
			prof = p
		}
	}
	require.NotNil(t, prof)
	doc := decad.New()
	body, err := doc.Extrude(s, prof, decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
	require.NoError(t, err)
	return doc, body
}

// TestShellCupHoledTessellate meshes a HOLED cup (modify §9, D4): a box with one
// central circular hole, top cap shelled inward. The result wraps a wall around
// the pocket AND a POST (a tube wall) around the hole — the tunnel through the
// body and the post around it must both be present, or the enclosed volume is
// wrong. The mesh is watertight, its bound within tolerance, and its enclosed
// volume equals the SAME A_O·h_o − A_C·h_c the build asserts, evaluated on the
// chorded circles the mesh actually holds.
func TestShellCupHoledTessellate(t *testing.T) {
	t.Parallel()
	const th, rh = 5.0, 8.0
	h := shellBoxHeight
	_, box := circleHoledBox(t, [3]float64{50, 30, rh})
	cup, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)

	tol := 0.1
	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.Positive(t, mesh.Bound().Mag(), `the tunnel and post circles carry a real sagitta`)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	// The tunnel ring sits at radius rh, the post ring at rh + th, both about the
	// hole centre — a post-missing mesh would carry neither. Count each; every
	// sample owns a lo and a hi vertex.
	nTunnel, nPost := 0, 0
	for _, v := range mesh.Vertices() {
		switch rho := math.Hypot(v.X-50, v.Y-30); {
		case math.Abs(rho-rh) < 1e-6:
			nTunnel++
		case math.Abs(rho-(rh+th)) < 1e-6:
			nPost++
		}
	}
	require.Positive(t, nTunnel, `the tunnel through the body is meshed`)
	require.Positive(t, nPost, `the post around the tunnel is meshed`)
	nTunnel /= 2
	nPost /= 2

	// Volume = A_O·h − A_C·(h − t): the outer box less its chorded tunnel over
	// the full height, less the inner box less its chorded post over the pocket.
	aO := 100.0*60.0 - polyArea(rh, nTunnel)
	aC := (100-2*th)*(60-2*th) - polyArea(rh+th, nPost)
	wantVol := aO*h - aC*(h-th)
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-6, `the mesh encloses exactly the chorded holed-cup volume, post included`)

	// Every one of the fourteen faces carries at least one facet, and every
	// facet's source is a live face.
	live := map[*decad.Face]struct{}{}
	for _, f := range cup.Faces() {
		live[f] = struct{}{}
	}
	covered := map[*decad.Face]struct{}{}
	for _, f := range mesh.SourceFaces() {
		_, ok := live[f]
		require.True(t, ok, `a facet's source is one of the cup's own faces`)
		covered[f] = struct{}{}
	}
	require.Len(t, covered, 14, `every face of the holed cup carries a facet`)

	// STL and OBJ export deterministically.
	var stl1, stl2 bytes.Buffer
	require.NoError(t, cup.STL(&stl1))
	require.NoError(t, cup.STL(&stl2))
	require.Equal(t, stl1.Bytes(), stl2.Bytes(), `STL export is deterministic`)
	require.Positive(t, stl1.Len())
	var obj1, obj2 bytes.Buffer
	require.NoError(t, cup.OBJ(&obj1))
	require.NoError(t, cup.OBJ(&obj2))
	require.Equal(t, obj1.Bytes(), obj2.Bytes(), `OBJ export is deterministic`)
}

// TestShellCupHoledTessellateOutward meshes an OUTWARD holed cup: an annular
// disk (outer R, hole rh) shelled outward on its top. Outward, the outer grows
// to R + th and the section hole shrinks to rh − th (the tunnel), while the
// original circles R and rh become the pocket wall and the post. All four rings
// are circular, so the rim bands close cleanly and the mesh is watertight with
// the post present.
func TestShellCupHoledTessellateOutward(t *testing.T) {
	t.Parallel()
	const R, rh, th = 30.0, 8.0, 4.0
	h := shellBoxHeight
	_, disk := diskHoledCup(t, R, rh, h)
	cup, err := disk.Shell(topCap(disk), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)

	tol := 0.2
	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	// Bucket the vertices by radius about the axis: the four rings at R + th
	// (outer), R (pocket wall), rh (post), rh − th (tunnel).
	counts := map[float64]int{}
	for _, v := range mesh.Vertices() {
		rho := math.Hypot(v.X, v.Y)
		for _, want := range []float64{R + th, R, rh, rh - th} {
			if math.Abs(rho-want) < 1e-6 {
				counts[want]++
			}
		}
	}
	for _, want := range []float64{R + th, R, rh, rh - th} {
		require.Positive(t, counts[want], `ring at radius %g is meshed`, want)
	}
	nOouter := counts[R+th] / 2
	nOtunnel := counts[rh-th] / 2
	nCouter := counts[R] / 2
	nCpost := counts[rh] / 2

	// Outward, the outer prism runs the full height plus the added floor below
	// (h + th) and the cavity runs h — the mirror of the inward heights.
	aO := polyArea(R+th, nOouter) - polyArea(rh-th, nOtunnel)
	aC := polyArea(R, nCouter) - polyArea(rh, nCpost)
	wantVol := aO*(h+th) - aC*h
	require.InDelta(t, wantVol, meshVolume(mesh), 1e-6, `an outward holed cup encloses its chorded volume, post included`)
}

// TestShellCupHoledTessellateOutwardBox meshes a circle-holed OUTWARD box cup:
// the outer region dilates to a rounded rectangle whose corner tangent points
// land on the cavity's own edge lines — the bridge-collinear rim — and the
// circular hole grows a post. It is a normal band, so it triangulates into a
// watertight mesh; the chorded features are the four outer corners plus the
// tunnel and post circles.
func TestShellCupHoledTessellateOutwardBox(t *testing.T) {
	t.Parallel()
	const th, rh = 5.0, 8.0
	h := shellBoxHeight
	_, box := circleHoledBox(t, [3]float64{50, 30, rh})
	cup, err := box.Shell(topCap(box), units.Millimeters(th), decad.WithShellSense(decad.Outward))
	require.NoError(t, err)

	tol := 0.1
	mesh, err := cup.Tessellate(units.Millimeters(tol))
	require.NoError(t, err)
	requireWatertight(t, mesh)
	require.LessOrEqual(t, mesh.Bound().Mag(), tol)

	vol, err := cup.Volume()
	require.NoError(t, err)
	exact, err := vol.Value.In(units.CubicMillimeter)
	require.NoError(t, err)
	b := mesh.Bound().Mag()
	// Curved features chorded: the four outer corners (radius th) and tunnel
	// circle (radius rh − th) over the outer prism height h + th, and the post
	// circle (radius rh) over the cavity height h.
	slack := 2 * math.Pi * b * (th*(h+th) + (rh-th)*(h+th) + rh*h)
	require.InDelta(t, exact, meshVolume(mesh), slack, `the mesh encloses the holed cup volume up to its chorded arcs`)
}

// TestShellCupHoledUndercuts surveys a HOLED cup against a pull (modify §9, D2).
// The post walls (side(i,j)/shellSide(i,j), i ≥ 1) are now walked, so a post
// undercut is caught rather than dropped into a false all-clear.
func TestShellCupHoledUndercuts(t *testing.T) {
	t.Parallel()
	const th, rh = 5.0, 8.0

	// Pulled straight out of the open top, every wall is perpendicular (the
	// cylinders' normals are radial) and the outer floor exactly antiparallel:
	// no undercut, Sound.
	t.Run("clear under an axial pull", func(t *testing.T) {
		doc, box := circleHoledBox(t, [3]float64{50, 30, rh})
		_, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		require.NotNil(t, br.Undercuts)
		require.Empty(t, br.Undercuts, `an axial pull frees the whole holed cup`)
		require.Equal(t, decad.Sound, br.Status)
		require.True(t, report.Trustworthy())
	})

	// Tilt the pull and the post cylinder hooks against it — a full cylinder
	// sweeps every lateral normal, so part of the post opposes any non-axial
	// pull. The reading is Violating and names the post wall shellSide(1,0).
	t.Run("tilt hooks the post wall", func(t *testing.T) {
		doc, box := circleHoledBox(t, [3]float64{50, 30, rh})
		cup, err := box.Shell(topCap(box), units.Millimeters(th))
		require.NoError(t, err)
		report, err := doc.Verify(t.Context(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
		require.NoError(t, err)
		br := report.Bodies[0]
		require.Equal(t, decad.Violating, br.Status)

		postRef := decad.FeatureRef{Step: cup.Origin().Step, Role: "shellSide(1,0)"}
		var post *decad.Face
		for _, f := range br.Undercuts {
			for _, o := range f.Origins() {
				if o == postRef {
					post = f
				}
			}
		}
		require.NotNil(t, post, `the post wall is caught as an undercut`)
		require.Equal(t, decad.KindCylinder, post.Surface().Kind(), `the post is a cylinder`)
	})
}

// TestShellCupHoledMinRadius reads the tightest concave radius of a HOLED cup
// (modify §9, D3): the tunnel the post wraps is a hole through the body, radius
// rh, the one concave cylindrical face. The post's own outer cylinder curves
// TOWARD the material and is not concave, so the reading is rh, exact.
func TestShellCupHoledMinRadius(t *testing.T) {
	t.Parallel()
	const th, rh = 5.0, 8.0
	doc, box := circleHoledBox(t, [3]float64{50, 30, rh})
	_, err := box.Shell(topCap(box), units.Millimeters(th))
	require.NoError(t, err)
	report, err := doc.Verify(t.Context(), decad.WithMinRadius())
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinRadius, `the tunnel is a concave cylindrical face`)
	require.Equal(t, decad.Exact, br.MinRadius.Exactness)
	require.True(t, br.MinRadius.Value.Equal(units.Millimeters(rh), 1e-9),
		`the tunnel wall the post wraps is the tightest concave radius, got %s`, br.MinRadius.Value)
	require.Equal(t, decad.Sound, br.Status)
	require.True(t, report.Trustworthy())
}

// TestShellCupSeparatesComputedOpenAndExactFloorBounds keeps an all-planar cup
// from assigning its computed open end's displacement to the exact floor.
func TestShellCupSeparatesComputedOpenAndExactFloorBounds(t *testing.T) {
	t.Parallel()
	const (
		plateHeight = 1e12
		shortBy     = 1e-3
		rounding    = 2.34375e-05
	)
	s, plateProfile, pinProfile := plateAndPin(t)
	doc := decad.New()
	plate, err := doc.Extrude(s, plateProfile, decad.Distance{D: units.Millimeters(plateHeight), Dir: decad.Along})
	require.NoError(t, err)
	pin, err := doc.Extrude(s, pinProfile, decad.ToFace{
		Body:   plate,
		Face:   capEndFace(plate),
		Offset: units.Millimeters(-shortBy),
	})
	require.NoError(t, err)
	cup, err := pin.Shell(capEndFace(pin), units.Millimeters(1))
	require.NoError(t, err)

	floorVertices := 0
	for _, vertex := range cup.Vertices() {
		position := vertex.Position()
		if position.Value.Z != 0 && position.Value.Z != 1 {
			continue
		}
		floorVertices++
		require.Equal(t, decad.Exact, position.Exactness,
			`the explicitly stated floor level must remain exact`)
		require.True(t, position.Bound.Equal(units.Millimeters(0), 1e-12),
			`the explicitly stated floor level must carry no displacement`)
	}
	require.NotZero(t, floorVertices, `the cup has vertices on both floor levels`)

	mesh, err := cup.Tessellate(units.Millimeters(0.1))
	require.NoError(t, err)
	bound, err := mesh.Bound().In(units.Millimeter)
	require.NoError(t, err)
	require.GreaterOrEqual(t, bound, rounding)
}

func TestShellCupWallThicknessCarriesConversionDelta(t *testing.T) {
	t.Parallel()
	// A cup whose shell thickness is stated in a non-millimetre unit: the
	// wall reading is exactly the shell theorem's own t, but that t's own
	// unit conversion (magnitudeInBounded, shell.go) carries a displacement
	// cupWall now composes (docs/payload-verification-design.md §4.1)
	// instead of asserting Exact. A millimetre thickness stays Exact — see
	// TestShellCupWallThickness and this file's other cylinderCup cases, all
	// stated in millimetres.
	ws := sketch.NewWorld()
	s, err := ws.CreateSketch(ws.XY())
	require.NoError(t, err)
	s.CreateCircle(s.CreatePoint(0, 0), 20)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	doc := decad.New()
	disk, err := doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(12), Dir: decad.Along})
	require.NoError(t, err)
	_, err = disk.Shell(topCap(disk), units.Inches(0.2))
	require.NoError(t, err)

	report, err := doc.Verify(t.Context(), decad.WithMinWallThickness(units.Millimeters(1)))
	require.NoError(t, err)
	br := report.Bodies[0]
	require.NotNil(t, br.MinWallThickness)
	require.Equal(t, decad.Approximate, br.MinWallThickness.Exactness)
	value, err := br.MinWallThickness.Value.In(units.Millimeter)
	require.NoError(t, err)
	bound, err := br.MinWallThickness.Bound.In(units.Millimeter)
	require.NoError(t, err)
	require.Greater(t, bound, 0.0)
	const denoted = 0.2 * 25.4 // 0.2 inch in millimetres
	require.LessOrEqual(t, value-bound, denoted)
	require.GreaterOrEqual(t, value+bound, denoted)
}

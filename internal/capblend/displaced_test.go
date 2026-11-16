package capblend_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booloperand"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topo/topotest"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// The displaced cap-loop chamfer of agent-docs/carrier-record-design.md §8.
// The fixture is a 20 mm square with one 4 mm square hole whose rims are
// chamfered on both caps; the outer loop is the displaced one. The twin of a
// displaced build is the same payload built with every LoopDelta zero.

const dispLoopDelta = 0.01

func dispEnv() topo.Env { return topo.Env{Doc: &topo.Document{}, Ref: 3, Alloc: topotest.Alloc{}} }

func dispFrame(t *testing.T) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	return f
}

// holeLoop is a clockwise square hole of side s with its lower left corner at (u, v).
func holeLoop(u, v, s float64) sectionrecord.LoopRecord {
	pts := []sectionrecord.Point2{{U: u, V: v}, {U: u, V: v + s}, {U: u + s, V: v + s}, {U: u + s, V: v}}
	segs := make([]sectionrecord.CurveSegment, len(pts))
	for i := range pts {
		segs[i] = sectionrecord.LineSeg{Start: pts[i], End: pts[(i+1)%len(pts)], TStart: 0, TEnd: 1}
	}
	return sectionrecord.LoopRecord{Segments: segs}
}

func dispPayload(t *testing.T) capblend.Payload {
	t.Helper()
	set := capblend.Setback{Dc: 0.5, Ds: 0.5}
	return capblend.Payload{
		Profile:    momentinput.Profile{Outer: squareLoop(20), Holes: []sectionrecord.LoopRecord{holeLoop(8, 8, 4)}},
		Frame:      dispFrame(t),
		Z1:         10,
		Xform:      r3.Identity(),
		Start:      set,
		End:        set,
		StartLoops: map[int]bool{1: true},
		EndLoops:   map[int]bool{1: true},
	}
}

func dispTwin(t *testing.T) *topo.Body {
	t.Helper()
	b, err := capblend.Build(t.Context(), dispEnv(), dispPayload(t))
	require.NoError(t, err)
	return b
}

func dispBuild(t *testing.T, loopDelta []float64) *topo.Body {
	t.Helper()
	b, err := capblend.BuildDisplaced(t.Context(), dispEnv(), dispPayload(t), loopDelta)
	require.NoError(t, err)
	return b
}

func TestDisplacedBuildChargesTheOuterLoop(t *testing.T) {
	t.Parallel()
	twin := dispTwin(t)
	b := dispBuild(t, []float64{dispLoopDelta, 0})
	dp, ok := b.Payload.(capblend.DisplacedPayload)
	require.True(t, ok, "a nonzero LoopDelta stores a DisplacedPayload")
	require.Equal(t, []float64{dispLoopDelta, 0}, dp.LoopDelta)
	_, plain := b.Payload.(capblend.Payload)
	require.False(t, plain, "a type assertion on the plain payload never matches")
	require.NotEmpty(t, dp.Base.Patches)
	require.NotEmpty(t, dp.Base.BandDelta)

	// The outer loop's displacement area is at least 2·δ·(its 80 mm perimeter).
	area := 2 * dispLoopDelta * 80
	require.Equal(t, twin.Volume.Value, b.Volume.Value, "the held volume does not change")
	require.GreaterOrEqual(t, b.Volume.Bound.Base()-twin.Volume.Bound.Base(), area*10)

	twinFaces, faces := topobuild.FacesByRole(twin), topobuild.FacesByRole(b)
	for _, role := range []string{provenance.RoleCapStart, provenance.RoleCapEnd} {
		require.GreaterOrEqual(t, faces[role].AreaBound-twinFaces[role].AreaBound, area, role)
	}
	require.Greater(t, b.Centroid.Bound.Base(), twin.Centroid.Bound.Base(), "the in-plane moments carry A·(M+δ)")
	require.GreaterOrEqual(t, b.Bounds.Bound.Base(), dispLoopDelta, "the box is read through the charged extent")

	rim := 0
	for _, e := range topobuild.Edges(b) {
		for _, v := range []*topo.Vertex{e.Start, e.End} {
			if v == nil {
				continue
			}
			// every vertex on the outer wall's rim
			if onOuterRim(v.Position) {
				require.GreaterOrEqual(t, v.Bound.Base(), dispLoopDelta)
				rim++
			}
		}
	}
	require.Positive(t, rim)
	for _, e := range topobuild.Edges(twin) {
		for _, v := range []*topo.Vertex{e.Start, e.End} {
			if v != nil && onOuterRim(v.Position) {
				require.Less(t, v.Bound.Base(), dispLoopDelta)
			}
		}
	}
}

func onOuterRim(p r3.Vec) bool {
	edge := func(x float64) bool { return math.Abs(x) < 1e-9 || math.Abs(x-20) < 1e-9 }
	return (edge(p.X) || edge(p.Y)) && (math.Abs(p.Z) < 1e-9 || math.Abs(p.Z-10) < 1e-9)
}

func TestDisplacedBuildRefusals(t *testing.T) {
	t.Parallel()
	env := dispEnv()
	t.Run("all zero stores the plain payload", func(t *testing.T) {
		b := dispBuild(t, []float64{0, 0})
		_, ok := b.Payload.(capblend.Payload)
		require.True(t, ok)
	})
	for name, delta := range map[string][]float64{
		"wrong length":       {dispLoopDelta},
		"nonzero on chamfer": {dispLoopDelta, dispLoopDelta},
		"negative":           {-1, 0},
		"NaN":                {math.NaN(), 0},
		"infinite":           {math.Inf(1), 0},
	} {
		t.Run(name, func(t *testing.T) {
			_, err := capblend.BuildDisplaced(t.Context(), env, dispPayload(t), delta)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		})
	}
	for name, mutate := range map[string]func(*capblend.Payload){
		"draft":  func(p *capblend.Payload) { p.Draft = true },
		"fillet": func(p *capblend.Payload) { p.Fillet = true },
		"loft":   func(p *capblend.Payload) { p.LoftSource = &capblend.LoftSource{} },
	} {
		t.Run(name, func(t *testing.T) {
			cbp := dispPayload(t)
			mutate(&cbp)
			_, err := capblend.BuildDisplaced(t.Context(), env, cbp, []float64{dispLoopDelta, 0})
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
		})
	}
}

func TestDisplacedPlacedKeepsLoopDelta(t *testing.T) {
	t.Parallel()
	b := dispBuild(t, []float64{dispLoopDelta, 0})
	dp := b.Payload.(capblend.DisplacedPayload)
	move, err := r3.Translation(r3.NewVec(3, 4, 5))
	require.NoError(t, err)
	placed, err := dp.Placed(t.Context(), dispEnv(), move)
	require.NoError(t, err)
	got, ok := placed.Payload.(capblend.DisplacedPayload)
	require.True(t, ok)
	require.Equal(t, dp.LoopDelta, got.LoopDelta)
	require.Equal(t, move, got.Transform())
	require.Equal(t, "displacedCapBlendPayload", got.SurveyClass())
}

func TestDisplacedExtent(t *testing.T) {
	t.Parallel()
	twin := dispTwin(t).Payload.(capblend.Payload)
	dp := dispBuild(t, []float64{dispLoopDelta, 0}).Payload.(capblend.DisplacedPayload)
	_, _, refusedBound, err := dp.ExtentAlong(r3.NewVec(1, 0, 0))
	require.Zero(t, refusedBound)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)

	extent := func(g r3.Vec) (float64, float64, float64) {
		lo, hi, bound, err := dp.ExtentBoundedAlong(t.Context(), g, nil)
		require.NoError(t, err)
		return lo, hi, bound
	}
	twinExtent := func(g r3.Vec) (float64, float64, float64) {
		lo, hi, bound, err := twin.ExtentBoundedAlong(t.Context(), g, nil)
		require.NoError(t, err)
		return lo, hi, bound
	}
	for _, g := range []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)} {
		lo, hi, bound := extent(g)
		tlo, thi, tbound := twinExtent(g)
		require.Equal(t, [2]float64{tlo, thi}, [2]float64{lo, hi})
		require.GreaterOrEqual(t, bound, tbound+dispLoopDelta, "the outer loop holds the extreme")
	}
	_, _, bound := extent(r3.NewVec(0, 0, 1))
	_, _, tbound := twinExtent(r3.NewVec(0, 0, 1))
	require.Equal(t, tbound, bound, "along the sweep no in-plane displacement reaches the extreme")
}

func TestDisplacedTessellation(t *testing.T) {
	t.Parallel()
	twinBody := dispTwin(t)
	body := dispBuild(t, []float64{dispLoopDelta, 0})
	dp := body.Payload.(capblend.DisplacedPayload)

	t.Run("a tolerance at or below the largest displacement refuses", func(t *testing.T) {
		_, _, err := capblend.TessellateDisplacedMesh(t.Context(), body, dp, dispLoopDelta, true, new(topo.Mesh))
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Contains(t, err.Error(), "section displacement")
	})
	t.Run("above it the mesh closes and charges the tube", func(t *testing.T) {
		const chord = 0.05
		mesh, twin := new(topo.Mesh), new(topo.Mesh)
		_, _, err := capblend.TessellateDisplacedMesh(t.Context(), body, dp, chord, true, mesh)
		require.NoError(t, err)
		_, _, err = capblend.TessellateMesh(t.Context(), twinBody, twinBody.Payload.(capblend.Payload), chord, true, twin)
		require.NoError(t, err)
		require.True(t, mesh.SymDiffOK)
		require.NoError(t, booloperand.RequireVolumeProof(t.Context(), body, 0), "the proof and the mesh agree")
		require.NoError(t, topobuild.RequireClosedMesh(mesh))
		for role, f := range topobuild.FacesByRole(body) {
			if role == provenance.RoleCapStart || role == provenance.RoleCapEnd || len(role) > 6 && role[:6] == "side(0" {
				require.GreaterOrEqual(t, mesh.FaceBound[f], dispLoopDelta, role)
			}
		}
		area := 2 * dispLoopDelta * 80
		require.GreaterOrEqual(t, mesh.VolSymDiff-twin.VolSymDiff, area*10)
		require.Greater(t, mesh.AreaSlack, twin.AreaSlack)
	})
}

// The transfer condition (§8.3). Each case builds with the failing check
// switched off, which shows the check is what refuses.
func TestDisplacedTransfer(t *testing.T) {
	t.Parallel()
	budget := func() *proofbound.WorkBudget { return proofbound.NewWorkBudget(t.Context()) }
	mixedOf := func(cbp capblend.Payload) momentinput.Profile {
		m, err := capblend.MixedOffsetProfile(budget(), cbp)
		require.NoError(t, err)
		return m
	}
	t.Run("T1 refuses a displacement above a quarter of the contact floor", func(t *testing.T) {
		cbp := dispPayload(t)
		err := capblend.RequireTransfer(t.Context(), cbp, mixedOf(cbp), []float64{1e-6, 0})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Contains(t, err.Error(), "T1)")
		restore := capblend.DisableTransfer("T2")
		err = capblend.RequireTransfer(t.Context(), cbp, mixedOf(cbp), []float64{1e-6, 0})
		restore()
		require.ErrorIs(t, err, decaderr.ErrUnsupported, "T2 off: T1 alone still refuses")
		defer capblend.DisableTransfer("T1")()
		require.NoError(t, capblend.RequireTransfer(t.Context(), cbp, mixedOf(cbp), []float64{1e-6, 0}))
	})
	t.Run("T2 refuses a displaced outer loop certified within the floor of a contour", func(t *testing.T) {
		// F is 1e-9 of the 20 × 20 section's diagonal; the hole's contour (offset
		// 0.5) stands F/2 from the outer wall at u = 20.
		floor := 1e-9 * 20 * math.Sqrt2
		cbp := dispPayload(t)
		cbp.Profile.Holes[0] = holeLoop(20-4-0.5-floor/2, 8, 4)
		mixed := mixedOf(cbp)
		err := capblend.RequireTransfer(t.Context(), cbp, mixed, []float64{1e-12, 0})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Contains(t, err.Error(), "T2)")
		defer capblend.DisableTransfer("T2")()
		require.NoError(t, capblend.RequireTransfer(t.Context(), cbp, mixed, []float64{1e-12, 0}))
	})
	t.Run("T2 charges the contour's own displacement", func(t *testing.T) {
		floor := 1e-9 * 20 * math.Sqrt2
		cbp := dispPayload(t)
		// A setback with a conversion rounding of 5e-9 displaces its contour by
		// about as much; the gap clears F by 3e-9 on the held numbers only.
		cbp.Start.DcDelta, cbp.End.DcDelta = 5e-9, 5e-9
		cbp.Profile.Holes[0] = holeLoop(20-4-0.5-floor-3e-9, 8, 4)
		mixed := mixedOf(cbp)
		err := capblend.RequireTransfer(t.Context(), cbp, mixed, []float64{1e-12, 0})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		defer capblend.DisableTransfer("ContourDelta")()
		require.NoError(t, capblend.RequireTransfer(t.Context(), cbp, mixed, []float64{1e-12, 0}))
	})
	t.Run("T3 refuses a thin displaced loop whose segments are all adjacent", func(t *testing.T) {
		const ld = 1e-8
		tri := sectionrecord.LoopRecord{Segments: []sectionrecord.CurveSegment{
			sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 0, V: 0}, End: sectionrecord.Point2{U: 100, V: 0}, TStart: 0, TEnd: 1},
			sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 100, V: 0}, End: sectionrecord.Point2{U: 50, V: 4 * ld}, TStart: 0, TEnd: 1},
			sectionrecord.LineSeg{Start: sectionrecord.Point2{U: 50, V: 4 * ld}, End: sectionrecord.Point2{U: 0, V: 0}, TStart: 0, TEnd: 1},
		}}
		cbp := capblend.Payload{Profile: momentinput.Profile{Outer: tri}, Frame: dispFrame(t), Z1: 10, Xform: r3.Identity()}
		err := capblend.RequireTransfer(t.Context(), cbp, cbp.Profile, []float64{ld})
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.Contains(t, err.Error(), "T3")
		defer capblend.DisableTransfer("T3")()
		require.NoError(t, capblend.RequireTransfer(t.Context(), cbp, cbp.Profile, []float64{ld}))
	})
	t.Run("a clear displaced loop passes", func(t *testing.T) {
		cbp := dispPayload(t)
		require.NoError(t, capblend.RequireTransfer(t.Context(), cbp, mixedOf(cbp), []float64{1e-12, 0}))
	})
}

func TestSameWalkSegments(t *testing.T) {
	t.Parallel()
	walk := func(segs ...int) survey2d.SideWalk { return survey2d.SideWalk{Segs: segs} }
	require.True(t, capblend.SameWalkSegments([]survey2d.SideWalk{walk(0, 1), walk(2)}, []survey2d.SideWalk{walk(0, 1), walk(2)}))
	require.False(t, capblend.SameWalkSegments([]survey2d.SideWalk{walk(0, 1), walk(2)}, []survey2d.SideWalk{walk(0), walk(1, 2)}))
	require.False(t, capblend.SameWalkSegments([]survey2d.SideWalk{walk(0)}, nil))
}

// TestDisplacedSpanTopTransferRefusal: a setback held with a conversion rounding
// whose top-of-span section fails §8.3's transfer condition (T2 here) is
// refused as undecidable, and the message names the transfer condition rather
// than the offset audit, which passed. The hole's contour stands 7e-9 beyond
// the contact floor from the outer wall at the held setback, so the held
// numbers build; the span's top moves it 5e-9 closer and its own
// displacement is charged.
func TestDisplacedSpanTopTransferRefusal(t *testing.T) {
	t.Parallel()
	floor := 1e-9 * 20 * math.Sqrt2
	build := func(dcDelta float64) error {
		set := capblend.Setback{Dc: 0.5, DcDelta: dcDelta, Ds: 0.5}
		pp := prismrecord.Payload{
			Profile: momentinput.Profile{Outer: squareLoop(20),
				Holes: []sectionrecord.LoopRecord{holeLoop(20-4-0.5-floor-7e-9, 8, 4)}},
			Frame: dispFrame(t), Z1: 10, Xform: r3.Identity(),
		}
		_, err := capblend.BuildCheckedDisplaced(t.Context(), dispEnv(), pp, set, set,
			map[int]bool{1: true}, map[int]bool{1: true}, []float64{1e-12, 0})
		return err
	}
	require.NoError(t, build(0), "the held setback alone builds")
	err := build(5e-9)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.Contains(t, err.Error(), "at the top of that span")
	require.Contains(t, err.Error(), "fails the transfer condition")
	require.Contains(t, err.Error(), "T2)")
	require.NotContains(t, err.Error(), "fails the offset audit")
}

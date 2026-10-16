package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func crossBoreCellGap() *big.Rat {
	return new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper),
		big.NewRat(2*crossbore.HalfStations, 1))
}

func requireCrossBoreBitsEqual(t *testing.T, a, b crossbore.CellResult) {
	t.Helper()
	require.Equal(t, math.Float64bits(a.Width), math.Float64bits(b.Width))
	require.Equal(t, math.Float64bits(a.SampleError), math.Float64bits(b.SampleError))
	require.Equal(t, math.Float64bits(a.Speed), math.Float64bits(b.Speed))
	require.Equal(t, math.Float64bits(a.BandLength), math.Float64bits(b.BandLength))
}

func crossBoreMeshHeld(mesh *crossbore.Mesh, j int) [3]r3.Vec {
	cell := mesh.CellHeld[j]
	return [3]r3.Vec{mesh.Verts[cell[0]], mesh.Verts[cell[1]], mesh.Verts[cell[2]]}
}

// crossBoreSourceBodies builds the target and tool prisms of the public
// five-by-one cross bore (R = 5, r = 1, H = 10, c = 5) with its target circle
// centred at (0, targetY) and its tool circle centred at Y = toolY.
func crossBoreSourceBodies(t *testing.T, targetY, toolY float64) (*Body, *Body) {
	t.Helper()
	const radius, bore, height, centerZ = 5.0, 1.0, 10.0, 5.0
	doc := New()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, targetY)
	s.Fix(center)
	s.CreateCircle(center, radius)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	target, err := Extrude(t.Context(), doc, s, s.Profiles()[0],
		Distance{D: units.Millimeters(height), Dir: Along})
	require.NoError(t, err)
	tw := sketch.NewWorld()
	ts, err := tw.CreateSketch(tw.YZ())
	require.NoError(t, err)
	toolCenter := ts.CreatePoint(toolY, centerZ)
	ts.Fix(toolCenter)
	ts.CreateCircle(toolCenter, bore)
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	tool, err := Extrude(t.Context(), doc, ts, ts.Profiles()[0], Symmetric{D: units.Millimeters(2*radius + 2)})
	require.NoError(t, err)
	return target, tool
}

func crossBoreCircleCenter(t *testing.T, body *Body) planePoint {
	t.Helper()
	pp, ok := body.n.Payload.(prismPayload)
	require.True(t, ok)
	circle, ok := pp.Profile.Outer.Segments[0].(circleSeg)
	require.True(t, ok)
	return circle.Center
}

// TestCrossBoreCellsRefuseNearSymmetricPose breaks the pose's mirror
// symmetry by one rounding step and requires the cell proof to refuse
// before it proves any cell. The cell formulas read only R, r and d, so
// without the pose check every one of these records would prove, on the
// symmetric record's own mesh.
func TestCrossBoreCellsRefuseNearSymmetricPose(t *testing.T) {
	const d = 0.5
	ulp := math.Nextafter(0, 1)
	target, tool := crossBoreSourceBodies(t, 0, 0)
	exact := recognizeCrossCylinderBore(target, tool)
	require.NotNil(t, exact)
	require.True(t, crossBoreMirrorPose(exact))
	mesh, err := buildCrossBoreChamferMesh(t.Context(), exact, d)
	require.NoError(t, err)

	_, offsetTool := crossBoreSourceBodies(t, 0, ulp)
	require.Equal(t, ulp, crossBoreCircleCenter(t, offsetTool).U)
	offsetTarget, _ := crossBoreSourceBodies(t, ulp, 0)
	require.Equal(t, ulp, crossBoreCircleCenter(t, offsetTarget).V)
	// A full turn about the bore axis is the identity in the reals; in
	// floating point it leaves sin(2π) ≠ 0 off the diagonal.
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(2*math.Pi))
	require.NoError(t, err)
	require.NotEqual(t, r3.Identity(), turn)
	_, turnSource := crossBoreSourceBodies(t, 0, 0)
	turnedTool, err := Transform(t.Context(), turnSource, turn)
	require.NoError(t, err)

	for _, tc := range []struct {
		name string
		rec  crossCylinderBoreRecord
	}{
		{"tool centre one ulp off the Y mirror", func() crossCylinderBoreRecord {
			rec := *exact
			rec.Tool = bodyNode(offsetTool)
			return rec
		}()},
		{"target centre one ulp off the Y mirror", func() crossCylinderBoreRecord {
			rec := *exact
			rec.Target = bodyNode(offsetTarget)
			return rec
		}()},
		{"tool turned by a rounded full turn", func() crossCylinderBoreRecord {
			rec := *exact
			rec.Tool = bodyNode(turnedTool)
			return rec
		}()},
		{"bore radius one ulp off its source", func() crossCylinderBoreRecord {
			rec := *exact
			rec.Bore = math.Nextafter(rec.Bore, 2)
			return rec
		}()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := proveCrossBoreCells(t.Context(), &tc.rec, d, mesh)
			require.ErrorIs(t, err, ErrUnsupported)
			require.ErrorContains(t, err, "not the exact mirror-symmetric pose")
		})
	}
}

// TestCrossBoreMirroredRouteMatchesDirect proves every cell of the public
// five-by-one record directly, with no mirror map, and compares that route
// with the mirrored one on the same held mesh: every cell's verdict, the
// readings that must be bit-identical, and the readings that may only
// tighten. It then moves one held vertex of a mapped cell and requires the
// mirrored route to charge it.
func TestCrossBoreMirroredRouteMatchesDirect(t *testing.T) {
	const n = 2 * crossbore.HalfStations
	const setback = 0.5
	rec := crossBoreExactRecord(t, 5, 1, 10, 5)
	mesh, err := buildCrossBoreChamferMesh(t.Context(), rec, setback)
	require.NoError(t, err)
	gap := crossBoreCellGap()

	direct := make([]crossbore.CellExact, n)
	_, err = orderedwork.Each(n, orderedwork.EachWorkers(n, crossbore.CellParallelMin), crossbore.CellGrain, nil,
		func(j int) error {
			var err error
			direct[j], err = crossbore.CellExactProof(rec, setback, j, gap)
			return err
		})
	require.NoError(t, err, "every cell passes directly")
	var want crossbore.CellResult
	var directMax crossbore.CellResult
	for j := range n {
		k, flipY, flipZ := crossbore.CellMirror(j)
		held := crossBoreMeshHeld(mesh, j)
		mapped := crossbore.CellSampleError(&held, rec.CenterZ, &direct[k].Coords, flipY, flipZ)
		own := crossbore.CellSampleError(&held, rec.CenterZ, &direct[j].Coords, false, false)
		require.False(t, proofbound.IsNonFinite(mapped), "cell %d", j)
		want.SampleError = max(want.SampleError, mapped)
		directMax.SampleError = max(directMax.SampleError, own)
		directMax.Width = max(directMax.Width, direct[j].Width)
		directMax.Speed = max(directMax.Speed, direct[j].Speed)
		directMax.BandLength = max(directMax.BandLength, direct[j].BandLength)
		if j <= n/4 {
			want.Width = max(want.Width, direct[j].Width)
			want.Speed = max(want.Speed, direct[j].Speed)
			want.BandLength = max(want.BandLength, direct[j].BandLength)
		}
	}
	got, err := proveCrossBoreCells(t.Context(), rec, setback, mesh)
	require.NoError(t, err, "every base cell passes")
	requireCrossBoreBitsEqual(t, want, got)
	require.Equal(t, math.Float64bits(directMax.Speed), math.Float64bits(got.Speed))
	require.Equal(t, math.Float64bits(directMax.BandLength), math.Float64bits(got.BandLength))
	require.LessOrEqual(t, got.Width, directMax.Width)
	require.LessOrEqual(t, got.SampleError, directMax.SampleError)
	require.Less(t, got.SampleError, 0.01, "the held vertices lie within the cell enclosures' scale")

	// A held mesh that is not symmetric is still caught: cell 3000 maps onto
	// base 952 with Y and Z−c negated, and the route compares cell 3000's own
	// outer foot with that mirrored enclosure.
	moved := append([]r3.Vec(nil), mesh.Verts...)
	moved[mesh.CellHeld[3000][0]].Y += 0.05
	charged, err := crossbore.MirroredCellMaxima(t.Context(), 4,
		func(k int) (crossbore.CellExact, error) { return direct[k], nil },
		func(j int, base *crossbore.CellExact, flipY, flipZ bool) float64 {
			cell := mesh.CellHeld[j]
			held := [3]r3.Vec{moved[cell[0]], moved[cell[1]], moved[cell[2]]}
			return crossbore.CellSampleError(&held, rec.CenterZ, &base.Coords, flipY, flipZ)
		})
	require.NoError(t, err)
	require.Greater(t, charged.SampleError, proofbound.Radius3D(0.04))
}

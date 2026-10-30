package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolvebore"
	"github.com/lestrrat-3d/decad/internal/revolveboretess"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRevolveBoreBoundaryBothSweepDirections(t *testing.T) {
	for _, tc := range []struct {
		name              string
		against, opposite bool
	}{{name: "along"}, {name: "against", against: true}, {name: "opposite axis", opposite: true}} {
		t.Run(tc.name, func(t *testing.T) {
			doc, w := New(), sketch.NewWorld()
			s, err := w.CreateSketch(w.XZ())
			require.NoError(t, err)
			points := [][2]float64{{0, 0}, {10, 0}, {8, 32}, {0, 32}}
			vertices := make([]*sketch.Point, len(points))
			for i, p := range points {
				vertices[i] = s.CreatePoint(p[0], p[1])
			}
			s.Fix(vertices[0])
			for i := range vertices {
				s.CreateLine(vertices[i], vertices[(i+1)%len(vertices)])
			}
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			host, err := Revolve(t.Context(), doc, s, s.Profiles()[0], ConstructionAxis{Dir: r3.Vec{Z: 1}}, FullRevolution{})
			require.NoError(t, err)
			plane := w.XY()
			if tc.against {
				plane, err = w.CreateOffsetPlane(plane, 32)
				require.NoError(t, err)
			}
			if tc.opposite {
				frame, err := r3.NewFrame(r3.Vec{Z: 32}, r3.Vec{X: 1}, r3.Vec{Y: -1})
				require.NoError(t, err)
				plane, err = w.CreatePlaneFromFrame(frame)
				require.NoError(t, err)
			}
			ts, err := w.CreateSketch(plane)
			require.NoError(t, err)
			h := math.Sqrt(4.1*4.1 - 3.1*3.1)
			a, z := ts.CreatePoint(3.1, h), ts.CreatePoint(3.1, -h)
			arc := ts.CreateArc(ts.Origin(), a, z)
			line := ts.CreateLine(z, a)
			ts.AddConstraint(sketch.NewRadius(arc, 4.1), sketch.NewHorizontalDistance(ts.Origin(), a, 3.1), sketch.NewVertical(line))
			_, err = ts.Solve(t.Context())
			require.NoError(t, err)
			direction := Along
			if tc.against {
				direction = Against
			}
			tool, err := Extrude(t.Context(), doc, ts, ts.Profiles()[0], Distance{D: units.Millimeters(32), Dir: direction})
			require.NoError(t, err)
			candidate, ok, err := revolvebore.Prepare(t.Context(), &host.n, &tool.n)
			require.NoError(t, err)
			require.True(t, ok)
			env := envOf(&doc.n, host.n.Origin.Producer)
			body, err := revolveplan.Build(t.Context(), env, candidate.Revolve, freeform.NewFreeformWork())
			require.NoError(t, err)
			_, err = revolvebore.AddBoundary(t.Context(), env, body, candidate)
			require.NoError(t, err)
			require.Len(t, body.Lumps, 1)
			require.Len(t, body.Lumps[0].Shells, 1)
			require.False(t, body.Lumps[0].Shells[0].Open)
			for _, edge := range topobuild.Edges(body) {
				require.Len(t, edge.Faces, 2)
			}
			toolFaces := 0
			for _, face := range topobuild.Faces(body) {
				for _, origin := range face.Origins {
					if origin.Producer == tool.n.Origin.Producer {
						toolFaces++
						break
					}
				}
			}
			require.Equal(t, 2, toolFaces)
			builtTool, err := prismrecord.Build(t.Context(), env, candidate.Tool, freeform.NewFreeformWork())
			require.NoError(t, err)
			require.NoError(t, revolvebore.ApplyMeasurements(body, builtTool, candidate.Meridian))
			areaD := math.Pi*4.1*4.1 - (4.1*4.1*math.Acos(3.1/4.1) - 3.1*math.Sqrt(4.1*4.1-3.1*3.1))
			capCount := 0
			for _, face := range topobuild.Faces(body) {
				if _, ok := face.Surface.(surfacegeom.Plane); !ok || len(face.Loops) != 2 {
					continue
				}
				capCount++
				lowWant, highWant := math.Pi*100-areaD, math.Pi*64-areaD
				difference := math.Min(math.Abs(face.Area-lowWant), math.Abs(face.Area-highWant))
				require.LessOrEqual(t, difference, face.AreaBound+1e-10)
			}
			require.Equal(t, 2, capCount)
			want := math.Pi*32/3*(10*10+10*8+8*8) - 32*areaD
			require.LessOrEqual(t, math.Abs(body.Volume.Value.Base()-want), body.Volume.Bound.Base())
			payload := revolvebore.Payload{Candidate: candidate}
			built, err := revolvebore.Build(t.Context(), env, payload)
			require.NoError(t, err)
			sources, err := revolveboretess.BuildSources(t.Context(), built, payload, 0.1)
			require.NoError(t, err)
			require.True(t, sources.Host.SymDiffOK)
			require.True(t, sources.Tool.SymDiffOK)
			mesh, err := revolveboretess.Assemble(t.Context(), built, payload, sources)
			require.NoError(t, err)
			require.True(t, mesh.SymDiffOK)
			require.InDelta(t, body.Volume.Value.Base(), built.Volume.Value.Base(), 0)
			move, err := r3.Translation(r3.Vec{X: 2, Z: 3})
			require.NoError(t, err)
			placed, err := payload.Placed(t.Context(), env, move)
			require.NoError(t, err)
			placedToolFaces := 0
			for _, face := range topobuild.Faces(placed) {
				for _, origin := range face.Origins {
					if origin.Producer == tool.n.Origin.Producer {
						placedToolFaces++
						break
					}
				}
			}
			require.Equal(t, toolFaces, placedToolFaces)
			require.InDelta(t, built.Volume.Value.Base(), placed.Volume.Value.Base(), placed.Volume.Bound.Base())
			require.InDelta(t, built.Centroid.Value.X+2, placed.Centroid.Value.X, placed.Centroid.Bound.Base())
			require.InDelta(t, built.Centroid.Value.Z+3, placed.Centroid.Value.Z, placed.Centroid.Bound.Base())
			turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{X: 1, Y: 1}, units.Radians(0.37))
			require.NoError(t, err)
			rotated, err := payload.Placed(t.Context(), env, turn)
			require.NoError(t, err)
			rotatedPayload := rotated.Payload.(revolvebore.Payload)
			rotatedSources, err := revolveboretess.BuildSources(t.Context(), rotated, rotatedPayload, 0.1)
			require.NoError(t, err)
			_, err = revolveboretess.Assemble(t.Context(), rotated, rotatedPayload, rotatedSources)
			require.NoError(t, err)
		})
	}
	for _, tc := range []struct {
		name string
		area float64
	}{{"D outline", math.Pi*4.1*4.1 - (4.1*4.1*math.Acos(3.1/4.1) - 3.1*math.Sqrt(4.1*4.1-3.1*3.1))},
		{"rectangular outline", 24}} {
		t.Run("repeated placement/"+tc.name, func(t *testing.T) {
			requireRevolveBoreRepeatedPlacement(t, tc.name, tc.area)
		})
	}
}

func requireRevolveBoreRepeatedPlacement(t *testing.T, outline string, toolArea float64) {
	t.Helper()
	doc, w := New(), sketch.NewWorld()
	hs, err := w.CreateSketch(w.XZ())
	require.NoError(t, err)
	points := [][2]float64{{0, 0}, {10, 0}, {8, 32}, {0, 32}}
	vertices := make([]*sketch.Point, len(points))
	for i, point := range points {
		vertices[i] = hs.CreatePoint(point[0], point[1])
	}
	hs.Fix(vertices[0])
	for i := range vertices {
		hs.CreateLine(vertices[i], vertices[(i+1)%len(vertices)])
	}
	_, err = hs.Solve(t.Context())
	require.NoError(t, err)
	host, err := Revolve(t.Context(), doc, hs, hs.Profiles()[0], ConstructionAxis{Dir: r3.Vec{Z: 1}}, FullRevolution{})
	require.NoError(t, err)
	ts, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	switch outline {
	case "D outline":
		const radius, flat = 4.1, 3.1
		h := math.Sqrt(radius*radius - flat*flat)
		upper, lower := ts.CreatePoint(flat, h), ts.CreatePoint(flat, -h)
		arc := ts.CreateArc(ts.Origin(), upper, lower)
		line := ts.CreateLine(lower, upper)
		ts.AddConstraint(sketch.NewRadius(arc, radius), sketch.NewHorizontalDistance(ts.Origin(), upper, flat),
			sketch.NewVertical(line))
	case "rectangular outline":
		rectangle := ts.CreateRectangle(-3, -2, 3, 2)
		ts.Fix(rectangle.A)
	default:
		t.Fatalf("unknown bore outline %q", outline)
	}
	_, err = ts.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, ts.Profiles(), 1)
	tool, err := Extrude(t.Context(), doc, ts, ts.Profiles()[0], Distance{D: units.Millimeters(32), Dir: Along})
	require.NoError(t, err)
	result, err := Cut(t.Context(), host, tool)
	require.NoError(t, err)
	_, ok := result.n.Payload.(revolvebore.Payload)
	require.True(t, ok, "the fixture exercises the native axial Revolve bore record")
	wantVolume := math.Pi*32/3*(10*10+10*8+8*8) - toolArea*32
	requireRevolveBoreVolume(t, result, wantVolume)

	placed, err := Transform(t.Context(), result, r3.Identity())
	require.NoError(t, err, "identity placement")
	requireRevolveBoreVolume(t, placed, wantVolume)
	angle := math.Remainder(7*math.Pi/3, 2*math.Pi)
	c, s := math.Cos(angle), math.Sin(angle)
	frame, err := r3.NewFrame(r3.Vec{X: -140, Y: 28, Z: 690}, r3.Vec{X: c, Z: s}, r3.Vec{X: -s, Z: c})
	require.NoError(t, err)
	turn, err := r3.FromFrame(frame)
	require.NoError(t, err)
	placed, err = Transform(t.Context(), placed, turn)
	require.NoError(t, err, "nonzero placement")
	requireRevolveBoreVolume(t, placed, wantVolume)
	placed, err = Transform(t.Context(), placed, r3.Identity())
	require.NoError(t, err, "repeated identity placement")
	requireRevolveBoreVolume(t, placed, wantVolume)
	shift, err := r3.Translation(r3.Vec{X: 13, Y: -7, Z: 5})
	require.NoError(t, err)
	placed, err = Transform(t.Context(), placed, shift)
	require.NoError(t, err, "repeated nonzero placement")
	requireRevolveBoreVolume(t, placed, wantVolume)
}

func requireRevolveBoreVolume(t *testing.T, body *Body, want float64) {
	t.Helper()
	volume, err := body.Volume()
	require.NoError(t, err)
	require.LessOrEqual(t, math.Abs(volume.Value.Base()-want), volume.Bound.Base()+1e-10)
}

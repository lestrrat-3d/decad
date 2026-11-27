package bodywire_test

import (
	"bytes"
	"cmp"
	"compress/gzip"
	"encoding/binary"
	"io"
	"math"
	"slices"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/bodywire"
	"github.com/lestrrat-3d/decad/decadtest"
	"github.com/lestrrat-3d/decad/export"
	"github.com/lestrrat-3d/decad/internal/stepread"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/step"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestCylinderRoundTrip(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(decad.New(), s, s.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	var frame bytes.Buffer
	require.NoError(t, bodywire.Write(t.Context(), &frame, body, units.Millimeters(0.1)))
	require.Equal(t, "DCS1", string(frame.Bytes()[:4]))
	received, err := bodywire.Read(t.Context(), &frame)
	require.NoError(t, err)
	require.False(t, received.IsSheet())
	_, hasBound := received.SourceBoundaryBound()
	require.False(t, hasBound)
	coarse, err := received.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)
	fine, err := received.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, coarse.Report().Closed)
	require.True(t, fine.Report().Closed)
	require.Greater(t, len(fine.Triangles()), len(coarse.Triangles()))
	require.InDelta(t, 5, fine.Bounds().Max.X, 0.05)
	require.InDelta(t, 8, fine.Bounds().Max.Z, 1e-12)
}

func TestCircularSheetRoundTrip(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.PatchProfile(decad.New(), s, s.Profiles()[0]).Do(t.Context())
	require.NoError(t, err)
	tol := units.Millimeters(0.1)
	want, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	var frame bytes.Buffer
	require.NoError(t, bodywire.Write(t.Context(), &frame, body, tol))
	require.Equal(t, "DCS3", string(frame.Bytes()[:4]))
	original := bytes.Clone(frame.Bytes())
	zr, err := gzip.NewReader(bytes.NewReader(original[24:]))
	require.NoError(t, err)
	payload, err := io.ReadAll(zr)
	require.NoError(t, err)
	require.NoError(t, zr.Close())
	for name, mutate := range map[string]func([]byte){
		"oversized vertex count": func(p []byte) { binary.LittleEndian.PutUint64(p, math.MaxUint64) },
		"nonfinite coordinate":   func(p []byte) { binary.LittleEndian.PutUint64(p[16:], math.Float64bits(math.NaN())) },
		"invalid index": func(p []byte) {
			nv := binary.LittleEndian.Uint64(p)
			binary.LittleEndian.PutUint32(p[16+24*nv:], math.MaxUint32)
		},
	} {
		t.Run(name, func(t *testing.T) {
			bad := bytes.Clone(payload)
			mutate(bad)
			var compressed bytes.Buffer
			zw := gzip.NewWriter(&compressed)
			_, err := zw.Write(bad)
			require.NoError(t, err)
			require.NoError(t, zw.Close())
			header := bytes.Clone(original[:24])
			binary.LittleEndian.PutUint32(header[4:], uint32(compressed.Len()))
			_, err = bodywire.Read(t.Context(), io.MultiReader(bytes.NewReader(header), &compressed))
			require.ErrorIs(t, err, decad.ErrMalformedMesh)
		})
	}
	received, err := bodywire.Read(t.Context(), &frame)
	require.NoError(t, err)
	require.True(t, received.IsSheet())
	bound, ok := received.SourceBoundaryBound()
	require.True(t, ok)
	require.Equal(t, want.Bound(), bound)
	if bound.Base() > 0 {
		_, err = received.Tessellate(t.Context(), units.Millimeters(bound.Base()/2))
		require.ErrorIs(t, err, decad.ErrUnsupported)
	}
	mesh, err := received.Tessellate(t.Context(), units.Millimeters(math.Max(tol.Base(), bound.Base())))
	require.NoError(t, err)
	require.False(t, mesh.Report().Closed)
	require.Equal(t, want.Vertices(), mesh.Vertices())
	require.Equal(t, want.Triangles(), mesh.Triangles())
}

func TestFacetedUnionRoundTrip(t *testing.T) {
	doc := decad.New()
	base := decadtest.NewBlock(t, doc, -5, -5, 5, 5, units.Millimeters(10))
	upper := decadtest.NewBlock(t, doc, -2, -2, 2, 2, units.Millimeters(4))
	move, err := r3.Translation(r3.NewVec(0, 0, 8))
	require.NoError(t, err)
	upper, err = decad.Transform(upper, move).Do(t.Context())
	require.NoError(t, err)
	body, err := decad.Union(base, upper).Do(t.Context())
	require.NoError(t, err)
	require.Equal(t, "decad.facetedPayload", decadtest.PayloadClass(body))
	tol := units.Millimeters(0.1)
	want, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	var frame bytes.Buffer
	require.NoError(t, bodywire.Write(t.Context(), &frame, body, tol))
	require.Equal(t, "DCS2", string(frame.Bytes()[:4]))
	invalid := bytes.Clone(frame.Bytes())
	binary.LittleEndian.PutUint64(invalid[16:], math.Float64bits(math.NaN()))
	_, err = bodywire.Read(t.Context(), bytes.NewReader(invalid))
	require.ErrorIs(t, err, decad.ErrMalformedMesh)
	received, err := bodywire.Read(t.Context(), &frame)
	require.NoError(t, err)
	bound, ok := received.SourceBoundaryBound()
	require.True(t, ok)
	require.Equal(t, want.Bound(), bound)
	require.Zero(t, bound.Base())
	mesh, err := received.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.True(t, mesh.Report().Closed)
	requireSameFacets(t, want.Vertices(), want.Triangles(), mesh.Vertices(), mesh.Triangles())
	volume, err := mesh.Volume()
	require.NoError(t, err)
	require.InDelta(t, 1032, volume.Value.Base(), 1e-9)

	move, err = r3.Translation(r3.NewVec(0.1, 0, 0))
	require.NoError(t, err)
	placed, err := decad.Transform(body, move).Do(t.Context())
	require.NoError(t, err)
	want, err = placed.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.Positive(t, want.Bound().Base())
	frame.Reset()
	require.NoError(t, bodywire.Write(t.Context(), &frame, placed, tol))
	received, err = bodywire.Read(t.Context(), &frame)
	require.NoError(t, err)
	bound, ok = received.SourceBoundaryBound()
	require.True(t, ok)
	require.Equal(t, want.Bound(), bound)
	mesh, err = received.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	requireSameFacets(t, want.Vertices(), want.Triangles(), mesh.Vertices(), mesh.Triangles())
	_, err = received.Tessellate(t.Context(), units.Millimeters(bound.Base()/2))
	require.ErrorIs(t, err, decad.ErrUnsupported)
}

func TestAnalyticConeFacetedFallbackRoundTrip(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	origin := s.CreatePoint(0, 0)
	s.Fix(origin)
	apex := s.CreatePoint(10, 0)
	top := s.CreatePoint(0, 5)
	s.CreateLine(origin, apex)
	s.CreateLine(apex, top)
	s.CreateLine(top, origin)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 1, V: 0}}
	body, err := decad.Revolve(decad.New(), s, s.Profiles()[0], axis, decad.FullRevolution{}).Do(t.Context())
	require.NoError(t, err)
	tol := units.Millimeters(0.5)
	want, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	require.Positive(t, want.Bound().Base())
	var frame bytes.Buffer
	require.NoError(t, bodywire.Write(t.Context(), &frame, body, tol))
	require.Equal(t, "DCS2", string(frame.Bytes()[:4]))
	received, err := bodywire.Read(t.Context(), &frame)
	require.NoError(t, err)
	bound, ok := received.SourceBoundaryBound()
	require.True(t, ok)
	require.Equal(t, want.Bound(), bound)
	mesh, err := received.Tessellate(t.Context(), tol)
	require.NoError(t, err)
	require.True(t, mesh.Report().Closed)
	requireSameFacets(t, want.Vertices(), want.Triangles(), mesh.Vertices(), mesh.Triangles())
}

func TestFilletedCubeReceiverFallback(t *testing.T) {
	body := decadtest.NewBlock(t, decad.New(), 0, 0, 20, 20, units.Millimeters(20))
	body, err := decad.Fillet(body, decad.Edges(), units.Millimeters(2)).Do(t.Context())
	require.NoError(t, err)
	tol := units.Millimeters(0.5)
	file, err := export.NewSTEPFile(t.Context(), body, tol, step.Header{
		Description: []string{"body"}, Name: "body.step", Timestamp: time.Unix(0, 0).UTC(),
		Authors: []string{"decad"}, Organizations: []string{"decad"},
	})
	require.NoError(t, err)
	analytic := false
	for _, entity := range file.Entities {
		if entity.Name == "SPHERICAL_SURFACE" {
			analytic = true
			break
		}
	}
	require.True(t, analytic, "the STEP exporter must produce analytic spherical faces")
	want, err := body.Tessellate(t.Context(), tol, decad.WithVerification(decad.VerifyBoundary))
	require.NoError(t, err)
	var frame bytes.Buffer
	require.NoError(t, bodywire.Write(t.Context(), &frame, body, tol))
	require.Equal(t, "DCS2", string(frame.Bytes()[:4]))
	received, err := bodywire.Read(t.Context(), &frame)
	require.NoError(t, err)
	bound, ok := received.SourceBoundaryBound()
	require.True(t, ok)
	require.Equal(t, want.Bound(), bound)
	mesh, err := received.Tessellate(t.Context(), units.Millimeters(math.Max(tol.Base(), bound.Base())))
	require.NoError(t, err)
	require.True(t, mesh.Report().Closed)
	requireSameFacets(t, want.Vertices(), want.Triangles(), mesh.Vertices(), mesh.Triangles())
}

func requireSameFacets(t *testing.T, source []r3.Vec, sourceTris [][3]int, got []r3.Vec, gotTris [][3]int) {
	t.Helper()
	key := func(vertices []r3.Vec, tri [3]int) [3]r3.Vec {
		points := [3]r3.Vec{vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]}
		slices.SortFunc(points[:], func(a, b r3.Vec) int {
			if c := cmp.Compare(a.X, b.X); c != 0 {
				return c
			}
			if c := cmp.Compare(a.Y, b.Y); c != 0 {
				return c
			}
			return cmp.Compare(a.Z, b.Z)
		})
		return points
	}
	if len(sourceTris) != len(gotTris) {
		t.Fatalf("facet count: source %d, received %d", len(sourceTris), len(gotTris))
	}
	remaining := make(map[[3]r3.Vec]int, len(gotTris))
	for _, tri := range gotTris {
		remaining[key(got, tri)]++
	}
	for i, tri := range sourceTris {
		k := key(source, tri)
		if remaining[k] == 0 {
			t.Fatalf("source facet %d has no received match", i)
		}
		remaining[k]--
	}
}

func TestRotatedCylinderRoundTrip(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 5)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := decad.Extrude(decad.New(), s, s.Profiles()[0], decad.Distance{D: units.Millimeters(8), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	xf, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	body, err = decad.Transform(body, xf).Do(t.Context())
	require.NoError(t, err)
	var frame bytes.Buffer
	require.NoError(t, bodywire.Write(t.Context(), &frame, body, units.Millimeters(0.1)))
	encoded := frame.Bytes()
	require.Equal(t, uint32(len(encoded)-16), binary.LittleEndian.Uint32(encoded[4:]))
	compressed, err := gzip.NewReader(bytes.NewReader(encoded[16:]))
	require.NoError(t, err)
	stepData, err := io.ReadAll(compressed)
	require.NoError(t, err)
	require.NoError(t, compressed.Close())
	model, err := stepread.Parse(stepData)
	require.NoError(t, err)
	counts := make(map[string]int)
	uses := make(map[int][]stepread.Enum)
	var wall stepread.Entity
	for _, entity := range model.Entities {
		counts[entity.Name]++
		if entity.Name == "ORIENTED_EDGE" {
			edge := int(entity.Params[3].(stepread.Ref))
			uses[edge] = append(uses[edge], entity.Params[4].(stepread.Enum))
		}
		if entity.Name == "ADVANCED_FACE" {
			surface := model.Entities[int(entity.Params[2].(stepread.Ref))]
			if surface.Name == "CYLINDRICAL_SURFACE" {
				wall = entity
			}
		}
	}
	require.Equal(t, 1, counts["CYLINDRICAL_SURFACE"])
	require.Equal(t, 2, counts["EDGE_CURVE"])
	require.Zero(t, counts["LINE"])
	for _, senses := range uses {
		require.ElementsMatch(t, []stepread.Enum{"T", "F"}, senses)
	}
	bounds := wall.Params[1].(stepread.List)
	require.Len(t, bounds, 2)
	for _, ref := range bounds {
		bound := model.Entities[int(ref.(stepread.Ref))]
		require.Equal(t, "FACE_BOUND", bound.Name)
	}
	received, err := bodywire.Read(t.Context(), &frame)
	require.NoError(t, err)
	mesh, err := received.Tessellate(t.Context(), units.Millimeters(0.1))
	require.NoError(t, err)
	require.True(t, mesh.Report().Closed)
	volume, err := mesh.Volume()
	require.NoError(t, err)
	steps := math.Ceil(math.Pi / math.Acos(1-0.1/5))
	expected := 0.5 * steps * 25 * math.Sin(2*math.Pi/steps) * 8
	require.InDelta(t, expected, volume.Value.Base(), 1e-9)
}

func TestRotatedCircularHoleRoundTrip(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 8, 6)
	s.Fix(rect.A)
	center := s.CreatePoint(3, 3)
	s.Fix(center)
	s.CreateCircle(center, 1)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var profile *sketch.Profile
	for _, candidate := range s.Profiles() {
		if len(candidate.Holes) == 1 {
			profile = candidate
		}
	}
	require.NotNil(t, profile)
	body, err := decad.Extrude(decad.New(), s, profile, decad.Distance{D: units.Millimeters(4), Dir: decad.Along}).Do(t.Context())
	require.NoError(t, err)
	xf, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	body, err = decad.Transform(body, xf).Do(t.Context())
	require.NoError(t, err)
	var frame bytes.Buffer
	require.NoError(t, bodywire.Write(t.Context(), &frame, body, units.Millimeters(0.1)))
	received, err := bodywire.Read(t.Context(), &frame)
	require.NoError(t, err)
	mesh, err := received.Tessellate(t.Context(), units.Millimeters(0.05))
	require.NoError(t, err)
	require.True(t, mesh.Report().Closed)
	volume, err := mesh.Volume()
	require.NoError(t, err)
	steps := math.Ceil(math.Pi / math.Acos(1-0.05))
	expected := (48 - 0.5*steps*math.Sin(2*math.Pi/steps)) * 4
	require.InDelta(t, expected, volume.Value.Base(), 1e-9)
}

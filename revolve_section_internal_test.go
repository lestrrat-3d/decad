package decad

import (
	"fmt"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file covers docs/surface-intersection-design.md §7.2: a solid revolve
// whose recorded meridian sits within a proven sectionDelta of the meridian it
// denotes builds, and every reading it publishes encloses the reading the
// DENOTED meridian gives. Each fixture records the denoted meridian moved —
// scaled by 1 + revolveSectionScale about the plane origin, shifted along the
// axis, or with a whole circle's radius grown — and states sectionDelta over
// that move. The denoted reading is the same revolve built from the unmoved
// record, whose own bounds are added to the displaced body's.
//
// Legs shown to fail before these fixtures were accepted (each removed, the
// fixture watched go red, then restored): the band's charge on ∫ρ dA sends
// every volume red; its charge on ∫zρ dA sends the axis-shifted rectangle's
// centroid red; the cap's band sends the half-turn caps' areas red; the
// whole-section endpoint charge sends the rectangles' wall areas red; the
// widened recorded point sends every vertex red; the arc's moment charge sends
// the half sphere's and the torus's wall areas red; the arc's own length
// charge sends the grown circle's cap edges red; the widened denotation sends
// the half sphere's NormalAt red; and in the mesh, the face bounds' share, the
// area slack's share and the occupied-volume share each send the grown torus
// red. Two legs are not separately observable. The charge on ∫ρ² dA is
// covered on every fixture tried by the volume's own charge, carried through
// the in-plane quotient. The circular denotation's radius widening is covered
// by its centre's, which alone decides the normal's direction at p.

// revolveSectionScale is the factor every fixture's recorded meridian is
// scaled by.
const revolveSectionScale = 1e-4

func sectionSketchBody(t *testing.T, build func(*sketch.Sketch), extent AngularExtent) *Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	build(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	body, err := New().Revolve(s, s.Profiles()[0], revolveAxisU, extent)
	require.NoError(t, err)
	return body
}

// movedProfile scales every recorded coordinate of p by k about the plane
// origin, shifts it by shift along u, the axis, and grows every whole
// circle's radius by grow, and returns the largest coordinate magnitude it
// scaled.
func movedProfile(t *testing.T, p profileRecord, k, shift, grow float64) (profileRecord, float64) {
	t.Helper()
	most := 0.0
	pt := func(q Point2) Point2 {
		most = math.Max(most, math.Hypot(q.U, q.V))
		return Point2{U: q.U*k + shift, V: q.V * k}
	}
	loop := func(l loopRecord) loopRecord {
		out := loopRecord{Segments: make([]curveSegment, len(l.Segments))}
		for i, seg := range l.Segments {
			switch s := seg.(type) {
			case lineSeg:
				s.Start, s.End = pt(s.Start), pt(s.End)
				out.Segments[i] = s
			case arcSeg:
				s.Center, s.Start, s.End = pt(s.Center), pt(s.Start), pt(s.End)
				out.Segments[i] = s
			case circleSeg:
				r, err := s.Radius.In(units.Millimeter)
				require.NoError(t, err)
				most = math.Max(most, math.Hypot(s.Center.U, s.Center.V)+r)
				s.Center = pt(s.Center)
				s.Radius = units.Millimeters(r*k + grow)
				out.Segments[i] = s
			default:
				t.Fatalf("movedProfile: unexpected segment %T", seg)
			}
		}
		return out
	}
	out := profileRecord{Outer: loop(p.Outer)}
	for _, h := range p.Holes {
		out.Holes = append(out.Holes, loop(h))
	}
	return out, most
}

// displacedRevolve rebuilds ref's payload over its meridian scaled by
// 1 + revolveSectionScale — or, where shift or grow is nonzero, shifted along
// the axis by shift and with every whole circle's radius grown by grow —
// carrying the displacement as a whole-section sectionDelta, and evaluates it.
func displacedRevolve(t *testing.T, ref *Body, shift, grow float64) (*Body, float64) {
	t.Helper()
	rp := ref.payload.(revolvePayload)
	k := 1 + revolveSectionScale
	if shift != 0 || grow != 0 {
		k = 1
	}
	moved, most := movedProfile(t, rp.profile, k, shift, grow)
	ax, err := revolveBlendAxis(t.Context(), rp, moved, freeform.NewFreeformWork())
	require.NoError(t, err)
	rp.profile, rp.ax, rp.radialProof = moved, ax, ax.RadialProof
	rp.sectionDelta = 2 * (revolveSectionScale*most + math.Abs(shift))
	if shift != 0 {
		rp.sectionDelta = 2 * math.Abs(shift)
	}
	if grow != 0 {
		// Every point of a grown circle moves radially by grow, its centre by
		// nothing; the factor covers the radius's own rounding and nothing
		// more, so the charge is held to the figure the growth states.
		rp.sectionDelta = math.Abs(grow) * (1 + 1e-9)
	}
	rp.sectionWhole = true
	body, err := evalRevolve(ref.doc, ref.doc.nextProducerID(), rp)
	require.NoError(t, err)
	return body, rp.sectionDelta
}

func requireEncloses(t *testing.T, what string, got, want Measurement) {
	t.Helper()
	gap := math.Abs(got.Value.Base() - want.Value.Base())
	require.LessOrEqual(t, gap, got.Bound.Base()+want.Bound.Base(),
		"%s: displaced %v ± %v does not enclose denoted %v ± %v", what, got.Value, got.Bound, want.Value, want.Bound)
}

func requireVecEncloses(t *testing.T, what string, got, want r3.Vec, gotBound, wantBound float64) {
	t.Helper()
	gap := got.Sub(want).Len()
	require.LessOrEqual(t, gap, gotBound+wantBound, "%s: displaced %v ± %v does not enclose denoted %v ± %v", what, got, gotBound, want, wantBound)
}

func TestRevolveSectionDisplacementEnclosesDenoted(t *testing.T) {
	t.Parallel()
	halfTurn := AngleExtent{A: units.Degrees(180), Dir: Along}
	for _, tc := range []struct {
		name   string
		build  func(*sketch.Sketch)
		extent AngularExtent
		shift  float64
		grow   float64
	}{
		{
			// The 10 × 8 rectangle on the axis: a solid cylinder.
			name: "rectangle on the axis, full turn",
			build: func(s *sketch.Sketch) {
				rect := s.CreateRectangle(0, 0, 10, 8)
				s.Fix(rect.A)
			},
			extent: FullRevolution{},
		},
		{
			name: "rectangle off the axis, half turn",
			build: func(s *sketch.Sketch) {
				rect := s.CreateRectangle(2, 3, 10, 8)
				s.Fix(rect.A)
			},
			extent: halfTurn,
		},
		{
			// A rectangle centred on z = 0, shifted along the axis: its
			// denoted centroid sits at z = 0 on the axis, so the volume's own
			// charge carries nothing into the axial quotient, a full turn has
			// no in-plane term, and the mixed moment's charge alone covers the
			// shift.
			name: "rectangle shifted along the axis, full turn",
			build: func(s *sketch.Sketch) {
				rect := s.CreateRectangle(-5, 2, 5, 6)
				s.Fix(rect.A)
			},
			extent: FullRevolution{},
			shift:  1e-3,
		},
		{
			// The half disk of radius 5 about (5, 0): a half sphere.
			name: "half disk, half turn",
			build: func(s *sketch.Sketch) {
				a := s.CreatePoint(0, 0)
				s.Fix(a)
				b := s.CreatePoint(10, 0)
				c := s.CreatePoint(5, 0)
				s.CreateArc(c, a, b)
				s.CreateLine(b, a)
			},
			extent: halfTurn,
		},
		{
			// A whole circle of radius 3 about (5, 10): a torus.
			name: "circle, full turn",
			build: func(s *sketch.Sketch) {
				c := s.CreatePoint(5, 10)
				s.Fix(c)
				s.CreateCircle(c, 3)
			},
			extent: FullRevolution{},
		},
		{
			// The same circle with its radius grown and its centre held, half
			// a turn: each cap's copy of the circle lengthens by 2π times the
			// growth, more than the two end charges' chord allows, so the
			// cap edges read the arc's own length charge.
			name: "circle with a grown radius, half turn",
			build: func(s *sketch.Sketch) {
				c := s.CreatePoint(5, 10)
				s.Fix(c)
				s.CreateCircle(c, 3)
			},
			extent: halfTurn,
			grow:   1e-3,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			ref := sectionSketchBody(t, tc.build, tc.extent)
			got, delta := displacedRevolve(t, ref, tc.shift, tc.grow)
			require.Positive(t, delta)

			gotVol, err := got.Volume()
			require.NoError(t, err)
			wantVol, err := ref.Volume()
			require.NoError(t, err)
			require.NotEqual(t, Exact, gotVol.Exactness)
			requireEncloses(t, "volume", gotVol, wantVol)

			gotArea, _ := got.Area()
			wantArea, _ := ref.Area()
			requireEncloses(t, "area", gotArea, wantArea)

			gotCen, err := got.Centroid()
			require.NoError(t, err)
			wantCen, err := ref.Centroid()
			require.NoError(t, err)
			requireVecEncloses(t, "centroid", gotCen.Value, wantCen.Value, gotCen.Bound.Base(), wantCen.Bound.Base())

			gotBox, _ := got.Bounds()
			wantBox, _ := ref.Bounds()
			requireVecEncloses(t, "bounds min", gotBox.Min, wantBox.Min, math.Sqrt(3)*gotBox.Bound.Base(), math.Sqrt(3)*wantBox.Bound.Base())
			requireVecEncloses(t, "bounds max", gotBox.Max, wantBox.Max, math.Sqrt(3)*gotBox.Bound.Base(), math.Sqrt(3)*wantBox.Bound.Base())

			gotFaces, wantFaces := got.Faces(), ref.Faces()
			require.Len(t, gotFaces, len(wantFaces))
			for i := range wantFaces {
				ga, err := gotFaces[i].Area()
				require.NoError(t, err)
				wa, err := wantFaces[i].Area()
				require.NoError(t, err)
				requireEncloses(t, fmt.Sprintf("face %d area", i), ga, wa)
			}

			gotEdges, wantEdges := got.Edges(), ref.Edges()
			require.Len(t, gotEdges, len(wantEdges))
			for i := range wantEdges {
				gl, err := gotEdges[i].Length()
				require.NoError(t, err)
				wl, err := wantEdges[i].Length()
				require.NoError(t, err)
				requireEncloses(t, fmt.Sprintf("edge %d length", i), gl, wl)
			}

			gotVerts, wantVerts := got.Vertices(), ref.Vertices()
			require.Len(t, gotVerts, len(wantVerts))
			for i := range wantVerts {
				gp, wp := gotVerts[i].Position(), wantVerts[i].Position()
				requireVecEncloses(t, fmt.Sprintf("vertex %d", i), gp.Value, wp.Value, gp.Bound.Base(), wp.Bound.Base())
			}
		})
	}
}

func TestRevolveSectionDisplacementNormal(t *testing.T) {
	t.Parallel()
	// The half sphere's wall: the recorded centre (5k, 0) sits 5·1e-4 from the
	// denoted one (5, 0), so the normal at a point of the recorded sphere,
	// taken against the recorded centre, misses the denoted normal there by
	// about 1e-4 — the widened denotation is what the bound must cover.
	ref := sectionSketchBody(t, func(s *sketch.Sketch) {
		a := s.CreatePoint(0, 0)
		s.Fix(a)
		b := s.CreatePoint(10, 0)
		c := s.CreatePoint(5, 0)
		s.CreateArc(c, a, b)
		s.CreateLine(b, a)
	}, AngleExtent{A: units.Degrees(180), Dir: Along})
	got, _ := displacedRevolve(t, ref, 0, 0)
	var wall *Face
	for _, f := range got.Faces() {
		if _, ok := f.Surface().(Sphere); ok {
			wall = f
		}
	}
	require.NotNil(t, wall)
	k := 1 + revolveSectionScale
	centre := r3.NewVec(5*k, 0, 0)
	for _, dir := range []r3.Vec{r3.NewVec(0.6, 0.8, 0), r3.NewVec(-0.6, 0, 0.8), r3.NewVec(0, 0.6, 0.8)} {
		p := centre.Add(dir.Scale(5 * k))
		n, err := wall.NormalAt(p)
		require.NoError(t, err)
		truth, ok := p.Sub(r3.NewVec(5, 0, 0)).Normalize()
		require.True(t, ok)
		require.LessOrEqual(t, n.Value.Sub(truth).Len(), n.Bound.Base(), "normal at %v: %v ± %v misses the denoted %v", p, n.Value, n.Bound, truth)
	}
}

func TestRevolveSectionDisplacementTessellates(t *testing.T) {
	t.Parallel()
	// The circle of radius 3 about (5, 10) recorded at radius 3.1, half a
	// turn: every mesh vertex sits on the recorded torus, 0.1 off the denoted
	// one, and a 0.13 tolerance leaves the chording only 0.03 of it, so the
	// face bounds hold the denoted torus only because they carry the
	// displacement.
	ref := sectionSketchBody(t, func(s *sketch.Sketch) {
		c := s.CreatePoint(5, 10)
		s.Fix(c)
		s.CreateCircle(c, 3)
	}, AngleExtent{A: units.Degrees(180), Dir: Along})
	const grow = 0.1
	got, delta := displacedRevolve(t, ref, 0, grow)
	// A tolerance under the displacement leaves no chord budget.
	_, err := got.Tessellate(t.Context(), units.Millimeters(delta/2))
	require.ErrorIs(t, err, ErrUnsupported)
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.13))
	require.NoError(t, err)
	for _, v := range mesh.vertices {
		// The revolve axis is world x, so z = v.X and ρ = hypot(v.Y, v.Z).
		off := math.Abs(math.Hypot(v.X-5, math.Hypot(v.Y, v.Z)-10) - 3)
		require.LessOrEqual(t, off, mesh.bound, "vertex %v sits %v off the denoted torus", v, off)
	}
	// Its area slack encloses the DENOTED solid's area.
	area, err := ref.Area()
	require.NoError(t, err)
	meshArea := 0.0
	for _, tri := range mesh.triangles {
		a, b, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
		meshArea += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	require.LessOrEqual(t, math.Abs(meshArea-area.Value.Base()), mesh.areaSlack+area.Bound.Base())
	// The mesh's occupied-volume bound encloses the DENOTED solid's volume.
	vol, err := ref.Volume()
	require.NoError(t, err)
	require.True(t, mesh.symDiffOK)
	require.LessOrEqual(t, math.Abs(internalMeshVolume(mesh)-vol.Value.Base()), mesh.volSymDiff+vol.Bound.Base())
}

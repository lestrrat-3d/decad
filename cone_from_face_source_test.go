package decad

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestPointConeSourceGates builds a real cone through ConeFromFace, then breaks
// one fact of its recorded meridian or faces at a time. Each break must make
// revolveplan.PointConeSource refuse: it reads the exact apex, far plane and
// radius from the authenticated meridian and nothing else. Three gates cannot
// be tested alone because others imply them: three segments, four axis points
// and two rim points are one fact (every point lands in one of the counts),
// a point below the axis leaves the counts short, and a nonpositive radius
// contradicts the two rim points. A non-revolve payload reads as a zero one,
// which the Full gate refuses.
func TestPointConeSourceGates(t *testing.T) {
	section := coneFromFaceSpiralSection()
	blank := spiralTestRevolveBlank(t, section)
	cone, err := ConeFromFace(t.Context(), blank, coneFromFaceSelector(t, blank, section[4], section[5]),
		units.Millimeters(-5))
	require.NoError(t, err)
	good := cone.n.Payload.(revolvePayload)
	limit, ok := revolveplan.PointConeSource(&cone.n)
	require.True(t, ok)

	// Every refusal edits a private copy of the meridian's segments.
	segments := func(p *revolvePayload) []sectionrecord.LineSeg {
		out := make([]sectionrecord.LineSeg, len(p.Profile.Outer.Segments))
		for i, s := range p.Profile.Outer.Segments {
			out[i] = s.(sectionrecord.LineSeg)
		}
		return out
	}
	setSegments := func(p *revolvePayload, lines []sectionrecord.LineSeg) {
		p.Profile.Outer.Segments = nil
		for _, l := range lines {
			p.Profile.Outer.Segments = append(p.Profile.Outer.Segments, l)
		}
	}
	// axisIndex is the first segment endpoint on the axis, and its coordinates
	// name the apex and far plane the other edits move.
	var apex, far, radius float64
	for _, l := range segments(&good) {
		for _, pt := range []planePoint{l.Start, l.End} {
			if pt.V == 0 {
				apex = max(apex, pt.U)
			}
			radius = max(radius, pt.V)
		}
	}
	far, _ = limit.Far.Float64()
	require.Positive(t, apex)
	require.Negative(t, far)
	require.Positive(t, radius)

	frameOf := func(t *testing.T, origin, u, v r3.Vec) r3.Frame {
		t.Helper()
		f, err := r3.NewFrame(origin, u, v)
		require.NoError(t, err)
		return f
	}
	move, err := r3.Translation(r3.Vec{X: 1})
	require.NoError(t, err)

	refusals := []struct {
		name string
		edit func(*revolvePayload)
	}{
		{"not a revolve", nil},
		{"partial revolution", func(p *revolvePayload) { p.Full = false }},
		{"surface result", func(p *revolvePayload) { p.SurfaceResult = true }},
		{"section displacement", func(p *revolvePayload) { p.SectionDelta = 1 }},
		{"placed", func(p *revolvePayload) { p.Xform = move }},
		{"profile with a hole", func(p *revolvePayload) {
			p.Profile.Holes = append(p.Profile.Holes, p.Profile.Outer)
		}},
		{"four segments", func(p *revolvePayload) {
			p.Profile.Outer.Segments = append(p.Profile.Outer.Segments, p.Profile.Outer.Segments[0])
		}},
		{"frame off the origin", func(p *revolvePayload) {
			p.Frame = frameOf(t, r3.Vec{X: 1}, r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		}},
		{"frame U not X", func(p *revolvePayload) {
			p.Frame = frameOf(t, r3.Vec{}, r3.NewVec(0, 0, 1), r3.NewVec(0, 1, 0))
		}},
		{"frame V not Y", func(p *revolvePayload) {
			p.Frame = frameOf(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1))
		}},
		{"axis origin U", func(p *revolvePayload) { p.Ax.AU = 1 }},
		{"axis origin V", func(p *revolvePayload) { p.Ax.AV = 1 }},
		{"axis direction U", func(p *revolvePayload) { p.Ax.DU = 0.5 }},
		{"axis direction V", func(p *revolvePayload) { p.Ax.DV = 1 }},
		{"axis origin U bound", func(p *revolvePayload) { p.Ax.AUBound = 1 }},
		{"axis origin V bound", func(p *revolvePayload) { p.Ax.AVBound = 1 }},
		{"axis direction U bound", func(p *revolvePayload) { p.Ax.DUBound = 1 }},
		{"axis direction V bound", func(p *revolvePayload) { p.Ax.DVBound = 1 }},
		{"non-line segment", func(p *revolvePayload) {
			p.Profile.Outer.Segments[0] = sectionrecord.ArcSeg{}
		}},
		{"point below the axis", func(p *revolvePayload) {
			lines := segments(p)
			for i := range lines {
				if lines[i].Start.V > 0 {
					lines[i].Start.V = -lines[i].Start.V
					break
				}
			}
			setSegments(p, lines)
		}},
		{"far plane at or past the origin", func(p *revolvePayload) {
			lines := segments(p)
			for i := range lines {
				lines[i].Start.U += 10 * (apex - far)
				lines[i].End.U += 10 * (apex - far)
			}
			setSegments(p, lines)
		}},
		{"apex at the origin", func(p *revolvePayload) {
			lines := segments(p)
			for i := range lines {
				lines[i].Start.U -= apex
				lines[i].End.U -= apex
			}
			setSegments(p, lines)
		}},
		{"axis point between apex and far plane", func(p *revolvePayload) {
			lines := segments(p)
			for i := range lines {
				if lines[i].Start.V == 0 && lines[i].Start.U == apex {
					lines[i].Start.U = (apex + far) / 2
					break
				}
			}
			setSegments(p, lines)
		}},
		{"rim point off the far plane", func(p *revolvePayload) {
			lines := segments(p)
			for i := range lines {
				if lines[i].End.V == radius {
					lines[i].End.U = far / 2
					break
				}
			}
			setSegments(p, lines)
		}},
		{"rim point below the rim radius", func(p *revolvePayload) {
			lines := segments(p)
			for i := range lines {
				if lines[i].End.V == radius {
					lines[i].End.V = radius / 2
					break
				}
			}
			setSegments(p, lines)
		}},
	}
	for _, tc := range refusals {
		t.Run(tc.name, func(t *testing.T) {
			saved := cone.n.Payload
			defer func() { cone.n.Payload = saved }()
			if tc.edit == nil {
				cone.n.Payload = nil
			} else {
				p := good
				p.Profile.Outer.Segments = append([]sectionrecord.CurveSegment(nil), good.Profile.Outer.Segments...)
				p.Profile.Holes = append(p.Profile.Holes[:0:0], good.Profile.Holes...)
				tc.edit(&p)
				cone.n.Payload = p
			}
			_, ok := revolveplan.PointConeSource(&cone.n)
			require.False(t, ok)
		})
	}

	t.Run("no cone face", func(t *testing.T) {
		faces := topobuild.Faces(&cone.n)
		saved := make([]surfacegeom.Surface, len(faces))
		for i, f := range faces {
			saved[i] = f.Surface
			f.Surface = surfacegeom.Plane{}
		}
		defer func() {
			for i, f := range faces {
				f.Surface = saved[i]
			}
		}()
		_, ok := revolveplan.PointConeSource(&cone.n)
		require.False(t, ok)
	})
	t.Run("cone face without origins", func(t *testing.T) {
		faces := topobuild.Faces(&cone.n)
		saved := make([][]provenance.Ref, len(faces))
		for i, f := range faces {
			saved[i] = f.Origins
			f.Origins = nil
		}
		defer func() {
			for i, f := range faces {
				f.Origins = saved[i]
			}
		}()
		_, ok := revolveplan.PointConeSource(&cone.n)
		require.False(t, ok)
	})
	t.Run("reads the limit from the meridian and the cone face", func(t *testing.T) {
		a, f, r := proofarith.FloatRat(apex), proofarith.FloatRat(far), proofarith.FloatRat(radius)
		require.Zero(t, limit.Apex.Cmp(a))
		require.Zero(t, limit.Far.Cmp(f))
		require.Zero(t, limit.Slope.Cmp(new(big.Rat).Quo(new(big.Rat).Sub(a, f), r)))
		for _, face := range topobuild.Faces(&cone.n) {
			surface, ok := face.Surface.(surfacegeom.Cone)
			if !ok {
				continue
			}
			require.Equal(t, surface, limit.Surface)
			require.Equal(t, face.Origins, limit.Origins)
			require.Same(t, face.Denoted, limit.Denoted)
			require.Equal(t, face.Reversed, limit.Reversed)
			face.Reversed = !face.Reversed
			defer func() { face.Reversed = !face.Reversed }()
			flipped, ok := revolveplan.PointConeSource(&cone.n)
			require.True(t, ok)
			require.Equal(t, face.Reversed, flipped.Reversed)
			return
		}
		require.FailNow(t, "the cone has no cone face")
	})
	t.Run("cone face without a denotation", func(t *testing.T) {
		faces := topobuild.Faces(&cone.n)
		saved := make([]*surfacenormal.Revolved, len(faces))
		for i, f := range faces {
			saved[i] = f.Denoted
			f.Denoted = nil
		}
		defer func() {
			for i, f := range faces {
				f.Denoted = saved[i]
			}
		}()
		_, ok := revolveplan.PointConeSource(&cone.n)
		require.False(t, ok)
	})
}

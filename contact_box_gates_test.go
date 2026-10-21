package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/contactbox"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// Case names the contact admission gate tests share.
const (
	caseSurfaceResult       = "surface result"
	caseSectionDisplacement = "section displacement"
)

// TestContactBoxAdmissionGates builds a real box, then breaks one fact of its
// payload, kind or faces at a time. Each break must make both source-box
// readers refuse; the unbroken box must read as six mapped faces.
func TestContactBoxAdmissionGates(t *testing.T) {
	doc := New()
	body := internalBoxBody(t, doc, 0, 0, 10, 10, 10)
	good := body.n.Payload.(prismPayload)
	turn, err := r3.RotationAround(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(30))
	require.NoError(t, err)
	tilt, err := r3.NewFrame(r3.Vec{}, r3.NewVec(math.Cos(0.5), math.Sin(0.5), 0), r3.NewVec(-math.Sin(0.5), math.Cos(0.5), 0))
	require.NoError(t, err)

	box, ok := contactbox.SourceBoxAtPose(&body.n, r3.Identity())
	require.True(t, ok)
	for axis := range 3 {
		require.NotNil(t, box.Faces[axis][0])
		require.NotNil(t, box.Faces[axis][1])
		require.NotSame(t, box.Faces[axis][0], box.Faces[axis][1])
	}
	_, ok = contactbox.SourceOrientedBoxAtPose(&body.n, turn)
	require.True(t, ok)

	payloadEdits := []struct {
		name  string
		edit  func(*prismPayload)
		axial bool
	}{
		{caseSurfaceResult, func(p *prismPayload) { p.SurfaceResult = true }, true},
		{caseSectionDisplacement, func(p *prismPayload) { p.SectionDelta = 1 }, true},
		{"near level displacement", func(p *prismPayload) { p.Z0Delta = 1 }, true},
		{"far level displacement", func(p *prismPayload) { p.Z1Delta = 1 }, true},
		{"placed off the signed axes", func(p *prismPayload) { p.Xform = turn }, true},
		{"frame off the cardinal axes", func(p *prismPayload) { p.Frame = tilt }, false},
		{"non-finite level", func(p *prismPayload) { p.Z1 = math.Inf(1) }, false},
		{"empty sweep", func(p *prismPayload) { p.Z1 = p.Z0 }, false},
		{"non-line segment", func(p *prismPayload) {
			segs := append([]sectionrecord.CurveSegment(nil), p.Profile.Outer.Segments...)
			segs[0] = sectionrecord.ArcSeg{}
			p.Profile.Outer.Segments = segs
		}, false},
		{"non-finite corner", func(p *prismPayload) {
			segs := append([]sectionrecord.CurveSegment(nil), p.Profile.Outer.Segments...)
			line := segs[1].(sectionrecord.LineSeg)
			line.Start.U = math.NaN()
			segs[1] = line
			p.Profile.Outer.Segments = segs
		}, false},
	}
	for _, tc := range payloadEdits {
		t.Run(tc.name, func(t *testing.T) {
			saved := body.n.Payload
			defer func() { body.n.Payload = saved }()
			p := good
			tc.edit(&p)
			body.n.Payload = p
			_, ok := contactbox.SourceBoxAtPose(&body.n, r3.Identity())
			require.False(t, ok)
			if tc.axial {
				_, ok = contactbox.SourceOrientedBoxAtPose(&body.n, turn)
				require.False(t, ok)
			}
		})
	}

	t.Run("pose off the signed axes", func(t *testing.T) {
		_, ok := contactbox.SourceBoxAtPose(&body.n, turn)
		require.False(t, ok)
	})
	t.Run("not a prism", func(t *testing.T) {
		saved := body.n.Payload
		defer func() { body.n.Payload = saved }()
		body.n.Payload = nil
		_, ok := contactbox.SourceBoxAtPose(&body.n, r3.Identity())
		require.False(t, ok)
		_, ok = contactbox.SourceOrientedBoxAtPose(&body.n, turn)
		require.False(t, ok)
	})
	t.Run("not a solid", func(t *testing.T) {
		body.n.Solid = false
		defer func() { body.n.Solid = true }()
		_, ok := contactbox.SourceBoxAtPose(&body.n, r3.Identity())
		require.False(t, ok)
		_, ok = contactbox.SourceOrientedBoxAtPose(&body.n, turn)
		require.False(t, ok)
	})
	t.Run("sheet kind", func(t *testing.T) {
		kind := body.n.Kind
		body.n.Kind = topo.BodySheet
		defer func() { body.n.Kind = kind }()
		_, ok := contactbox.SourceBoxAtPose(&body.n, r3.Identity())
		require.False(t, ok)
		_, ok = contactbox.SourceOrientedBoxAtPose(&body.n, turn)
		require.False(t, ok)
	})

	faces := topobuild.Faces(&body.n)
	faceEdits := []struct {
		name string
		edit func(*topo.Face) func()
	}{
		{"curved face", func(f *topo.Face) func() {
			s := f.Surface
			f.Surface = surfacegeom.Faceted{}
			return func() { f.Surface = s }
		}},
		{"held normal", func(f *topo.Face) func() {
			f.NormalBound = 1
			return func() { f.NormalBound = 0 }
		}},
		{"flipped face sharing a support slot", func(f *topo.Face) func() {
			f.Reversed = !f.Reversed
			return func() { f.Reversed = !f.Reversed }
		}},
	}
	for _, tc := range faceEdits {
		t.Run(tc.name, func(t *testing.T) {
			restore := tc.edit(faces[0])
			defer restore()
			_, ok := contactbox.SourceBoxAtPose(&body.n, r3.Identity())
			require.False(t, ok)
			_, ok = contactbox.SourceOrientedBoxAtPose(&body.n, turn)
			require.False(t, ok)
		})
	}
	t.Run("five faces", func(t *testing.T) {
		shell := body.n.Lumps[0].Shells[0]
		saved := shell.Faces
		shell.Faces = saved[:5]
		defer func() { shell.Faces = saved }()
		_, ok := contactbox.SourceBoxAtPose(&body.n, r3.Identity())
		require.False(t, ok)
		_, ok = contactbox.SourceOrientedBoxAtPose(&body.n, turn)
		require.False(t, ok)
	})
}
